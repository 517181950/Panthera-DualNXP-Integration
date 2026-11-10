/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/bulk.h"

#include <stdbool.h>
#include <string.h>

#define WL_BULK_RECEIVER_MAGIC UINT32_C(0x42554c52)
#define WL_BULK_MAX_INTERVAL UINT32_C(0x80000000)

typedef struct {
  wl_bulk_receiver_config_t config;
  wl_bulk_descriptor_t descriptor;
  wl_bulk_status_t pending_status;
  wl_bulk_receiver_stats_t stats;
  uint64_t next_offset;
  wl_time_ms_t last_activity_ms;
  uint32_t status_token;
  uint32_t accepted_chunk_size;
  uint32_t magic;
  wl_bulk_receiver_state_t state;
  wl_bulk_status_code_t terminal_code;
  wl_bulk_phase_t terminal_phase;
  bool status_pending;
  bool status_acquired;
  bool sink_active;
} wl_bulk_receiver_impl_t;

_Static_assert(sizeof(wl_bulk_receiver_impl_t) <= WL_BULK_RECEIVER_STORAGE_SIZE,
               "WL_BULK_RECEIVER_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_bulk_receiver_impl_t) <=
                   _Alignof(wl_bulk_receiver_t),
               "wl_bulk_receiver_t alignment is too small");

static wl_bulk_receiver_impl_t *receiver_impl(wl_bulk_receiver_t *receiver) {
  return (wl_bulk_receiver_impl_t *)(void *)receiver;
}

static const wl_bulk_receiver_impl_t *
receiver_impl_const(const wl_bulk_receiver_t *receiver) {
  return (const wl_bulk_receiver_impl_t *)(const void *)receiver;
}

static bool receiver_initialized(const wl_bulk_receiver_t *receiver) {
  return receiver != NULL &&
         receiver_impl_const(receiver)->magic == WL_BULK_RECEIVER_MAGIC;
}

static bool power_of_two(uint32_t value) {
  return value != 0U && (value & (value - 1U)) == 0U;
}

static bool interval_valid(uint32_t interval_ms) {
  return interval_ms < WL_BULK_MAX_INTERVAL;
}

static bool interval_elapsed(wl_time_ms_t now_ms, wl_time_ms_t started_at,
                             uint32_t interval_ms) {
  return interval_ms != 0U && (uint32_t)(now_ms - started_at) >= interval_ms;
}

static uint32_t interval_remaining(wl_time_ms_t now_ms, wl_time_ms_t started_at,
                                   uint32_t interval_ms) {
  const uint32_t age = (uint32_t)(now_ms - started_at);

  return age >= interval_ms ? 0U : interval_ms - age;
}

static void counter_increment(uint32_t *counter) {
  if (*counter != UINT32_MAX) {
    ++*counter;
  }
}

static void counter_add(uint32_t *counter, uint32_t increment) {
  if (UINT32_MAX - *counter < increment) {
    *counter = UINT32_MAX;
  } else {
    *counter += increment;
  }
}

static uint32_t next_token(uint32_t token) {
  ++token;
  return token == 0U ? 1U : token;
}

static bool descriptor_equal(const wl_bulk_descriptor_t *left,
                             const wl_bulk_descriptor_t *right) {
  return left->transfer_id == right->transfer_id &&
         left->total_length == right->total_length &&
         left->requested_chunk_size == right->requested_chunk_size &&
         left->object_crc32c == right->object_crc32c;
}

static bool status_equal(const wl_bulk_status_t *left,
                         const wl_bulk_status_t *right) {
  return left->transfer_id == right->transfer_id &&
         left->phase == right->phase && left->code == right->code &&
         left->next_offset == right->next_offset &&
         left->accepted_chunk_size == right->accepted_chunk_size;
}

static void retain_status(wl_bulk_receiver_impl_t *impl, uint32_t transfer_id,
                          wl_bulk_phase_t phase, wl_bulk_status_code_t code,
                          uint64_t next_offset, uint32_t accepted_chunk_size) {
  impl->pending_status = (wl_bulk_status_t){
      .transfer_id = transfer_id,
      .phase = phase,
      .code = code,
      .next_offset = next_offset,
      .accepted_chunk_size = accepted_chunk_size,
  };
  impl->status_token = next_token(impl->status_token);
  impl->status_pending = true;
  impl->status_acquired = false;
}

static wl_bulk_err_t
reject_protocol(wl_bulk_receiver_impl_t *impl, uint32_t transfer_id,
                wl_bulk_phase_t phase, wl_bulk_status_code_t code,
                uint64_t next_offset, uint32_t accepted_chunk_size) {
  counter_increment(&impl->stats.protocol_errors);
  retain_status(impl, transfer_id, phase, code, next_offset,
                accepted_chunk_size);
  return WL_BULK_OK;
}

static wl_bulk_err_t require_input_slot(wl_bulk_receiver_impl_t *impl) {
  return impl->status_pending ? WL_BULK_ERR_BUSY : WL_BULK_OK;
}

static bool negotiate_chunk_size(const wl_bulk_receiver_impl_t *impl,
                                 const wl_bulk_descriptor_t *descriptor,
                                 uint32_t *out_chunk_size) {
  uint32_t limit;
  uint32_t accepted;

  if (descriptor->transfer_id == 0U ||
      descriptor->total_length > impl->config.max_object_length ||
      descriptor->requested_chunk_size == 0U) {
    return false;
  }

  limit = descriptor->requested_chunk_size;
  if (limit > impl->config.max_chunk_size) {
    limit = impl->config.max_chunk_size;
  }
  accepted = limit - (limit % impl->config.write_alignment);
  if (accepted == 0U) {
    /* A sub-alignment chunk can only represent the sole, final span. */
    if (descriptor->total_length > (uint64_t)limit) {
      return false;
    }
    accepted = limit;
  }

  *out_chunk_size = accepted;
  return true;
}

static bool resume_offset_valid(const wl_bulk_receiver_impl_t *impl,
                                uint64_t resume_offset) {
  if (resume_offset > impl->descriptor.total_length) {
    return false;
  }
  return resume_offset == impl->descriptor.total_length ||
         resume_offset % (uint64_t)impl->config.write_alignment == 0U;
}

static wl_bulk_status_code_t
sink_failure_code(wl_bulk_receiver_impl_t *impl,
                  wl_bulk_sink_result_t sink_result) {
  switch (sink_result) {
  case WL_BULK_SINK_WRITE_FAILED:
    counter_increment(&impl->stats.write_failures);
    return WL_BULK_STATUS_WRITE_FAILED;
  case WL_BULK_SINK_INTEGRITY_FAILED:
    counter_increment(&impl->stats.integrity_failures);
    return WL_BULK_STATUS_INTEGRITY_FAILED;
  case WL_BULK_SINK_INVALID:
    return WL_BULK_STATUS_INVALID;
  default:
    counter_increment(&impl->stats.write_failures);
    return WL_BULK_STATUS_WRITE_FAILED;
  }
}

static void enter_failure(wl_bulk_receiver_impl_t *impl, wl_bulk_phase_t phase,
                          wl_bulk_status_code_t code) {
  impl->state = WL_BULK_RECEIVER_FAILED;
  impl->terminal_phase = phase;
  impl->terminal_code = code;
}

static void release_sink(wl_bulk_receiver_impl_t *impl, int32_t reason) {
  if (impl->sink_active) {
    impl->config.sink.abort(impl->config.sink.user_data,
                            impl->descriptor.transfer_id, reason);
    impl->sink_active = false;
  }
}

static void clear_session(wl_bulk_receiver_impl_t *impl, int32_t reason) {
  release_sink(impl, reason);
  impl->descriptor = (wl_bulk_descriptor_t){0};
  impl->next_offset = 0U;
  impl->last_activity_ms = 0U;
  impl->accepted_chunk_size = 0U;
  impl->state = WL_BULK_RECEIVER_IDLE;
  impl->terminal_code = WL_BULK_STATUS_OK;
  impl->terminal_phase = WL_BULK_PHASE_NONE;
}

static void retain_abort_tombstone(wl_bulk_receiver_impl_t *impl,
                                   uint32_t transfer_id,
                                   wl_time_ms_t now_ms) {
  clear_session(impl, WL_BULK_ERR_INVALID_STATE);
  impl->descriptor.transfer_id = transfer_id;
  impl->last_activity_ms = now_ms;
  impl->state = WL_BULK_RECEIVER_ABORTED;
  impl->terminal_phase = WL_BULK_PHASE_ABORT;
  impl->terminal_code = WL_BULK_STATUS_ABORTED;
  counter_increment(&impl->stats.aborts);
  retain_status(impl, transfer_id, WL_BULK_PHASE_ABORT,
                WL_BULK_STATUS_ABORTED, 0U, 0U);
}

static uint64_t active_next_offset(const wl_bulk_receiver_impl_t *impl) {
  return impl->state == WL_BULK_RECEIVER_IDLE ? 0U : impl->next_offset;
}

static uint32_t active_chunk_size(const wl_bulk_receiver_impl_t *impl) {
  return impl->state == WL_BULK_RECEIVER_IDLE ? 0U : impl->accepted_chunk_size;
}

static wl_bulk_status_code_t
terminal_status_code(const wl_bulk_receiver_impl_t *impl) {
  if (impl->state == WL_BULK_RECEIVER_COMPLETED) {
    return WL_BULK_STATUS_OK;
  }
  if (impl->state == WL_BULK_RECEIVER_ABORTED) {
    return WL_BULK_STATUS_ABORTED;
  }
  return impl->terminal_code;
}

static bool chunk_shape_valid(const wl_bulk_receiver_impl_t *impl,
                              const wl_bulk_chunk_t *chunk, uint64_t *out_end) {
  uint64_t length;
  uint64_t end;

  if (chunk->transfer_id == 0U || chunk->data == NULL || chunk->length == 0U ||
      chunk->length > (size_t)impl->accepted_chunk_size) {
    return false;
  }
  length = (uint64_t)chunk->length;
  if (chunk->offset > UINT64_MAX - length) {
    return false;
  }
  end = chunk->offset + length;
  if (end > impl->descriptor.total_length ||
      chunk->offset % (uint64_t)impl->config.write_alignment != 0U) {
    return false;
  }
  if (end != impl->descriptor.total_length &&
      length % (uint64_t)impl->config.write_alignment != 0U) {
    return false;
  }

  *out_end = end;
  return true;
}

const char *wl_bulk_err_str(wl_bulk_err_t error) {
  switch (error) {
  case WL_BULK_OK:
    return "ok";
  case WL_BULK_ERR_INVALID_ARG:
    return "invalid argument";
  case WL_BULK_ERR_NOT_INITIALIZED:
    return "not initialized";
  case WL_BULK_ERR_BUSY:
    return "busy";
  case WL_BULK_ERR_INVALID_STATE:
    return "invalid state";
  case WL_BULK_ERR_NOT_FOUND:
    return "not found";
  case WL_BULK_ERR_PROTOCOL:
    return "protocol error";
  case WL_BULK_ERR_TIMEOUT:
    return "timed out";
  default:
    return "unknown bulk error";
  }
}

wl_bulk_err_t wl_bulk_receiver_init(wl_bulk_receiver_t *receiver,
                                    const wl_bulk_receiver_config_t *config) {
  wl_bulk_receiver_impl_t *impl;

  if (receiver == NULL || config == NULL || config->max_object_length == 0U ||
      config->max_chunk_size == 0U || !power_of_two(config->write_alignment) ||
      config->max_chunk_size < config->write_alignment ||
      !interval_valid(config->idle_timeout_ms) || config->sink.begin == NULL ||
      config->sink.write == NULL || config->sink.finish == NULL ||
      config->sink.abort == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }

  memset(receiver, 0, sizeof(*receiver));
  impl = receiver_impl(receiver);
  impl->config = *config;
  impl->state = WL_BULK_RECEIVER_IDLE;
  impl->terminal_code = WL_BULK_STATUS_OK;
  impl->terminal_phase = WL_BULK_PHASE_NONE;
  impl->magic = WL_BULK_RECEIVER_MAGIC;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_reset(wl_bulk_receiver_t *receiver) {
  wl_bulk_receiver_impl_t *impl;

  if (receiver == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (impl->status_acquired) {
    return WL_BULK_ERR_BUSY;
  }

  clear_session(impl, WL_BULK_ERR_INVALID_STATE);
  impl->pending_status = (wl_bulk_status_t){0};
  impl->status_pending = false;
  impl->status_acquired = false;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_on_begin(wl_bulk_receiver_t *receiver,
                                        const wl_bulk_descriptor_t *descriptor,
                                        wl_time_ms_t now_ms) {
  wl_bulk_receiver_impl_t *impl;
  wl_bulk_sink_result_t sink_result;
  wl_bulk_status_code_t code;
  uint64_t resume_offset = 0U;
  uint32_t accepted_chunk_size = 0U;

  if (receiver == NULL || descriptor == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (require_input_slot(impl) != WL_BULK_OK) {
    return WL_BULK_ERR_BUSY;
  }
  if (!negotiate_chunk_size(impl, descriptor, &accepted_chunk_size)) {
    return reject_protocol(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN,
                           WL_BULK_STATUS_INVALID, active_next_offset(impl),
                           active_chunk_size(impl));
  }

  if (impl->state != WL_BULK_RECEIVER_IDLE) {
    if (descriptor_equal(&impl->descriptor, descriptor)) {
      if (impl->state == WL_BULK_RECEIVER_RECEIVING) {
        impl->last_activity_ms = now_ms;
      }
      counter_increment(&impl->stats.duplicate_messages);
      retain_status(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN,
                    terminal_status_code(impl), impl->next_offset,
                    impl->accepted_chunk_size);
      return WL_BULK_OK;
    }
    if (impl->state == WL_BULK_RECEIVER_RECEIVING ||
        descriptor->transfer_id == impl->descriptor.transfer_id) {
      return reject_protocol(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN,
                             WL_BULK_STATUS_CONFLICT, impl->next_offset,
                             impl->accepted_chunk_size);
    }
    /*
     * The old terminal record remains authoritative until the new sink
     * actually accepts or rejects Begin. In particular, BUSY must not erase
     * an Abort tombstone and allow a delayed old Begin to resurrect.
     */
    release_sink(impl, WL_BULK_ERR_INVALID_STATE);
  }

  sink_result = impl->config.sink.begin(impl->config.sink.user_data, descriptor,
                                        &resume_offset);
  if (sink_result == WL_BULK_SINK_BUSY) {
    counter_increment(&impl->stats.busy_responses);
    retain_status(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN,
                  WL_BULK_STATUS_BUSY, 0U, accepted_chunk_size);
    return WL_BULK_OK;
  }

  impl->descriptor = *descriptor;
  impl->accepted_chunk_size = accepted_chunk_size;
  impl->last_activity_ms = now_ms;
  impl->sink_active = true;
  if (sink_result == WL_BULK_SINK_OK) {
    if (!resume_offset_valid(impl, resume_offset)) {
      code = WL_BULK_STATUS_INVALID;
      enter_failure(impl, WL_BULK_PHASE_BEGIN, code);
      retain_status(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN, code,
                    0U, accepted_chunk_size);
      return WL_BULK_OK;
    }
    impl->next_offset = resume_offset;
    impl->state = WL_BULK_RECEIVER_RECEIVING;
    counter_increment(&impl->stats.begins);
    retain_status(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN,
                  WL_BULK_STATUS_OK, resume_offset, accepted_chunk_size);
    return WL_BULK_OK;
  }

  code = sink_failure_code(impl, sink_result);
  enter_failure(impl, WL_BULK_PHASE_BEGIN, code);
  retain_status(impl, descriptor->transfer_id, WL_BULK_PHASE_BEGIN, code, 0U,
                accepted_chunk_size);
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_on_chunk(wl_bulk_receiver_t *receiver,
                                        const wl_bulk_chunk_t *chunk,
                                        wl_time_ms_t now_ms) {
  wl_bulk_receiver_impl_t *impl;
  wl_bulk_sink_result_t sink_result;
  wl_bulk_status_code_t code;
  uint64_t end = 0U;

  if (receiver == NULL || chunk == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (require_input_slot(impl) != WL_BULK_OK) {
    return WL_BULK_ERR_BUSY;
  }
  if (chunk->transfer_id == 0U || chunk->data == NULL || chunk->length == 0U) {
    return reject_protocol(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                           WL_BULK_STATUS_INVALID, active_next_offset(impl),
                           active_chunk_size(impl));
  }
  if (impl->state == WL_BULK_RECEIVER_IDLE) {
    return reject_protocol(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                           WL_BULK_STATUS_OUT_OF_ORDER, 0U, 0U);
  }
  if (chunk->transfer_id != impl->descriptor.transfer_id) {
    return reject_protocol(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                           WL_BULK_STATUS_CONFLICT, impl->next_offset,
                           impl->accepted_chunk_size);
  }
  if (!chunk_shape_valid(impl, chunk, &end)) {
    return reject_protocol(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                           WL_BULK_STATUS_INVALID, impl->next_offset,
                           impl->accepted_chunk_size);
  }

  if (end <= impl->next_offset) {
    impl->last_activity_ms = now_ms;
    counter_increment(&impl->stats.duplicate_messages);
    retain_status(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                  terminal_status_code(impl), impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }
  if (chunk->offset != impl->next_offset) {
    return reject_protocol(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                           WL_BULK_STATUS_OUT_OF_ORDER, impl->next_offset,
                           impl->accepted_chunk_size);
  }
  if (impl->state != WL_BULK_RECEIVER_RECEIVING) {
    retain_status(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                  terminal_status_code(impl), impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }

  sink_result =
      impl->config.sink.write(impl->config.sink.user_data, chunk->transfer_id,
                              chunk->offset, chunk->data, chunk->length);
  impl->last_activity_ms = now_ms;
  if (sink_result == WL_BULK_SINK_BUSY) {
    counter_increment(&impl->stats.busy_responses);
    retain_status(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                  WL_BULK_STATUS_BUSY, impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }
  if (sink_result == WL_BULK_SINK_OK) {
    impl->next_offset = end;
    counter_increment(&impl->stats.chunks);
    counter_add(&impl->stats.bytes_written, (uint32_t)chunk->length);
    retain_status(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK,
                  WL_BULK_STATUS_OK, impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }

  code = sink_failure_code(impl, sink_result);
  enter_failure(impl, WL_BULK_PHASE_CHUNK, code);
  retain_status(impl, chunk->transfer_id, WL_BULK_PHASE_CHUNK, code,
                impl->next_offset, impl->accepted_chunk_size);
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_on_end(wl_bulk_receiver_t *receiver,
                                      uint32_t transfer_id,
                                      uint64_t total_length,
                                      uint32_t object_crc32c,
                                      wl_time_ms_t now_ms) {
  wl_bulk_receiver_impl_t *impl;
  wl_bulk_sink_result_t sink_result;
  wl_bulk_status_code_t code;

  if (receiver == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (require_input_slot(impl) != WL_BULK_OK) {
    return WL_BULK_ERR_BUSY;
  }
  if (transfer_id == 0U) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_END,
                           WL_BULK_STATUS_INVALID, active_next_offset(impl),
                           active_chunk_size(impl));
  }
  if (impl->state == WL_BULK_RECEIVER_IDLE) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_END,
                           WL_BULK_STATUS_OUT_OF_ORDER, 0U, 0U);
  }
  if (transfer_id != impl->descriptor.transfer_id ||
      total_length != impl->descriptor.total_length ||
      object_crc32c != impl->descriptor.object_crc32c) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_END,
                           WL_BULK_STATUS_CONFLICT, impl->next_offset,
                           impl->accepted_chunk_size);
  }
  if (impl->state == WL_BULK_RECEIVER_COMPLETED ||
      (impl->state == WL_BULK_RECEIVER_FAILED &&
       impl->terminal_phase == WL_BULK_PHASE_END)) {
    impl->last_activity_ms = now_ms;
    counter_increment(&impl->stats.duplicate_messages);
    retain_status(impl, transfer_id, WL_BULK_PHASE_END,
                  terminal_status_code(impl), impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }
  if (impl->state != WL_BULK_RECEIVER_RECEIVING) {
    retain_status(impl, transfer_id, WL_BULK_PHASE_END,
                  terminal_status_code(impl), impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }
  if (impl->next_offset != impl->descriptor.total_length) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_END,
                           WL_BULK_STATUS_OUT_OF_ORDER, impl->next_offset,
                           impl->accepted_chunk_size);
  }

  sink_result =
      impl->config.sink.finish(impl->config.sink.user_data, &impl->descriptor);
  impl->last_activity_ms = now_ms;
  if (sink_result == WL_BULK_SINK_BUSY) {
    counter_increment(&impl->stats.busy_responses);
    retain_status(impl, transfer_id, WL_BULK_PHASE_END, WL_BULK_STATUS_BUSY,
                  impl->next_offset, impl->accepted_chunk_size);
    return WL_BULK_OK;
  }
  if (sink_result == WL_BULK_SINK_OK) {
    impl->state = WL_BULK_RECEIVER_COMPLETED;
    impl->terminal_phase = WL_BULK_PHASE_END;
    impl->terminal_code = WL_BULK_STATUS_OK;
    impl->sink_active = false;
    retain_status(impl, transfer_id, WL_BULK_PHASE_END, WL_BULK_STATUS_OK,
                  impl->next_offset, impl->accepted_chunk_size);
    return WL_BULK_OK;
  }

  code = sink_failure_code(impl, sink_result);
  enter_failure(impl, WL_BULK_PHASE_END, code);
  retain_status(impl, transfer_id, WL_BULK_PHASE_END, code, impl->next_offset,
                impl->accepted_chunk_size);
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_on_abort(wl_bulk_receiver_t *receiver,
                                        uint32_t transfer_id, int32_t reason,
                                        wl_time_ms_t now_ms) {
  wl_bulk_receiver_impl_t *impl;

  if (receiver == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (require_input_slot(impl) != WL_BULK_OK) {
    return WL_BULK_ERR_BUSY;
  }
  if (transfer_id == 0U) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_ABORT,
                           WL_BULK_STATUS_INVALID, active_next_offset(impl),
                           active_chunk_size(impl));
  }
  if (impl->state == WL_BULK_RECEIVER_IDLE ||
      (impl->state != WL_BULK_RECEIVER_RECEIVING &&
       transfer_id != impl->descriptor.transfer_id)) {
    retain_abort_tombstone(impl, transfer_id, now_ms);
    return WL_BULK_OK;
  }
  if (transfer_id != impl->descriptor.transfer_id) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_ABORT,
                           WL_BULK_STATUS_CONFLICT, impl->next_offset,
                           impl->accepted_chunk_size);
  }
  if (impl->state == WL_BULK_RECEIVER_ABORTED) {
    impl->last_activity_ms = now_ms;
    counter_increment(&impl->stats.duplicate_messages);
    retain_status(impl, transfer_id, WL_BULK_PHASE_ABORT,
                  WL_BULK_STATUS_ABORTED, impl->next_offset,
                  impl->accepted_chunk_size);
    return WL_BULK_OK;
  }
  if (impl->state == WL_BULK_RECEIVER_COMPLETED) {
    return reject_protocol(impl, transfer_id, WL_BULK_PHASE_ABORT,
                           WL_BULK_STATUS_CONFLICT, impl->next_offset,
                           impl->accepted_chunk_size);
  }

  if (impl->sink_active) {
    impl->config.sink.abort(impl->config.sink.user_data, transfer_id, reason);
    impl->sink_active = false;
  }
  impl->last_activity_ms = now_ms;
  impl->state = WL_BULK_RECEIVER_ABORTED;
  impl->terminal_phase = WL_BULK_PHASE_ABORT;
  impl->terminal_code = WL_BULK_STATUS_ABORTED;
  counter_increment(&impl->stats.aborts);
  retain_status(impl, transfer_id, WL_BULK_PHASE_ABORT, WL_BULK_STATUS_ABORTED,
                impl->next_offset, impl->accepted_chunk_size);
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_receiver_status_acquire(wl_bulk_receiver_t *receiver,
                                wl_bulk_receiver_status_view_t *out_view) {
  wl_bulk_receiver_impl_t *impl;

  if (out_view != NULL) {
    *out_view = (wl_bulk_receiver_status_view_t){0};
  }
  if (receiver == NULL || out_view == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (!impl->status_pending) {
    return WL_BULK_ERR_NOT_FOUND;
  }
  if (impl->status_acquired) {
    return WL_BULK_ERR_BUSY;
  }

  out_view->status = impl->pending_status;
  out_view->token = impl->status_token;
  impl->status_acquired = true;
  return WL_BULK_OK;
}

static bool status_view_matches(const wl_bulk_receiver_impl_t *impl,
                                const wl_bulk_receiver_status_view_t *view) {
  return view != NULL && impl->status_pending && impl->status_acquired &&
         view->token == impl->status_token &&
         status_equal(&view->status, &impl->pending_status);
}

wl_bulk_err_t
wl_bulk_receiver_status_defer(wl_bulk_receiver_t *receiver,
                              const wl_bulk_receiver_status_view_t *view) {
  wl_bulk_receiver_impl_t *impl;

  if (receiver == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (!status_view_matches(impl, view)) {
    return WL_BULK_ERR_INVALID_STATE;
  }

  impl->status_acquired = false;
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_receiver_status_release(wl_bulk_receiver_t *receiver,
                                const wl_bulk_receiver_status_view_t *view) {
  wl_bulk_receiver_impl_t *impl;

  if (receiver == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (!status_view_matches(impl, view)) {
    return WL_BULK_ERR_INVALID_STATE;
  }

  impl->status_pending = false;
  impl->status_acquired = false;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_poll(wl_bulk_receiver_t *receiver,
                                    wl_time_ms_t now_ms) {
  wl_bulk_receiver_impl_t *impl;

  if (receiver == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl(receiver);
  if (impl->state != WL_BULK_RECEIVER_RECEIVING ||
      impl->config.idle_timeout_ms == 0U || impl->status_pending ||
      !interval_elapsed(now_ms, impl->last_activity_ms,
                        impl->config.idle_timeout_ms)) {
    return WL_BULK_OK;
  }

  if (impl->sink_active) {
    impl->config.sink.abort(impl->config.sink.user_data,
                            impl->descriptor.transfer_id, WL_BULK_ERR_TIMEOUT);
    impl->sink_active = false;
  }
  enter_failure(impl, WL_BULK_PHASE_ABORT, WL_BULK_STATUS_TIMED_OUT);
  counter_increment(&impl->stats.timeouts);
  retain_status(impl, impl->descriptor.transfer_id, WL_BULK_PHASE_ABORT,
                WL_BULK_STATUS_TIMED_OUT, impl->next_offset,
                impl->accepted_chunk_size);
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_receiver_get_deadline_hint(const wl_bulk_receiver_t *receiver,
                                   wl_time_ms_t now_ms,
                                   wl_bulk_deadline_hint_t *out_hint) {
  const wl_bulk_receiver_impl_t *impl;

  if (out_hint != NULL) {
    out_hint->next_deadline_ms = WL_BULK_NO_DEADLINE_MS;
  }
  if (receiver == NULL || out_hint == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl_const(receiver);
  if (impl->state == WL_BULK_RECEIVER_RECEIVING &&
      impl->config.idle_timeout_ms != 0U && !impl->status_pending) {
    out_hint->next_deadline_ms = interval_remaining(
        now_ms, impl->last_activity_ms, impl->config.idle_timeout_ms);
  }
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_get_state(const wl_bulk_receiver_t *receiver,
                                         wl_bulk_receiver_state_t *out_state,
                                         uint64_t *out_next_offset) {
  const wl_bulk_receiver_impl_t *impl;

  if (out_state != NULL) {
    *out_state = WL_BULK_RECEIVER_IDLE;
  }
  if (out_next_offset != NULL) {
    *out_next_offset = 0U;
  }
  if (receiver == NULL || out_state == NULL || out_next_offset == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl_const(receiver);
  *out_state = impl->state;
  *out_next_offset = impl->next_offset;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_receiver_get_stats(const wl_bulk_receiver_t *receiver,
                                         wl_bulk_receiver_stats_t *out_stats) {
  const wl_bulk_receiver_impl_t *impl;

  if (out_stats != NULL) {
    *out_stats = (wl_bulk_receiver_stats_t){0};
  }
  if (receiver == NULL || out_stats == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!receiver_initialized(receiver)) {
    return WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = receiver_impl_const(receiver);
  *out_stats = impl->stats;
  return WL_BULK_OK;
}

#ifdef WL_BULK_TEST_HOOKS
void wl_bulk_receiver_test_seed_stats(wl_bulk_receiver_t *receiver,
                                      const wl_bulk_receiver_stats_t *stats);

void wl_bulk_receiver_test_seed_stats(wl_bulk_receiver_t *receiver,
                                      const wl_bulk_receiver_stats_t *stats) {
  receiver_impl(receiver)->stats = *stats;
}
#endif
