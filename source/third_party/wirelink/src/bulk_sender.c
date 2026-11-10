/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/bulk.h"

#include <stdbool.h>
#include <string.h>

#define WL_BULK_SENDER_MAGIC UINT32_C(0x42554c53)
#define WL_BULK_MAX_INTERVAL UINT32_C(0x80000000)

typedef struct {
  wl_bulk_descriptor_t descriptor;
  wl_bulk_sender_stats_t stats;
  uint64_t next_offset;
  size_t action_length;
  wl_time_ms_t wait_started_at;
  uint32_t wait_interval_ms;
  uint32_t status_timeout_ms;
  uint32_t busy_retry_ms;
  uint32_t next_token;
  uint32_t acquired_token;
  uint32_t magic;
  int32_t abort_reason;
  wl_bulk_sender_state_t state;
  wl_bulk_phase_t phase;
  wl_bulk_status_code_t last_status;
  uint32_t negotiated_chunk_size;
  uint16_t retry_count;
  uint16_t max_retries;
  uint8_t action_acquired;
  uint8_t chunk_acknowledged;
  uint8_t retry_wait;
} wl_bulk_sender_impl_t;

_Static_assert(sizeof(wl_bulk_sender_impl_t) <= WL_BULK_SENDER_STORAGE_SIZE,
               "WL_BULK_SENDER_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_bulk_sender_impl_t) <= _Alignof(wl_bulk_sender_t),
               "wl_bulk_sender_t alignment is too small");
_Static_assert(sizeof(wl_bulk_sender_action_t) <= 256U,
               "wl_bulk_sender_action_t exceeds the stack-frame budget");

static wl_bulk_sender_impl_t *sender_impl(wl_bulk_sender_t *sender) {
  return (wl_bulk_sender_impl_t *)(void *)sender;
}

static const wl_bulk_sender_impl_t *
sender_impl_const(const wl_bulk_sender_t *sender) {
  return (const wl_bulk_sender_impl_t *)(const void *)sender;
}

static bool sender_initialized(const wl_bulk_sender_t *sender) {
  return sender != NULL &&
         sender_impl_const(sender)->magic == WL_BULK_SENDER_MAGIC;
}

static bool interval_valid(uint32_t interval_ms) {
  return interval_ms != 0U && interval_ms < WL_BULK_MAX_INTERVAL;
}

static bool deadline_elapsed(wl_time_ms_t now_ms, wl_time_ms_t started_at,
                             uint32_t interval_ms) {
  return (uint32_t)(now_ms - started_at) >= interval_ms;
}

static uint32_t deadline_remaining(wl_time_ms_t now_ms, wl_time_ms_t started_at,
                                   uint32_t interval_ms) {
  uint32_t age = (uint32_t)(now_ms - started_at);

  return age >= interval_ms ? 0U : interval_ms - age;
}

static void increment(uint32_t *counter) {
  if (*counter != UINT32_MAX) {
    ++*counter;
  }
}

static bool sender_active(const wl_bulk_sender_impl_t *impl) {
  return impl->state == WL_BULK_SENDER_READY ||
         impl->state == WL_BULK_SENDER_WAIT_STATUS;
}

static bool phase_valid(wl_bulk_phase_t phase) {
  return phase >= WL_BULK_PHASE_BEGIN && phase <= WL_BULK_PHASE_ABORT;
}

static bool status_code_valid(wl_bulk_status_code_t code) {
  return code >= WL_BULK_STATUS_OK && code <= WL_BULK_STATUS_TIMED_OUT;
}

static uint32_t next_token(wl_bulk_sender_impl_t *impl) {
  ++impl->next_token;
  if (impl->next_token == 0U) {
    ++impl->next_token;
  }
  return impl->next_token;
}

static void set_action_length(wl_bulk_sender_impl_t *impl) {
  uint64_t remaining;

  if (impl->phase != WL_BULK_PHASE_CHUNK ||
      impl->next_offset >= impl->descriptor.total_length) {
    impl->action_length = 0U;
    return;
  }
  remaining = impl->descriptor.total_length - impl->next_offset;
  impl->action_length = remaining < impl->negotiated_chunk_size
                            ? (size_t)remaining
                            : (size_t)impl->negotiated_chunk_size;
}

static void prepare_phase(wl_bulk_sender_impl_t *impl, wl_bulk_phase_t phase) {
  impl->phase = phase;
  impl->state = WL_BULK_SENDER_READY;
  impl->action_acquired = 0U;
  impl->acquired_token = 0U;
  impl->retry_count = 0U;
  impl->retry_wait = 0U;
  impl->wait_interval_ms = 0U;
  set_action_length(impl);
}

static void fail_sender(wl_bulk_sender_impl_t *impl,
                        wl_bulk_status_code_t status) {
  impl->state = WL_BULK_SENDER_FAILED;
  impl->last_status = status;
  impl->action_acquired = 0U;
  impl->acquired_token = 0U;
  impl->retry_wait = 0U;
  impl->wait_interval_ms = 0U;
  increment(&impl->stats.failed);
}

static void protocol_failure(wl_bulk_sender_impl_t *impl) {
  increment(&impl->stats.protocol_errors);
  fail_sender(impl, WL_BULK_STATUS_INVALID);
}

static bool descriptor_matches(const wl_bulk_descriptor_t *left,
                               const wl_bulk_descriptor_t *right) {
  return left->transfer_id == right->transfer_id &&
         left->total_length == right->total_length &&
         left->requested_chunk_size == right->requested_chunk_size &&
         left->object_crc32c == right->object_crc32c;
}

static bool action_matches(const wl_bulk_sender_impl_t *impl,
                           const wl_bulk_sender_action_t *action) {
  return action->phase == impl->phase &&
         descriptor_matches(&action->descriptor, &impl->descriptor) &&
         action->offset == impl->next_offset &&
         action->length == impl->action_length &&
         action->abort_reason == impl->abort_reason;
}

static bool status_chunk_size_matches(const wl_bulk_sender_impl_t *impl,
                                      const wl_bulk_status_t *status) {
  if (impl->phase == WL_BULK_PHASE_BEGIN) {
    return true;
  }
  return status->accepted_chunk_size == impl->negotiated_chunk_size;
}

static bool proposed_chunk_size_valid(const wl_bulk_sender_impl_t *impl,
                                      uint32_t chunk_size) {
  if (chunk_size == 0U || chunk_size > impl->descriptor.requested_chunk_size) {
    return false;
  }
#if SIZE_MAX < UINT32_MAX
  if (chunk_size > SIZE_MAX) {
    return false;
  }
#endif
  return true;
}

static bool terminal_status_valid(const wl_bulk_sender_impl_t *impl,
                                  const wl_bulk_status_t *status) {
  if (status->next_offset > impl->descriptor.total_length) {
    return false;
  }
  if (impl->negotiated_chunk_size != 0U) {
    return status->accepted_chunk_size == impl->negotiated_chunk_size;
  }
  if (status->accepted_chunk_size == 0U) {
    return impl->phase == WL_BULK_PHASE_ABORT;
  }
  return proposed_chunk_size_valid(impl, status->accepted_chunk_size);
}

static bool status_is_stale(const wl_bulk_sender_impl_t *impl,
                            const wl_bulk_status_t *status) {
  if (status->phase < impl->phase) {
    return true;
  }
  return impl->phase == WL_BULK_PHASE_CHUNK &&
         status->phase == WL_BULK_PHASE_CHUNK &&
         status->code == WL_BULK_STATUS_OK && impl->chunk_acknowledged != 0U &&
         status->next_offset <= impl->next_offset;
}

static void wait_for_retry(wl_bulk_sender_impl_t *impl, wl_time_ms_t now_ms,
                           uint32_t delay_ms,
                           wl_bulk_status_code_t exhausted_status) {
  if (impl->retry_count >= impl->max_retries) {
    fail_sender(impl, exhausted_status);
    return;
  }
  impl->wait_started_at = now_ms;
  impl->wait_interval_ms = delay_ms;
  impl->retry_wait = 1U;
}

wl_bulk_err_t wl_bulk_sender_init(wl_bulk_sender_t *sender,
                                  const wl_bulk_sender_config_t *config) {
  wl_bulk_sender_impl_t *impl;

  if (sender == NULL || config == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  if (!interval_valid(config->status_timeout_ms) ||
      !interval_valid(config->busy_retry_ms)) {
    return WL_BULK_ERR_INVALID_ARG;
  }

  memset(sender, 0, sizeof(*sender));
  impl = sender_impl(sender);
  impl->status_timeout_ms = config->status_timeout_ms;
  impl->busy_retry_ms = config->busy_retry_ms;
  impl->max_retries = config->max_retries;
  impl->state = WL_BULK_SENDER_IDLE;
  impl->last_status = WL_BULK_STATUS_OK;
  impl->magic = WL_BULK_SENDER_MAGIC;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_sender_reset(wl_bulk_sender_t *sender) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = sender_impl(sender);
  if (impl->action_acquired != 0U || sender_active(impl) ||
      (impl->state == WL_BULK_SENDER_FAILED &&
       impl->phase != WL_BULK_PHASE_ABORT)) {
    return WL_BULK_ERR_BUSY;
  }
  memset(&impl->descriptor, 0, sizeof(impl->descriptor));
  impl->next_offset = 0U;
  impl->action_length = 0U;
  impl->wait_started_at = 0U;
  impl->wait_interval_ms = 0U;
  impl->acquired_token = 0U;
  impl->abort_reason = 0;
  impl->state = WL_BULK_SENDER_IDLE;
  impl->phase = WL_BULK_PHASE_NONE;
  impl->last_status = WL_BULK_STATUS_OK;
  impl->negotiated_chunk_size = 0U;
  impl->retry_count = 0U;
  impl->action_acquired = 0U;
  impl->chunk_acknowledged = 0U;
  impl->retry_wait = 0U;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_sender_start(wl_bulk_sender_t *sender,
                                   const wl_bulk_descriptor_t *descriptor) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (descriptor == NULL || descriptor->transfer_id == 0U ||
      descriptor->requested_chunk_size == 0U) {
    return WL_BULK_ERR_INVALID_ARG;
  }
#if SIZE_MAX < UINT32_MAX
  if (descriptor->requested_chunk_size > SIZE_MAX) {
    return WL_BULK_ERR_INVALID_ARG;
  }
#endif
  impl = sender_impl(sender);
  if (impl->state != WL_BULK_SENDER_IDLE) {
    return WL_BULK_ERR_BUSY;
  }

  impl->descriptor = *descriptor;
  impl->next_offset = 0U;
  impl->negotiated_chunk_size = 0U;
  impl->chunk_acknowledged = 0U;
  impl->abort_reason = 0;
  impl->last_status = WL_BULK_STATUS_OK;
  prepare_phase(impl, WL_BULK_PHASE_BEGIN);
  increment(&impl->stats.starts);
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_sender_request_abort(wl_bulk_sender_t *sender,
                                           int32_t reason) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = sender_impl(sender);
  if ((!sender_active(impl) && impl->state != WL_BULK_SENDER_FAILED) ||
      impl->phase == WL_BULK_PHASE_ABORT) {
    return WL_BULK_ERR_INVALID_STATE;
  }
  if (impl->action_acquired != 0U) {
    return WL_BULK_ERR_BUSY;
  }
  impl->abort_reason = reason;
  prepare_phase(impl, WL_BULK_PHASE_ABORT);
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_sender_action_acquire(wl_bulk_sender_t *sender,
                              wl_bulk_sender_action_t *out_action) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (out_action == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl = sender_impl(sender);
  if (impl->state != WL_BULK_SENDER_READY) {
    return WL_BULK_ERR_NOT_FOUND;
  }
  if (impl->action_acquired != 0U) {
    return WL_BULK_ERR_BUSY;
  }

  memset(out_action, 0, sizeof(*out_action));
  out_action->phase = impl->phase;
  out_action->descriptor = impl->descriptor;
  out_action->offset = impl->next_offset;
  out_action->length = impl->action_length;
  out_action->abort_reason = impl->abort_reason;
  out_action->token = next_token(impl);
  impl->acquired_token = out_action->token;
  impl->action_acquired = 1U;
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_sender_action_defer(wl_bulk_sender_t *sender,
                            const wl_bulk_sender_action_t *action) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (action == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl = sender_impl(sender);
  if (impl->state != WL_BULK_SENDER_READY || impl->action_acquired == 0U ||
      action->token != impl->acquired_token) {
    return WL_BULK_ERR_INVALID_STATE;
  }
  if (!action_matches(impl, action)) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl->action_acquired = 0U;
  impl->acquired_token = 0U;
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_sender_action_submitted(wl_bulk_sender_t *sender,
                                const wl_bulk_sender_action_t *action,
                                wl_time_ms_t now_ms) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (action == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl = sender_impl(sender);
  if (impl->state != WL_BULK_SENDER_READY || impl->action_acquired == 0U ||
      action->token != impl->acquired_token) {
    return WL_BULK_ERR_INVALID_STATE;
  }
  if (!action_matches(impl, action)) {
    return WL_BULK_ERR_INVALID_ARG;
  }

  impl->action_acquired = 0U;
  impl->acquired_token = 0U;
  impl->state = WL_BULK_SENDER_WAIT_STATUS;
  impl->wait_started_at = now_ms;
  impl->wait_interval_ms = impl->status_timeout_ms;
  impl->retry_wait = 0U;
  increment(&impl->stats.actions_submitted);
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_sender_on_status(wl_bulk_sender_t *sender,
                                       const wl_bulk_status_t *status,
                                       wl_time_ms_t now_ms) {
  wl_bulk_sender_impl_t *impl;
  uint64_t expected_offset;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (status == NULL || !phase_valid(status->phase) ||
      !status_code_valid(status->code)) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl = sender_impl(sender);
  if (status->transfer_id != impl->descriptor.transfer_id) {
    return WL_BULK_ERR_NOT_FOUND;
  }
  if (impl->state != WL_BULK_SENDER_WAIT_STATUS || impl->retry_wait != 0U) {
    return WL_BULK_ERR_INVALID_STATE;
  }
  if (status->phase == WL_BULK_PHASE_ABORT &&
      (status->code == WL_BULK_STATUS_ABORTED ||
       status->code == WL_BULK_STATUS_TIMED_OUT)) {
    if (!terminal_status_valid(impl, status)) {
      protocol_failure(impl);
      return WL_BULK_ERR_PROTOCOL;
    }
    increment(&impl->stats.statuses_received);
    impl->last_status = status->code;
    impl->next_offset = status->next_offset;
    impl->wait_interval_ms = 0U;
    if (status->code == WL_BULK_STATUS_ABORTED) {
      impl->state = WL_BULK_SENDER_ABORTED;
      increment(&impl->stats.aborted);
    } else {
      impl->phase = WL_BULK_PHASE_ABORT;
      fail_sender(impl, status->code);
    }
    return WL_BULK_OK;
  }
  if (status_is_stale(impl, status)) {
    return WL_BULK_ERR_NOT_FOUND;
  }
  if (status->phase != impl->phase ||
      !status_chunk_size_matches(impl, status)) {
    protocol_failure(impl);
    return WL_BULK_ERR_PROTOCOL;
  }

  increment(&impl->stats.statuses_received);
  impl->last_status = status->code;
  if (status->code == WL_BULK_STATUS_BUSY) {
    if (status->next_offset != impl->next_offset) {
      protocol_failure(impl);
      return WL_BULK_ERR_PROTOCOL;
    }
    increment(&impl->stats.busy_responses);
    wait_for_retry(impl, now_ms, impl->busy_retry_ms, WL_BULK_STATUS_BUSY);
    return WL_BULK_OK;
  }

  if (status->code == WL_BULK_STATUS_OUT_OF_ORDER) {
    if (status->next_offset > impl->descriptor.total_length ||
        (impl->phase == WL_BULK_PHASE_BEGIN &&
         !proposed_chunk_size_valid(impl, status->accepted_chunk_size)) ||
        (impl->phase == WL_BULK_PHASE_CHUNK &&
         status->accepted_chunk_size != impl->negotiated_chunk_size) ||
        (impl->phase != WL_BULK_PHASE_BEGIN &&
         impl->phase != WL_BULK_PHASE_CHUNK)) {
      protocol_failure(impl);
      return WL_BULK_ERR_PROTOCOL;
    }
    if (impl->phase == WL_BULK_PHASE_BEGIN) {
      impl->negotiated_chunk_size = status->accepted_chunk_size;
    }
    impl->next_offset = status->next_offset;
    impl->chunk_acknowledged = 0U;
    prepare_phase(impl, impl->next_offset == impl->descriptor.total_length
                            ? WL_BULK_PHASE_END
                            : WL_BULK_PHASE_CHUNK);
    return WL_BULK_OK;
  }

  if (status->code != WL_BULK_STATUS_OK) {
    if (status->code == WL_BULK_STATUS_ABORTED) {
      impl->state = WL_BULK_SENDER_ABORTED;
      increment(&impl->stats.aborted);
    } else {
      fail_sender(impl, status->code);
    }
    return WL_BULK_OK;
  }

  switch (impl->phase) {
  case WL_BULK_PHASE_BEGIN:
    if (!proposed_chunk_size_valid(impl, status->accepted_chunk_size) ||
        status->next_offset > impl->descriptor.total_length) {
      protocol_failure(impl);
      return WL_BULK_ERR_PROTOCOL;
    }
    impl->negotiated_chunk_size = status->accepted_chunk_size;
    impl->next_offset = status->next_offset;
    prepare_phase(impl, impl->next_offset == impl->descriptor.total_length
                            ? WL_BULK_PHASE_END
                            : WL_BULK_PHASE_CHUNK);
    return WL_BULK_OK;

  case WL_BULK_PHASE_CHUNK:
    expected_offset = impl->next_offset + impl->action_length;
    if (status->next_offset != expected_offset ||
        status->next_offset > impl->descriptor.total_length) {
      protocol_failure(impl);
      return WL_BULK_ERR_PROTOCOL;
    }
    impl->next_offset = status->next_offset;
    impl->chunk_acknowledged = 1U;
    prepare_phase(impl, impl->next_offset == impl->descriptor.total_length
                            ? WL_BULK_PHASE_END
                            : WL_BULK_PHASE_CHUNK);
    return WL_BULK_OK;

  case WL_BULK_PHASE_END:
    if (status->next_offset != impl->descriptor.total_length) {
      protocol_failure(impl);
      return WL_BULK_ERR_PROTOCOL;
    }
    impl->next_offset = status->next_offset;
    impl->state = WL_BULK_SENDER_COMPLETED;
    impl->wait_interval_ms = 0U;
    increment(&impl->stats.completed);
    return WL_BULK_OK;

  case WL_BULK_PHASE_ABORT:
    impl->next_offset = status->next_offset;
    impl->state = WL_BULK_SENDER_ABORTED;
    impl->wait_interval_ms = 0U;
    increment(&impl->stats.aborted);
    return WL_BULK_OK;

  default:
    protocol_failure(impl);
    return WL_BULK_ERR_PROTOCOL;
  }
}

wl_bulk_err_t wl_bulk_sender_poll(wl_bulk_sender_t *sender,
                                  wl_time_ms_t now_ms) {
  wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  impl = sender_impl(sender);
  if (impl->state != WL_BULK_SENDER_WAIT_STATUS ||
      !deadline_elapsed(now_ms, impl->wait_started_at,
                        impl->wait_interval_ms)) {
    return WL_BULK_OK;
  }
  if (impl->retry_wait != 0U) {
    ++impl->retry_count;
    increment(&impl->stats.retries);
    impl->state = WL_BULK_SENDER_READY;
    impl->retry_wait = 0U;
    impl->wait_interval_ms = 0U;
    return WL_BULK_OK;
  }

  wait_for_retry(impl, now_ms, 0U, WL_BULK_STATUS_TIMED_OUT);
  if (impl->state == WL_BULK_SENDER_FAILED) {
    return WL_BULK_OK;
  }
  ++impl->retry_count;
  increment(&impl->stats.retries);
  impl->state = WL_BULK_SENDER_READY;
  impl->retry_wait = 0U;
  impl->wait_interval_ms = 0U;
  return WL_BULK_OK;
}

wl_bulk_err_t
wl_bulk_sender_get_deadline_hint(const wl_bulk_sender_t *sender,
                                 wl_time_ms_t now_ms,
                                 wl_bulk_deadline_hint_t *out_hint) {
  const wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (out_hint == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl = sender_impl_const(sender);
  out_hint->next_deadline_ms =
      impl->state == WL_BULK_SENDER_WAIT_STATUS
          ? deadline_remaining(now_ms, impl->wait_started_at,
                               impl->wait_interval_ms)
          : WL_BULK_NO_DEADLINE_MS;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_sender_get_result(const wl_bulk_sender_t *sender,
                                        wl_bulk_sender_result_t *out_result) {
  const wl_bulk_sender_impl_t *impl;

  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (out_result == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  impl = sender_impl_const(sender);
  out_result->state = impl->state;
  out_result->status = impl->last_status;
  out_result->next_offset = impl->next_offset;
  out_result->retry_count = impl->retry_count;
  return WL_BULK_OK;
}

wl_bulk_err_t wl_bulk_sender_get_stats(const wl_bulk_sender_t *sender,
                                       wl_bulk_sender_stats_t *out_stats) {
  if (!sender_initialized(sender)) {
    return sender == NULL ? WL_BULK_ERR_INVALID_ARG
                          : WL_BULK_ERR_NOT_INITIALIZED;
  }
  if (out_stats == NULL) {
    return WL_BULK_ERR_INVALID_ARG;
  }
  *out_stats = sender_impl_const(sender)->stats;
  return WL_BULK_OK;
}
