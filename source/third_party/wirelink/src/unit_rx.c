/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "wirelink/frame.h"
#include "wirelink/wirelink.h"

#include "context.h"
#include "unit_rx.h"

wl_err_t wl_rx_unit_queue_init(wl_ctx_t *ctx,
                          const wl_rx_unit_queue_config_t *config) {
  wl_rx_unit_queue_state_t *queue;

  if (ctx == NULL || config == NULL || config->storage == NULL ||
      config->unit_size == 0U || config->slot_count < 2U ||
      config->slot_count > WL_RX_UNIT_QUEUE_MAX_SLOTS ||
      config->unit_size > SIZE_MAX / config->slot_count ||
      config->storage_size < config->unit_size * config->slot_count) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope == WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  if (config->unit_size < wl_frame_raw_size(
                              wl_ctx_impl(ctx)->config.max_payload_len,
                              wl_ctx_impl(ctx)->config.integrity)) {
    return WL_ERR_BUF_TOO_SMALL;
  }

  queue = &wl_ctx_impl(ctx)->rx_units;
  memset(queue, 0, sizeof(*queue));
  queue->storage = config->storage;
  queue->unit_size = config->unit_size;
  queue->slot_count = config->slot_count;
  atomic_init(&queue->write_cursor, 0U);
  atomic_init(&queue->read_cursor, 0U);
  if (!atomic_is_lock_free(&queue->write_cursor) ||
      !atomic_is_lock_free(&queue->read_cursor)) {
    return WL_ERR_NOT_SUPPORTED;
  }
  queue->initialized = 1U;
  return WL_OK;
}

wl_err_t wl_rx_unit_claim(wl_ctx_t *ctx, size_t maximum_length,
                     wl_rx_unit_claim_t *out_claim) {
  wl_rx_unit_queue_state_t *queue;
  uint32_t write;
  uint32_t read;

  if (ctx == NULL || out_claim == NULL || maximum_length == 0U) {
    return WL_ERR_INVALID_ARG;
  }
  memset(out_claim, 0, sizeof(*out_claim));
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  queue = &wl_ctx_impl(ctx)->rx_units;
  if (queue->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (queue->claim_active != 0U) {
    return WL_ERR_BUSY;
  }
  if (maximum_length > queue->unit_size) {
    return WL_ERR_BUF_TOO_SMALL;
  }

  write = atomic_load_explicit(&queue->write_cursor, memory_order_relaxed);
  read = atomic_load_explicit(&queue->read_cursor, memory_order_acquire);
  if ((uint32_t)(write - read) >= queue->slot_count) {
    return WL_ERR_WOULD_BLOCK;
  }

  ++queue->claim_token;
  if (queue->claim_token == 0U) {
    ++queue->claim_token;
  }
  queue->claim_cursor = write;
  queue->claim_active = 1U;
  out_claim->span.data =
      queue->storage + ((size_t)(write % queue->slot_count) * queue->unit_size);
  out_claim->span.length = maximum_length;
  out_claim->token = queue->claim_token;
  return WL_OK;
}

static int claim_matches(const wl_rx_unit_queue_state_t *queue,
                         const wl_rx_unit_claim_t *claim) {
  const uint8_t *expected =
      queue->storage +
      ((size_t)(queue->claim_cursor % queue->slot_count) * queue->unit_size);
  return queue->claim_active != 0U && claim != NULL &&
         claim->token == queue->claim_token && claim->span.data == expected;
}

wl_err_t wl_rx_unit_commit(wl_ctx_t *ctx, const wl_rx_unit_claim_t *claim,
                      size_t length) {
  wl_rx_unit_queue_state_t *queue;
  uint32_t write;

  if (ctx == NULL || claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  queue = &wl_ctx_impl(ctx)->rx_units;
  if (!claim_matches(queue, claim)) {
    return WL_ERR_NOT_FOUND;
  }
  if (length == 0U || length > claim->span.length ||
      length > queue->unit_size) {
    return WL_ERR_INVALID_ARG;
  }

  write = queue->claim_cursor;
  queue->lengths[write % queue->slot_count] = length;
  queue->claim_active = 0U;
  atomic_store_explicit(&queue->write_cursor, write + 1U,
                        memory_order_release);
  return WL_OK;
}

wl_err_t wl_rx_unit_abort(wl_ctx_t *ctx,
                          const wl_rx_unit_claim_t *claim) {
  wl_rx_unit_queue_state_t *queue;

  if (ctx == NULL || claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  queue = &wl_ctx_impl(ctx)->rx_units;
  if (!claim_matches(queue, claim)) {
    return WL_ERR_NOT_FOUND;
  }
  queue->claim_active = 0U;
  return WL_OK;
}

wl_span_t wl_rx_unit_consumer_peek(wl_ctx_t *ctx) {
  wl_rx_unit_queue_state_t *queue = &wl_ctx_impl(ctx)->rx_units;
  uint32_t read;
  uint32_t write;
  wl_span_t result = {0};

  if (queue->initialized == 0U) {
    return result;
  }
  read = atomic_load_explicit(&queue->read_cursor, memory_order_relaxed);
  write = atomic_load_explicit(&queue->write_cursor, memory_order_acquire);
  if (read == write) {
    return result;
  }
  result.data =
      queue->storage + ((size_t)(read % queue->slot_count) * queue->unit_size);
  result.length = queue->lengths[read % queue->slot_count];
  return result;
}

int wl_rx_unit_consumer_has_data(const wl_ctx_t *ctx) {
  const wl_rx_unit_queue_state_t *queue;
  uint32_t read;
  uint32_t write;

  if (ctx == NULL) {
    return 0;
  }
  queue = &wl_ctx_impl_const(ctx)->rx_units;
  if (queue->initialized == 0U) {
    return 0;
  }
  read = atomic_load_explicit(&queue->read_cursor, memory_order_relaxed);
  write = atomic_load_explicit(&queue->write_cursor, memory_order_acquire);
  return read != write;
}

int wl_rx_unit_consumer_consume(wl_ctx_t *ctx) {
  wl_rx_unit_queue_state_t *queue = &wl_ctx_impl(ctx)->rx_units;
  uint32_t read;
  uint32_t write;

  if (queue->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  read = atomic_load_explicit(&queue->read_cursor, memory_order_relaxed);
  write = atomic_load_explicit(&queue->write_cursor, memory_order_acquire);
  if (read == write) {
    return WL_ERR_NO_DATA;
  }
  queue->lengths[read % queue->slot_count] = 0U;
  atomic_store_explicit(&queue->read_cursor, read + 1U, memory_order_release);
  return WL_OK;
}
