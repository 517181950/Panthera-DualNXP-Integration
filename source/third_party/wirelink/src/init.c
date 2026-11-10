/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "wirelink/cobs.h"
#include "wirelink/frame.h"
#include "wirelink/wirelink.h"

#include "context.h"
#include "rx_ring.h"

uint64_t wl_link_session_id(const wl_ctx_t *ctx) {
  return ctx != NULL ? wl_ctx_impl_const(ctx)->session_id : 0U;
}

static int storage_overlaps(const uint8_t *a, size_t a_size,
                             const uint8_t *b, size_t b_size) {
  const uintptr_t first = (uintptr_t)a;
  const uintptr_t second = (uintptr_t)b;
  return first <= second ? second - first < a_size : first - second < b_size;
}

static size_t wl_max_unit_size(const wl_config_t *config) {
  const size_t raw = wl_frame_raw_size(config->max_payload_len, config->integrity);
  if (raw == 0U) {
    return 0U;
  }
  switch (config->envelope) {
  case WL_ENVELOPE_COBS_STREAM:
    return wl_cobs_encoded_max_size(raw) + 1U;
  case WL_ENVELOPE_BUS_LENGTH16:
    return raw + 2U;
  case WL_ENVELOPE_NATIVE_PACKET:
  default:
    return raw;
  }
}

wl_err_t wl_config_requirements(const wl_config_t *config,
                           wl_storage_requirements_t *out_requirements) {
  size_t unit;
  size_t control;
  if (config == NULL || out_requirements == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (config->max_payload_len == 0U ||
      config->max_payload_len > WL_FRAME_MAX_PAYLOAD) {
    return WL_ERR_INVALID_ARG;
  }
  if (config->envelope < WL_ENVELOPE_COBS_STREAM ||
      config->envelope > WL_ENVELOPE_BUS_LENGTH16) {
    return WL_ERR_INVALID_ARG;
  }
  if (config->integrity < WL_INTEGRITY_NONE ||
      config->integrity > WL_INTEGRITY_CRC32) {
    return WL_ERR_INVALID_ARG;
  }
  if (config->session_id == 0ULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (config->ack_timeout_ms >= UINT32_C(0x80000000)) {
    return WL_ERR_INVALID_ARG;
  }
  unit = wl_max_unit_size(config);
  control = wl_frame_encode_overhead(config->envelope, config->integrity);
  if (unit == 0U || control == 0U ||
      (config->max_transmission_unit != 0U &&
       config->max_transmission_unit < unit)) {
    return WL_ERR_INVALID_ARG;
  }
  out_requirements->tx_payload_size = config->max_payload_len;
  out_requirements->tx_unit_size = unit;
  out_requirements->control_unit_size = control;
  out_requirements->rx_fifo_size =
      (config->envelope == WL_ENVELOPE_COBS_STREAM)
          ? wl_rx_ring_storage_size(unit)
          : 0U;
  out_requirements->rx_fallback_size =
      (config->envelope == WL_ENVELOPE_COBS_STREAM) ? unit - 1U : unit;
  return WL_OK;
}

wl_err_t wl_init(wl_ctx_t *ctx, const wl_config_t *config,
            const wl_storage_t *storage) {
  wl_storage_requirements_t requirements;
  if (ctx == NULL || storage == NULL ||
      wl_config_requirements(config, &requirements) != WL_OK) {
    return WL_ERR_INVALID_ARG;
  }
  if (storage->tx_payload == NULL || storage->tx_unit == NULL || storage->control_unit == NULL ||
      storage->rx_fallback == NULL ||
      storage->tx_payload_size < requirements.tx_payload_size ||
      storage->tx_unit_size < requirements.tx_unit_size ||
      storage->control_unit_size < requirements.control_unit_size ||
      storage->rx_fallback_size < requirements.rx_fallback_size ||
      (requirements.rx_fifo_size != 0U &&
       (storage->rx_fifo == NULL ||
        storage->rx_fifo_size < requirements.rx_fifo_size))) {
    return WL_ERR_BUF_TOO_SMALL;
  }
  /* Retained reliable payload, reusable DATA image and control image have
   * distinct lifetimes. Validate once, never on the per-byte hot path. */
  if (storage_overlaps(storage->tx_payload, storage->tx_payload_size,
                       storage->tx_unit, storage->tx_unit_size) ||
      storage_overlaps(storage->tx_payload, storage->tx_payload_size,
                       storage->control_unit, storage->control_unit_size) ||
      storage_overlaps(storage->tx_unit, storage->tx_unit_size,
                       storage->control_unit, storage->control_unit_size)) {
    return WL_ERR_INVALID_ARG;
  }

  memset(ctx, 0, sizeof(*ctx));
  wl_ctx_impl(ctx)->config = *config;
  wl_ctx_impl(ctx)->storage = *storage;
  wl_ctx_impl(ctx)->tx_retries_max =
      (config->max_retries != 0U) ? config->max_retries : 0U;
  wl_ctx_impl(ctx)->tx_retries_left = wl_ctx_impl(ctx)->tx_retries_max;
  wl_ctx_impl(ctx)->session_id = config->session_id;
  wl_ctx_impl(ctx)->tx_handle = 0U;
  wl_ctx_impl(ctx)->tx_token = 1U;
  wl_ctx_impl(ctx)->tx_unreliable_completions = 0U;
  wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_IDLE;
  wl_ctx_impl(ctx)->tx_last_message_id = 0U;
  wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
  wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
  wl_ctx_impl(ctx)->tx_payload = (wl_span_t){storage->tx_payload, 0U};
  wl_ctx_impl(ctx)->rx_payload = (wl_span_t){storage->rx_fallback, 0U};

  if (requirements.rx_fifo_size != 0U &&
      wl_rx_ring_init(&wl_ctx_impl(ctx)->rx_ring, storage->rx_fifo,
                      storage->rx_fifo_size) != WL_OK) {
    return WL_ERR_INVALID_ARG;
  }

  wl_ctx_impl(ctx)->initialized = 1U;
  return WL_OK;
}
