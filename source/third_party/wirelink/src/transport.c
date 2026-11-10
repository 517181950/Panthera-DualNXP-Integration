/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "wirelink/wirelink.h"

#include "context.h"
#include "rx_ring.h"
#include "unit_rx.h"

wl_err_t wl_set_sink(wl_ctx_t *ctx, wl_sink_fn sink, void *user_data) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->in_callback) {
    return WL_ERR_REENTRANT;
  }
  wl_ctx_impl(ctx)->sink = sink;
  wl_ctx_impl(ctx)->sink_user_data = user_data;
  return WL_OK;
}

wl_err_t wl_get_config(const wl_ctx_t *ctx, wl_config_t *out_config) {
  if (ctx == NULL || out_config == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl_const(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  *out_config = wl_ctx_impl_const(ctx)->config;
  return WL_OK;
}

wl_err_t wl_tx_cancel(wl_ctx_t *ctx, wl_tx_handle_t handle) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (handle == 0U || handle != wl_ctx_impl_const(ctx)->tx_handle) {
    return WL_ERR_NOT_FOUND;
  }
  if (wl_ctx_impl(ctx)->tx_state == WL_TX_STATE_IDLE || wl_ctx_impl(ctx)->tx_state == WL_TX_STATE_SUCCESS ||
      wl_ctx_impl(ctx)->tx_state == WL_TX_STATE_FAILED || wl_ctx_impl(ctx)->tx_state == WL_TX_STATE_CANCELLED) {
    return WL_ERR_INVALID_STATE;
  }
  if (wl_ctx_impl(ctx)->tx_inflight != 0U && wl_ctx_impl(ctx)->in_flight_reliable != 0U) {
    wl_ctx_impl(ctx)->tx_cancel_requested = 1U;
  } else if (wl_ctx_impl(ctx)->tx_queued != 0U && wl_ctx_impl(ctx)->in_flight_reliable != 0U) {
    wl_ctx_impl(ctx)->tx_queued = 0U;
    wl_ctx_impl(ctx)->in_flight_reliable = 0U;
  }
  wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_CANCELLED;
  wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
  wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
  wl_ctx_impl(ctx)->tx_retries_left = 0U;
  wl_ctx_impl(ctx)->tx_result_code = WL_ERR_CANCELLED;
  return WL_OK;
}

wl_err_t wl_tx_status(const wl_ctx_t *ctx, wl_tx_handle_t handle,
                 wl_tx_state_t *out_state) {
  if (ctx == NULL || out_state == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl_const(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (handle == 0U || handle != wl_ctx_impl_const(ctx)->tx_handle) {
    return WL_ERR_NOT_FOUND;
  }
  *out_state = wl_ctx_impl_const(ctx)->tx_state;
  return WL_OK;
}

wl_err_t wl_tx_take(wl_ctx_t *ctx, wl_tx_handle_t handle,
               wl_tx_result_t *out_result) {
  if (ctx == NULL || out_result == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (handle == 0U || handle != wl_ctx_impl(ctx)->tx_handle) {
    return WL_ERR_NOT_FOUND;
  }
  if ((wl_ctx_impl(ctx)->tx_inflight != 0U && wl_ctx_impl(ctx)->in_flight_reliable != 0U) ||
      (wl_ctx_impl(ctx)->tx_state != WL_TX_STATE_SUCCESS &&
       wl_ctx_impl(ctx)->tx_state != WL_TX_STATE_FAILED &&
       wl_ctx_impl(ctx)->tx_state != WL_TX_STATE_CANCELLED)) {
    return WL_ERR_INVALID_STATE;
  }
  out_result->state = wl_ctx_impl(ctx)->tx_state;
  out_result->result = wl_ctx_impl(ctx)->tx_result_code;
  out_result->retries_used = wl_ctx_impl(ctx)->tx_retries_used;
  wl_ctx_impl(ctx)->tx_handle = 0U;
  wl_ctx_impl(ctx)->tx_pending_event = WL_EVT_NONE;
  wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_IDLE;
  wl_ctx_impl(ctx)->tx_cancel_requested = 0U;
  wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
  wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
  wl_ctx_impl(ctx)->tx_payload.length = 0U;
  wl_ctx_impl(ctx)->tx_payload.data = wl_ctx_impl(ctx)->storage.tx_payload;
  wl_ctx_impl(ctx)->tx_encoded_len = 0U;
  return WL_OK;
}

wl_err_t wl_feed_recover_reset(wl_ctx_t *ctx) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  if (wl_ctx_impl(ctx)->rx_event_leased != 0U) {
    return WL_ERR_WOULD_BLOCK;
  }
  (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring,
                                    wl_rx_ring_readable(&wl_ctx_impl(ctx)->rx_ring));
  return WL_OK;
}

wl_err_t wl_rx_get_counters(const wl_ctx_t *ctx,
                            wl_rx_counters_t *out_counters) {
  if (ctx == NULL || out_counters == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl_const(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  *out_counters = wl_ctx_impl_const(ctx)->rx_counters;
  return WL_OK;
}

void wl_event_release(wl_ctx_t *ctx, const wl_event_t *event) {
  if (ctx == NULL || event == NULL || wl_ctx_impl(ctx)->initialized == 0U) {
    return;
  }
  if ((event->type == WL_EVT_UNRELIABLE_RX ||
       event->type == WL_EVT_RELIABLE_RX) &&
      wl_ctx_impl(ctx)->rx_event_leased != 0U &&
      event->lease == wl_ctx_impl(ctx)->rx_event_generation) {
    wl_ctx_impl(ctx)->rx_event_leased = 0U;
    if (wl_ctx_impl(ctx)->rx_pending_consume != 0U) {
      if (wl_ctx_impl(ctx)->rx_candidate_source == WL_RX_SOURCE_UNIT) {
        (void)wl_rx_unit_consumer_consume(ctx);
      } else {
        (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring,
                                          wl_ctx_impl(ctx)->rx_pending_consume);
      }
      wl_ctx_impl(ctx)->rx_pending_consume = 0U;
    }
    wl_ctx_impl(ctx)->rx_candidate_source = 0U;
    wl_ctx_impl(ctx)->rx_payload.length = 0U;
  }
}
