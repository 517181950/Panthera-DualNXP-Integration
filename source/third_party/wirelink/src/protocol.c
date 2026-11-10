/* SPDX-License-Identifier: Apache-2.0 */

#include <limits.h>
#include <string.h>

#include "wirelink/cobs.h"
#include "wirelink/frame.h"
#include "wirelink/wirelink.h"

#include "context.h"
#include "rx_ring.h"
#include "unit_rx.h"

static int wl_feed_unit_raw(wl_ctx_t *ctx, const uint8_t *unit, size_t len);
static int wl_send_ack(wl_ctx_t *ctx, const wl_frame_view_t *view);
static int wl_handle_ack(wl_ctx_t *ctx, const wl_frame_view_t *view);
static int wl_send_tx_payload(wl_ctx_t *ctx);
static int wl_submit_data(wl_ctx_t *ctx, uint8_t reliable);
static int wl_complete_unreliable_tx(wl_ctx_t *ctx);
static int wl_submit_control(wl_ctx_t *ctx);
static void wl_process_rx_stream(wl_ctx_t *ctx);
static void wl_process_rx_units(wl_ctx_t *ctx);
static void wl_prepare_reliable_payload(wl_ctx_t *ctx, const wl_wire_packet_t *pkt,
                                         uint8_t direct);

static int wl_push_event(wl_ctx_t *ctx, wl_event_type_t type, uint16_t message_id,
                        const uint8_t *payload, size_t payload_len,
                        wl_tx_handle_t handle) {
  const uint8_t is_rx = (type == WL_EVT_UNRELIABLE_RX ||
                         type == WL_EVT_RELIABLE_RX);
  if (wl_ctx_impl(ctx)->has_event || (is_rx != 0U && wl_ctx_impl(ctx)->rx_event_leased != 0U)) {
    return WL_ERR_QUEUE_FULL;
  }
  if (is_rx != 0U) {
    if (payload_len > wl_ctx_impl(ctx)->config.max_payload_len ||
        (payload_len != 0U && payload == NULL) ||
        wl_ctx_impl(ctx)->rx_candidate_source == WL_RX_SOURCE_NONE) {
      return WL_ERR_PAYLOAD_TOO_LONG;
    }
    wl_ctx_impl(ctx)->rx_payload.data = (uint8_t *)(uintptr_t)payload;
    wl_ctx_impl(ctx)->rx_payload.length = payload_len;
    wl_ctx_impl(ctx)->rx_event_generation++;
    if (wl_ctx_impl(ctx)->rx_event_generation == 0U) {
      wl_ctx_impl(ctx)->rx_event_generation = 1U;
    }
  }

  wl_ctx_impl(ctx)->event.type = type;
  wl_ctx_impl(ctx)->event.message_id = message_id;
  wl_ctx_impl(ctx)->event.payload = (is_rx != 0U) ? wl_ctx_impl(ctx)->rx_payload.data : payload;
  wl_ctx_impl(ctx)->event.payload_len = payload_len;
  wl_ctx_impl(ctx)->event.handle = handle;
  wl_ctx_impl(ctx)->event.io_result = 0;
  wl_ctx_impl(ctx)->event.lease = (is_rx != 0U) ? wl_ctx_impl(ctx)->rx_event_generation : 0U;
  wl_ctx_impl(ctx)->event.peer_session_id = 0U;
  wl_ctx_impl(ctx)->has_event = 1U;
  return WL_OK;
}

static int wl_push_tx_event(wl_ctx_t *ctx, wl_event_type_t type) {
  wl_ctx_impl_t *impl = wl_ctx_impl(ctx);
  if ((impl->tx_inflight != 0U && impl->in_flight_reliable != 0U) ||
      wl_push_event(ctx, type, 0U, NULL, 0U, impl->tx_handle) != WL_OK) {
    impl->tx_pending_event = type;
  }
  return WL_OK;
}

static int wl_complete_unreliable_tx(wl_ctx_t *ctx) {
  if (wl_ctx_impl(ctx)->tx_unreliable_completions == UINT32_MAX) {
    return WL_ERR_QUEUE_FULL;
  }

  wl_ctx_impl(ctx)->tx_unreliable_completions++;
  wl_ctx_impl(ctx)->tx_unreliable_len = 0U;
  return WL_OK;
}

static int wl_send_tx_payload(wl_ctx_t *ctx) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U || wl_ctx_impl(ctx)->sink == NULL) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->in_callback) {
    return WL_ERR_REENTRANT;
  }

  if (wl_ctx_impl(ctx)->tx_encoded_len == 0U) {
    const wl_wire_packet_t wire = {
        .type = WL_PACKET_DATA,
        .integrity = wl_ctx_impl(ctx)->config.integrity,
        .flags = WL_PACKET_FLAG_RELIABLE,
        .message_id = wl_ctx_impl(ctx)->tx_last_message_id,
        .session_id = wl_ctx_impl(ctx)->session_id,
        .sequence = wl_ctx_impl(ctx)->tx_waiting_seq,
        .payload = wl_ctx_impl(ctx)->tx_payload.data,
        .payload_len = wl_ctx_impl(ctx)->tx_payload.length,
    };
    size_t encoded_len = 0U;
    int ret = wl_frame_encode(&wire, wl_ctx_impl(ctx)->config.envelope,
        wl_ctx_impl(ctx)->storage.tx_unit, wl_ctx_impl(ctx)->storage.tx_unit_size, &encoded_len);
    if (ret != WL_OK) return ret;
    wl_ctx_impl(ctx)->tx_encoded_len = encoded_len;
  }

  return wl_submit_data(ctx, 1U);
}

/* One physical DATA resource, independent of the reliable ACK transaction. */
static int wl_submit_data(wl_ctx_t *ctx, uint8_t reliable) {
  wl_ctx_impl_t *impl = wl_ctx_impl(ctx);
  wl_sink_result_t sink_result;
  const size_t length = reliable != 0U ? impl->tx_encoded_len : impl->tx_unreliable_len;
  if (impl->sink == NULL) return WL_ERR_NOT_INITIALIZED;

  if (wl_ctx_impl(ctx)->tx_token == UINT32_MAX) {
    wl_ctx_impl(ctx)->tx_token = 1U;
  } else {
    wl_ctx_impl(ctx)->tx_token++;
  }

  wl_ctx_impl(ctx)->in_callback = 1;
  sink_result = wl_ctx_impl(ctx)->sink(wl_ctx_impl(ctx)->sink_user_data, wl_ctx_impl(ctx)->tx_token,
                          wl_ctx_impl(ctx)->storage.tx_unit, length);
  wl_ctx_impl(ctx)->in_callback = 0;
  impl->in_flight_reliable = reliable;

  if (sink_result == WL_SINK_STARTED) {
    wl_ctx_impl(ctx)->tx_inflight = 1;
    wl_ctx_impl(ctx)->tx_queued = 0U;
    if (reliable != 0U) wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_SENDING;
    return WL_OK;
  }

  if (sink_result == WL_SINK_BUSY) {
    wl_ctx_impl(ctx)->tx_inflight = 0;
    wl_ctx_impl(ctx)->tx_queued = 1U;
    if (reliable != 0U) wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_SENDING;
    return WL_OK;
  }

  if (sink_result == WL_SINK_SENT) {
    wl_ctx_impl(ctx)->tx_inflight = 0;
    wl_ctx_impl(ctx)->tx_queued = 0U;
    impl->in_flight_reliable = 0U;
    if (reliable != 0U) {
      wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_WAITING_ACK;
      wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_ACK;
      wl_ctx_impl(ctx)->tx_start_ts = wl_ctx_impl(ctx)->now_ms;
      return WL_OK;
    }

    return wl_complete_unreliable_tx(ctx);
  }

  impl->tx_queued = 0U;
  impl->tx_inflight = 0U;
  impl->in_flight_reliable = 0U;
  if (reliable == 0U) impl->tx_unreliable_len = 0U;
  return WL_ERR_IO;
}

/* Save a direct native claim lazily, only when telemetry needs its unit.
 * Ordinary reliable sends already own tx_payload. No new packet buffer, and
 * retries with no intervening DATA still reuse the encoded image unchanged. */
static void wl_invalidate_tx_image(wl_ctx_t *ctx) {
  wl_ctx_impl_t *impl = wl_ctx_impl(ctx);
  if (impl->tx_handle != 0U && impl->tx_payload.data != impl->storage.tx_payload) {
    if (impl->tx_payload.length != 0U)
      memmove(impl->storage.tx_payload, impl->tx_payload.data, impl->tx_payload.length);
    impl->tx_payload.data = impl->storage.tx_payload;
  }
  impl->tx_encoded_len = 0U;
}

static int wl_data_busy(const wl_ctx_impl_t *impl, uint8_t reliable) {
  return impl->tx_inflight != 0U || impl->control_pending != 0U ||
      impl->control_inflight != 0U || impl->tx_queued != 0U ||
      (reliable != 0U ? impl->tx_handle != 0U : wl_tx_retry_due(impl, impl->now_ms));
}

static void wl_prepare_reliable_payload(wl_ctx_t *ctx, const wl_wire_packet_t *pkt,
                                         uint8_t direct) {
  wl_ctx_impl(ctx)->tx_encoded_len = 0U;
  wl_ctx_impl(ctx)->tx_last_message_id = pkt->message_id;
  wl_ctx_impl(ctx)->tx_payload.data = direct != 0U
                                          ? (uint8_t *)(uintptr_t)pkt->payload
                                          : wl_ctx_impl(ctx)->storage.tx_payload;
  wl_ctx_impl(ctx)->tx_payload.length = pkt->payload_len;
  if (direct == 0U && pkt->payload_len != 0U) {
    memcpy(wl_ctx_impl(ctx)->storage.tx_payload, pkt->payload, pkt->payload_len);
  }
}

wl_err_t wl_tx_complete(wl_ctx_t *ctx, wl_io_token_t token, int io_result) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->control_inflight != 0U && wl_ctx_impl(ctx)->tx_token == token) {
    wl_ctx_impl(ctx)->control_inflight = 0U;
    wl_ctx_impl(ctx)->control_pending = 0U;
    return (io_result == WL_OK) ? WL_OK : WL_ERR_IO;
  }
  if (wl_ctx_impl(ctx)->tx_token != token) {
    return WL_ERR_NOT_FOUND;
  }
  if (wl_ctx_impl(ctx)->tx_inflight == 0U) return WL_OK;
  const uint8_t reliable = wl_ctx_impl(ctx)->in_flight_reliable;
  wl_ctx_impl(ctx)->tx_inflight = 0U;
  wl_ctx_impl(ctx)->in_flight_reliable = 0U;
  if (reliable == 0U) {
    const int result = io_result == WL_OK ? wl_complete_unreliable_tx(ctx) : WL_OK;
    wl_ctx_impl(ctx)->tx_unreliable_len = 0U;
    (void)wl_submit_control(ctx);
    return result;
  }
  if (wl_ctx_impl(ctx)->tx_cancel_requested != 0U) {
    wl_ctx_impl(ctx)->tx_cancel_requested = 0U;
    wl_ctx_impl(ctx)->tx_payload.data = wl_ctx_impl(ctx)->storage.tx_payload;
    wl_ctx_impl(ctx)->tx_payload.length = 0U;
    wl_ctx_impl(ctx)->tx_encoded_len = 0U;
    (void)wl_submit_control(ctx);
    return WL_OK;
  }
  /* An ACK for a previous attempt can arrive during physical retransmission.
   * Drain that I/O lease without reopening an already completed transaction. */
  if (wl_ctx_impl(ctx)->tx_state != WL_TX_STATE_SENDING) {
    (void)wl_submit_control(ctx);
    return WL_OK;
  }

  if (io_result == WL_OK) {
    wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_WAITING_ACK;
    wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_ACK;
    wl_ctx_impl(ctx)->tx_start_ts = wl_ctx_impl(ctx)->now_ms;
    (void)wl_submit_control(ctx);
    return WL_OK;
  }

  if (wl_ctx_impl(ctx)->tx_retries_left != 0U) {
    int retry;
    if (wl_ctx_impl(ctx)->control_pending != 0U) {
      /* A queued ACK owns the next physical turn, even after a DATA I/O error. */
      wl_ctx_impl(ctx)->tx_queued = 1U;
      wl_ctx_impl(ctx)->in_flight_reliable = 1U;
      retry = WL_OK;
    } else retry = wl_send_tx_payload(ctx);
    if (retry == WL_OK) {
      wl_ctx_impl(ctx)->tx_retries_left--;
      wl_ctx_impl(ctx)->tx_retries_used++;
      (void)wl_submit_control(ctx);
      return WL_OK;
    }
  }
  wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_FAILED;
  wl_ctx_impl(ctx)->tx_result_code = WL_ERR_IO;
  wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
  wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
  (void)wl_push_tx_event(ctx, WL_EVT_TX_FAILED);
  (void)wl_submit_control(ctx);
  return WL_OK;
}

static int wl_send_ack(wl_ctx_t *ctx, const wl_frame_view_t *view) {
  size_t encoded_len = 0;
  wl_wire_packet_t ack = {0};

  if (ctx == NULL || view == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U || wl_ctx_impl(ctx)->sink == NULL) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->in_callback) {
    return WL_ERR_REENTRANT;
  }

  ack.type = WL_PACKET_ACK;
  ack.flags = 0U;
  ack.message_id = 0U;
  ack.session_id = view->session_id;
  ack.sequence = view->sequence;
  ack.payload = NULL;
  ack.payload_len = 0U;
  ack.integrity = wl_ctx_impl(ctx)->config.integrity;

  if (wl_ctx_impl(ctx)->control_pending != 0U) {
    return WL_ERR_QUEUE_FULL;
  }
  int ret = wl_frame_encode(&ack, wl_ctx_impl(ctx)->config.envelope, wl_ctx_impl(ctx)->storage.control_unit,
                            wl_ctx_impl(ctx)->storage.control_unit_size, &encoded_len);
  if (ret != WL_OK) {
    return ret;
  }
  wl_ctx_impl(ctx)->control_len = encoded_len;
  wl_ctx_impl(ctx)->control_pending = 1U;
  return wl_submit_control(ctx);
}

static int wl_submit_control(wl_ctx_t *ctx) {
  wl_sink_result_t sink_result;
  if (ctx == NULL || wl_ctx_impl(ctx)->control_pending == 0U ||
      wl_ctx_impl(ctx)->control_inflight != 0U || wl_ctx_impl(ctx)->tx_inflight != 0U) {
    return WL_OK;
  }
  if (wl_ctx_impl(ctx)->sink == NULL) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->in_callback) {
    return WL_ERR_REENTRANT;
  }
  if (wl_ctx_impl(ctx)->tx_token == UINT32_MAX) {
    wl_ctx_impl(ctx)->tx_token = 1U;
  } else {
    wl_ctx_impl(ctx)->tx_token++;
  }
  wl_ctx_impl(ctx)->in_callback = 1;
  sink_result = wl_ctx_impl(ctx)->sink(wl_ctx_impl(ctx)->sink_user_data, wl_ctx_impl(ctx)->tx_token,
                          wl_ctx_impl(ctx)->storage.control_unit, wl_ctx_impl(ctx)->control_len);
  wl_ctx_impl(ctx)->in_callback = 0;
  if (sink_result == WL_SINK_SENT) {
    wl_ctx_impl(ctx)->control_pending = 0U;
    return WL_OK;
  }
  if (sink_result == WL_SINK_STARTED) {
    wl_ctx_impl(ctx)->control_inflight = 1U;
    return WL_OK;
  }
  if (sink_result == WL_SINK_BUSY) {
    return WL_OK;
  }
  wl_ctx_impl(ctx)->control_pending = 0U;
  return WL_ERR_IO;
}

static int wl_handle_ack(wl_ctx_t *ctx, const wl_frame_view_t *view) {
  if (ctx == NULL || view == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->tx_wait_state != WL_TX_WAIT_ACK ||
      (wl_ctx_impl(ctx)->tx_state != WL_TX_STATE_WAITING_ACK &&
       wl_ctx_impl(ctx)->tx_state != WL_TX_STATE_SENDING)) {
    return WL_OK;
  }

  if (wl_ctx_impl(ctx)->session_id != view->session_id ||
      wl_ctx_impl(ctx)->tx_waiting_seq != view->sequence) {
    return WL_OK;
  }

  wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_SUCCESS;
  wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
  wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
  wl_ctx_impl(ctx)->tx_retries_left = 0U;
  wl_ctx_impl(ctx)->tx_result_code = WL_OK;
  if (wl_ctx_impl(ctx)->tx_queued != 0U && wl_ctx_impl(ctx)->in_flight_reliable != 0U) {
    wl_ctx_impl(ctx)->tx_queued = 0U;
    wl_ctx_impl(ctx)->in_flight_reliable = 0U;
  }
  return wl_push_tx_event(ctx, WL_EVT_TX_SUCCESS);
}

static int wl_send_frame_internal(wl_ctx_t *ctx, const wl_wire_packet_t *pkt,
                                 wl_tx_handle_t *out_handle,
                                 uint8_t reliable, uint8_t direct,
                                 wl_time_ms_t now_ms) {
  wl_tx_handle_t generated_handle;

  if (ctx == NULL || pkt == NULL || (pkt->payload_len != 0U && pkt->payload == NULL)) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->session_id == 0ULL) {
    /* session_id=0 is reserved by protocol; fail fast before serialization. */
    return WL_ERR_INVALID_ARG;
  }
  if (pkt->payload_len > wl_ctx_impl(ctx)->config.max_payload_len) {
    return WL_ERR_PAYLOAD_TOO_LONG;
  }
  if (wl_data_busy(wl_ctx_impl(ctx), reliable)) {
    return WL_ERR_BUSY;
  }
  if (wl_ctx_impl(ctx)->in_callback) {
    return WL_ERR_REENTRANT;
  }
  if (reliable == 0U &&
      wl_ctx_impl(ctx)->tx_unreliable_completions == UINT32_MAX) {
    return WL_ERR_QUEUE_FULL;
  }

  if (reliable == 0U) {
    wl_wire_packet_t wire = *pkt;
    wire.integrity = wl_ctx_impl(ctx)->config.integrity;
    wl_invalidate_tx_image(ctx);
    int result = wl_frame_encode(&wire, wl_ctx_impl(ctx)->config.envelope,
        wl_ctx_impl(ctx)->storage.tx_unit, wl_ctx_impl(ctx)->storage.tx_unit_size,
        &wl_ctx_impl(ctx)->tx_unreliable_len);
    if (result != WL_OK) return result;
    if (out_handle != NULL) *out_handle = 0U;
    return wl_submit_data(ctx, 0U);
  }

  wl_ctx_impl(ctx)->now_ms = now_ms;
  wl_prepare_reliable_payload(ctx, pkt, direct);
  wl_ctx_impl(ctx)->tx_generation++;
  if (wl_ctx_impl(ctx)->tx_generation == 0U) {
    wl_ctx_impl(ctx)->tx_generation = 1U;
  }
  generated_handle = ((uint32_t)wl_ctx_impl(ctx)->tx_generation << 16U) | 1U;
  wl_ctx_impl(ctx)->tx_handle = generated_handle;
  wl_ctx_impl(ctx)->tx_waiting_seq = pkt->sequence;
  wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
  wl_ctx_impl(ctx)->tx_retries_left = wl_ctx_impl(ctx)->tx_retries_max;
  wl_ctx_impl(ctx)->tx_retries_used = 0U;

  int ret = wl_send_tx_payload(ctx);
  if (ret != WL_OK) {
    wl_ctx_impl(ctx)->tx_handle = 0U;
    wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_IDLE;
    if (out_handle != NULL) {
      *out_handle = 0U;
    }
    wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
    wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
    wl_ctx_impl(ctx)->tx_retries_left = 0U;
    wl_ctx_impl(ctx)->tx_payload.data = wl_ctx_impl(ctx)->storage.tx_payload;
    wl_ctx_impl(ctx)->tx_payload.length = 0U;
    wl_ctx_impl(ctx)->tx_last_message_id = 0U;
    wl_ctx_impl(ctx)->tx_encoded_len = 0U;
    return ret;
  }

  wl_ctx_impl(ctx)->tx_result_code = WL_ERR_BUSY;

  if (out_handle != NULL) {
    *out_handle = generated_handle;
  }
  return WL_OK;
}

wl_err_t wl_send_unreliable(wl_ctx_t *ctx, uint16_t message_id,
                            const uint8_t *payload, size_t payload_len) {
  wl_wire_packet_t pkt;

  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  memset(&pkt, 0, sizeof(pkt));
  pkt.type = WL_PACKET_DATA;
  pkt.flags = 0U;
  pkt.message_id = message_id;
  pkt.sequence = 0U;
  pkt.payload = payload;
  pkt.payload_len = payload_len;

  if (wl_ctx_impl(ctx)->tx_claim_active != 0U) {
    return WL_ERR_BUSY;
  }
  return wl_send_frame_internal(ctx, &pkt, NULL, 0U, 0U, 0U);
}

wl_err_t wl_send_reliable(wl_ctx_t *ctx, uint16_t message_id,
                          const uint8_t *payload, size_t payload_len,
                          wl_time_ms_t now_ms,
                          wl_tx_handle_t *out_handle) {
  wl_wire_packet_t pkt;

  if (ctx == NULL || out_handle == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->tx_sequence == UINT32_MAX) {
    return WL_ERR_INVALID_STATE;
  }

  memset(&pkt, 0, sizeof(pkt));
  pkt.type = WL_PACKET_DATA;
  pkt.flags = WL_PACKET_FLAG_RELIABLE;
  pkt.message_id = message_id;
  pkt.sequence = wl_ctx_impl(ctx)->tx_sequence++;
  pkt.payload = payload;
  pkt.payload_len = payload_len;

  *out_handle = 0;
  if (wl_ctx_impl(ctx)->tx_claim_active != 0U) {
    return WL_ERR_BUSY;
  }
  return wl_send_frame_internal(ctx, &pkt, out_handle, 1U, 0U, now_ms);
}

wl_err_t wl_tx_payload_claim(wl_ctx_t *ctx, uint16_t message_id,
                             wl_delivery_t delivery,
                             wl_tx_payload_claim_t *out_claim) {
  uint8_t *payload;

  if (ctx == NULL || out_claim == NULL || message_id == 0U ||
      (delivery != WL_DELIVERY_UNRELIABLE &&
       delivery != WL_DELIVERY_RELIABLE)) {
    return WL_ERR_INVALID_ARG;
  }
  memset(out_claim, 0, sizeof(*out_claim));
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->tx_claim_active != 0U ||
      wl_data_busy(wl_ctx_impl(ctx), delivery == WL_DELIVERY_RELIABLE)) {
    return WL_ERR_BUSY;
  }

  if (delivery == WL_DELIVERY_UNRELIABLE &&
      wl_ctx_impl(ctx)->tx_unreliable_completions == UINT32_MAX) {
    return WL_ERR_QUEUE_FULL;
  }
  if (delivery == WL_DELIVERY_UNRELIABLE) wl_invalidate_tx_image(ctx);

  if (wl_ctx_impl(ctx)->config.envelope == WL_ENVELOPE_NATIVE_PACKET) {
    payload = wl_ctx_impl(ctx)->storage.tx_unit +
              wl_frame_packet_header_size(
                  WL_PACKET_DATA,
                  delivery == WL_DELIVERY_RELIABLE
                      ? WL_PACKET_FLAG_RELIABLE
                      : 0U);
  } else if (delivery == WL_DELIVERY_UNRELIABLE && wl_ctx_impl(ctx)->tx_handle != 0U) {
    /* tx_payload retains the reliable transaction. In-place frame encoding
     * stages this claim safely inside the existing final-unit capacity. */
    payload = wl_ctx_impl(ctx)->storage.tx_unit;
  } else {
    /* Stream and length-prefixed envelopes still need a distinct encoded
     * unit, but generated codecs can avoid an extra application scratch copy
     * by constructing the payload directly in the retained staging region. */
    payload = wl_ctx_impl(ctx)->storage.tx_payload;
  }
  ++wl_ctx_impl(ctx)->tx_claim_token;
  if (wl_ctx_impl(ctx)->tx_claim_token == 0U) {
    ++wl_ctx_impl(ctx)->tx_claim_token;
  }
  wl_ctx_impl(ctx)->tx_claim_active = 1U;
  wl_ctx_impl(ctx)->tx_claim_reliable =
      delivery == WL_DELIVERY_RELIABLE ? 1U : 0U;
  wl_ctx_impl(ctx)->tx_claim_message_id = message_id;
  wl_ctx_impl(ctx)->tx_claim_payload =
      (wl_span_t){payload, wl_ctx_impl(ctx)->config.max_payload_len};
  out_claim->span = wl_ctx_impl(ctx)->tx_claim_payload;
  out_claim->token = wl_ctx_impl(ctx)->tx_claim_token;
  return WL_OK;
}

static int tx_claim_matches(const wl_ctx_impl_t *impl,
                            const wl_tx_payload_claim_t *claim) {
  return impl->tx_claim_active != 0U && claim != NULL &&
         claim->token == impl->tx_claim_token &&
         claim->span.data == impl->tx_claim_payload.data;
}

wl_err_t wl_tx_payload_commit(wl_ctx_t *ctx,
                              const wl_tx_payload_claim_t *claim,
                              size_t payload_len,
                              wl_time_ms_t now_ms,
                              wl_tx_handle_t *out_handle) {
  wl_wire_packet_t packet = {0};
  uint8_t reliable;

  if (ctx == NULL || claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!tx_claim_matches(wl_ctx_impl(ctx), claim)) {
    return WL_ERR_NOT_FOUND;
  }
  reliable = wl_ctx_impl(ctx)->tx_claim_reliable;
  wl_ctx_impl(ctx)->tx_claim_active = 0U;
  wl_ctx_impl(ctx)->tx_claim_payload = (wl_span_t){0};
  if (out_handle != NULL) {
    *out_handle = 0U;
  }
  if (payload_len > claim->span.length ||
      (reliable != 0U && out_handle == NULL)) {
    return WL_ERR_INVALID_ARG;
  }
  if (reliable != 0U && wl_ctx_impl(ctx)->tx_sequence == UINT32_MAX) {
    return WL_ERR_INVALID_STATE;
  }

  packet.type = WL_PACKET_DATA;
  packet.integrity = wl_ctx_impl(ctx)->config.integrity;
  packet.flags = reliable != 0U ? WL_PACKET_FLAG_RELIABLE : 0U;
  packet.message_id = wl_ctx_impl(ctx)->tx_claim_message_id;
  packet.session_id = wl_ctx_impl(ctx)->session_id;
  packet.sequence = reliable != 0U ? wl_ctx_impl(ctx)->tx_sequence++ : 0U;
  packet.payload = claim->span.data;
  packet.payload_len = payload_len;
  return wl_send_frame_internal(ctx, &packet, out_handle, reliable, 1U, now_ms);
}

wl_err_t wl_tx_payload_abort(wl_ctx_t *ctx,
                             const wl_tx_payload_claim_t *claim) {
  if (ctx == NULL || claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!tx_claim_matches(wl_ctx_impl(ctx), claim)) {
    return WL_ERR_NOT_FOUND;
  }
  wl_ctx_impl(ctx)->tx_claim_active = 0U;
  wl_ctx_impl(ctx)->tx_claim_payload = (wl_span_t){0};
  return WL_OK;
}

static int wl_feed_parse_wire(wl_ctx_t *ctx, const uint8_t *data, size_t len) {
  wl_frame_view_t view;
  int ret;

  if (ctx == NULL || data == NULL || len == 0U) {
    return WL_ERR_INVALID_ARG;
  }

  ret = wl_frame_decode(data, len,
                        wl_ctx_impl(ctx)->initialized != 0U
                            ? wl_ctx_impl(ctx)->config.integrity
                            : WL_INTEGRITY_NONE,
                       &view);
  if (ret != WL_OK) {
    if (ret == WL_ERR_CRC) {
      wl_ctx_impl(ctx)->rx_counters.bad_integrity++;
    } else if (ret == WL_ERR_NOT_SUPPORTED) {
      wl_ctx_impl(ctx)->rx_counters.unsupported++;
    } else {
      wl_ctx_impl(ctx)->rx_counters.malformed++;
    }
    return ret;
  }

  if (view.payload.length > wl_ctx_impl(ctx)->config.max_payload_len) {
    return WL_ERR_PAYLOAD_TOO_LONG;
  }

  switch (view.type) {
  case WL_PACKET_DATA:
    if ((view.flags & WL_PACKET_FLAG_RELIABLE) != 0U) {
      if (wl_ctx_impl(ctx)->rx_have_reliable != 0U &&
          wl_ctx_impl(ctx)->rx_session_id == view.session_id &&
          wl_ctx_impl(ctx)->rx_sequence == view.sequence) {
        wl_ctx_impl(ctx)->rx_counters.duplicate++;
        return wl_send_ack(ctx, &view);
      }
      ret = wl_push_event(ctx, WL_EVT_RELIABLE_RX, view.message_id,
                          view.payload.data, view.payload.length, 0U);
      if (ret != WL_OK) {
        return ret;
      }
      wl_ctx_impl(ctx)->event.peer_session_id = view.session_id;
      wl_ctx_impl(ctx)->rx_session_id = view.session_id;
      wl_ctx_impl(ctx)->rx_sequence = view.sequence;
      wl_ctx_impl(ctx)->rx_have_reliable = 1U;
      return wl_send_ack(ctx, &view);
    }
    return wl_push_event(ctx, WL_EVT_UNRELIABLE_RX, view.message_id, view.payload.data,
                        view.payload.length, 0U);
  case WL_PACKET_ACK:
    return wl_handle_ack(ctx, &view);
  case WL_PACKET_NACK:
    wl_ctx_impl(ctx)->rx_counters.unsupported++;
    return WL_ERR_NOT_SUPPORTED;
  default:
    return WL_OK;
  }
}

wl_err_t wl_feed_unit(wl_ctx_t *ctx, const uint8_t *unit, size_t len) {
  size_t accepted = 0U;
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (unit == NULL && len != 0U) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }

  if (wl_ctx_impl(ctx)->config.envelope == WL_ENVELOPE_COBS_STREAM) {
    return wl_feed_bytes(ctx, unit, len, &accepted);
  }

  if (wl_ctx_impl(ctx)->has_event != 0U || wl_ctx_impl(ctx)->rx_event_leased != 0U) {
    return WL_ERR_WOULD_BLOCK;
  }
  if (len > wl_ctx_impl(ctx)->storage.rx_fallback_size) {
    return WL_ERR_PAYLOAD_TOO_LONG;
  }
  if (len != 0U) {
    memcpy(wl_ctx_impl(ctx)->storage.rx_fallback, unit, len);
  }
  wl_ctx_impl(ctx)->rx_candidate_source = WL_RX_SOURCE_FALLBACK;
  return wl_feed_unit_raw(ctx, wl_ctx_impl(ctx)->storage.rx_fallback, len);
}

static int wl_feed_unit_raw(wl_ctx_t *ctx, const uint8_t *unit, size_t len) {
  size_t raw_len;
  if (ctx == NULL || unit == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->config.envelope == WL_ENVELOPE_BUS_LENGTH16) {
    if (len < 2U) {
      wl_ctx_impl(ctx)->rx_counters.malformed++;
      return WL_ERR_BAD_FRAME;
    }
    raw_len = ((size_t)unit[0] << 8U) | unit[1];
    if (raw_len + 2U > len ||
        (wl_ctx_impl(ctx)->config.max_transmission_unit != 0U &&
         raw_len + 2U > wl_ctx_impl(ctx)->config.max_transmission_unit)) {
      wl_ctx_impl(ctx)->rx_counters.malformed++;
      return WL_ERR_BAD_FRAME;
    }
    for (size_t i = raw_len + 2U; i < len; ++i) {
      if (unit[i] != 0U) {
        wl_ctx_impl(ctx)->rx_counters.malformed++;
        return WL_ERR_BAD_FRAME;
      }
    }
    return wl_feed_parse_wire(ctx, unit + 2U, raw_len);
  }
  return wl_feed_parse_wire(ctx, unit, len);
}

wl_err_t wl_rx_reserve(wl_ctx_t *ctx, wl_span_t *out_span) {
  if (ctx == NULL || out_span == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return wl_rx_ring_producer_reserve(&wl_ctx_impl(ctx)->rx_ring, out_span);
}

wl_err_t wl_rx_commit(wl_ctx_t *ctx, size_t len) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return wl_rx_ring_producer_commit(&wl_ctx_impl(ctx)->rx_ring, len);
}

wl_err_t wl_rx_dma_claim(wl_ctx_t *ctx, size_t maximum_length,
                         wl_rx_dma_claim_t *out_claim) {
  if (ctx == NULL || out_claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return wl_rx_ring_dma_claim(&wl_ctx_impl(ctx)->rx_ring, maximum_length, out_claim);
}

wl_err_t wl_rx_dma_publish(wl_ctx_t *ctx,
                           const wl_rx_dma_claim_t *claim, size_t offset,
                           size_t length) {
  if (ctx == NULL || claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return wl_rx_ring_dma_publish(&wl_ctx_impl(ctx)->rx_ring, claim, offset, length);
}

wl_err_t wl_rx_dma_finish(wl_ctx_t *ctx,
                          const wl_rx_dma_claim_t *claim) {
  if (ctx == NULL || claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return wl_rx_ring_dma_finish(&wl_ctx_impl(ctx)->rx_ring, claim);
}

wl_err_t wl_rx_dma_abort(wl_ctx_t *ctx) {
  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return wl_rx_ring_dma_abort(&wl_ctx_impl(ctx)->rx_ring);
}

void wl_rx_note_overflow(wl_ctx_t *ctx) {
  if (ctx != NULL && wl_ctx_impl(ctx)->initialized != 0U &&
      wl_ctx_impl(ctx)->config.envelope == WL_ENVELOPE_COBS_STREAM) {
    wl_rx_ring_producer_note_overflow(&wl_ctx_impl(ctx)->rx_ring);
  }
}

wl_err_t wl_feed_bytes(wl_ctx_t *ctx, const uint8_t *data, size_t len,
                       size_t *out_accepted) {
  size_t accepted = 0U;

  if (ctx == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if ((data == NULL && len != 0U) || out_accepted == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  *out_accepted = 0U;
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (wl_ctx_impl(ctx)->config.envelope != WL_ENVELOPE_COBS_STREAM) {
    return WL_ERR_NOT_SUPPORTED;
  }

  while (accepted < len) {
    wl_span_t span = {0};
    size_t chunk;
    int ret = wl_rx_reserve(ctx, &span);
    if (ret != WL_OK) {
      *out_accepted = accepted;
      return ret;
    }
    if (span.length == 0U) {
      (void)wl_rx_commit(ctx, 0U);
      break;
    }
    chunk = len - accepted;
    if (chunk > span.length) {
      chunk = span.length;
    }
    memcpy(span.data, data + accepted, chunk);
    ret = wl_rx_commit(ctx, chunk);
    if (ret != WL_OK) {
      *out_accepted = accepted;
      return ret;
    }
    accepted += chunk;
  }

  *out_accepted = accepted;
  if (accepted != len) {
    wl_rx_ring_producer_note_overflow(&wl_ctx_impl(ctx)->rx_ring);
    return WL_ERR_WOULD_BLOCK;
  }
  return WL_OK;
}

static void wl_process_rx_stream(wl_ctx_t *ctx) {
  unsigned int overflow_events =
      wl_rx_ring_consumer_take_overflow(&wl_ctx_impl(ctx)->rx_ring);

  if (overflow_events != 0U) {
    size_t readable = wl_rx_ring_readable(&wl_ctx_impl(ctx)->rx_ring);
    uint32_t remaining = UINT32_MAX - wl_ctx_impl(ctx)->rx_counters.overflow;
    uint32_t increment = overflow_events > remaining
                             ? remaining
                             : (uint32_t)overflow_events;

    wl_ctx_impl(ctx)->rx_counters.overflow += increment;
    (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring, readable);
  }

  while (wl_ctx_impl(ctx)->has_event == 0U && wl_ctx_impl(ctx)->rx_event_leased == 0U) {
    wl_span_t contiguous;
    uint8_t *encoded;
    size_t delimiter_offset;
    size_t decoded_len = 0U;
    size_t physical_len;
    int ret;

    ret = wl_rx_ring_consumer_find(&wl_ctx_impl(ctx)->rx_ring, 0U, &delimiter_offset);
    if (ret != WL_OK) {
      return;
    }
    physical_len = delimiter_offset + 1U;
    if (delimiter_offset == 0U) {
      (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring, physical_len);
      continue;
    }

    contiguous = wl_rx_ring_consumer_peek(&wl_ctx_impl(ctx)->rx_ring);
    if (delimiter_offset <= contiguous.length) {
      encoded = contiguous.data;
      wl_ctx_impl(ctx)->rx_candidate_source = WL_RX_SOURCE_RING;
    } else {
      if (delimiter_offset > wl_ctx_impl(ctx)->storage.rx_fallback_size ||
          wl_rx_ring_consumer_copy(&wl_ctx_impl(ctx)->rx_ring, 0U,
                                   wl_ctx_impl(ctx)->storage.rx_fallback,
                                   delimiter_offset) != WL_OK) {
        wl_ctx_impl(ctx)->rx_counters.overflow++;
        (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring, physical_len);
        continue;
      }
      encoded = wl_ctx_impl(ctx)->storage.rx_fallback;
      wl_ctx_impl(ctx)->rx_candidate_source = WL_RX_SOURCE_FALLBACK;
    }

    ret = wl_cobs_decode_in_place(encoded, delimiter_offset, &decoded_len);
    if (ret == WL_OK) {
      ret = wl_feed_parse_wire(ctx, encoded, decoded_len);
    } else {
      wl_ctx_impl(ctx)->rx_counters.malformed++;
    }

    if (wl_ctx_impl(ctx)->has_event != 0U &&
        (wl_ctx_impl(ctx)->event.type == WL_EVT_UNRELIABLE_RX ||
         wl_ctx_impl(ctx)->event.type == WL_EVT_RELIABLE_RX)) {
      if (wl_ctx_impl(ctx)->rx_candidate_source == WL_RX_SOURCE_RING) {
        wl_ctx_impl(ctx)->rx_pending_consume = physical_len;
      } else {
        (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring, physical_len);
        wl_ctx_impl(ctx)->rx_pending_consume = 0U;
      }
      return;
    }

    (void)wl_rx_ring_consumer_consume(&wl_ctx_impl(ctx)->rx_ring, physical_len);
    wl_ctx_impl(ctx)->rx_candidate_source = WL_RX_SOURCE_NONE;
    if (wl_ctx_impl(ctx)->has_event != 0U) {
      return;
    }
    (void)ret;
  }
}

static void wl_process_rx_units(wl_ctx_t *ctx) {
  while (wl_ctx_impl(ctx)->has_event == 0U &&
         wl_ctx_impl(ctx)->rx_event_leased == 0U) {
    wl_span_t unit = wl_rx_unit_consumer_peek(ctx);
    int ret;

    if (unit.data == NULL || unit.length == 0U) {
      return;
    }
    wl_ctx_impl(ctx)->rx_candidate_source = WL_RX_SOURCE_UNIT;
    ret = wl_feed_unit_raw(ctx, unit.data, unit.length);
    if (wl_ctx_impl(ctx)->has_event != 0U &&
        (wl_ctx_impl(ctx)->event.type == WL_EVT_UNRELIABLE_RX ||
         wl_ctx_impl(ctx)->event.type == WL_EVT_RELIABLE_RX)) {
      wl_ctx_impl(ctx)->rx_pending_consume = 1U;
      return;
    }
    (void)wl_rx_unit_consumer_consume(ctx);
    wl_ctx_impl(ctx)->rx_candidate_source = WL_RX_SOURCE_NONE;
    if (wl_ctx_impl(ctx)->has_event != 0U) {
      return;
    }
    (void)ret;
  }
}

wl_err_t wl_poll(wl_ctx_t *ctx, wl_time_ms_t now_ms,
                 wl_event_t *out_event) {
  if (ctx == NULL || out_event == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  memset(out_event, 0, sizeof(*out_event));
  wl_ctx_impl(ctx)->now_ms = now_ms;

  (void)wl_submit_control(ctx);

  if (wl_ctx_impl(ctx)->tx_pending_event != WL_EVT_NONE &&
      wl_ctx_impl(ctx)->has_event == 0U &&
      !(wl_ctx_impl(ctx)->tx_inflight != 0U && wl_ctx_impl(ctx)->in_flight_reliable != 0U)) {
    (void)wl_push_event(ctx, wl_ctx_impl(ctx)->tx_pending_event, 0U, NULL, 0U,
                        wl_ctx_impl(ctx)->tx_handle);
    wl_ctx_impl(ctx)->tx_pending_event = WL_EVT_NONE;
  }

  /* A previously queued ACK keeps priority because the control slot cannot be
   * overwritten. Once it drains, parse received input before retrying data TX
   * so a reliable RX event can expose a restarted peer's new session first. */
  if (wl_ctx_impl(ctx)->control_pending == 0U && wl_ctx_impl(ctx)->has_event == 0U &&
      wl_ctx_impl(ctx)->rx_event_leased == 0U &&
      wl_ctx_impl(ctx)->config.envelope == WL_ENVELOPE_COBS_STREAM) {
    wl_process_rx_stream(ctx);
  }
  if (wl_ctx_impl(ctx)->control_pending == 0U && wl_ctx_impl(ctx)->has_event == 0U &&
      wl_ctx_impl(ctx)->rx_event_leased == 0U &&
      wl_ctx_impl(ctx)->rx_units.initialized != 0U) {
    wl_process_rx_units(ctx);
  }

  if (wl_ctx_impl(ctx)->has_event == 0U &&
      wl_ctx_impl(ctx)->control_pending == 0U &&
      wl_ctx_impl(ctx)->tx_claim_active == 0U &&
      wl_ctx_impl(ctx)->tx_queued != 0U && wl_ctx_impl(ctx)->tx_inflight == 0U) {
    const uint8_t reliable = wl_ctx_impl(ctx)->in_flight_reliable;
    const int sent = reliable != 0U ? wl_send_tx_payload(ctx) : wl_submit_data(ctx, 0U);
    if (sent != WL_OK) {
      wl_ctx_impl(ctx)->tx_queued = 0U;
      wl_ctx_impl(ctx)->in_flight_reliable = 0U;
      if (reliable != 0U) {
        wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_FAILED;
        wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
        wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
        wl_ctx_impl(ctx)->tx_result_code = WL_ERR_IO;
        (void)wl_push_tx_event(ctx, WL_EVT_TX_FAILED);
      } else wl_ctx_impl(ctx)->tx_unreliable_len = 0U;
    }
  }

  if (wl_ctx_impl(ctx)->has_event == 0U &&
      wl_ctx_impl(ctx)->control_pending == 0U &&
      wl_ctx_impl(ctx)->tx_inflight == 0U && wl_ctx_impl(ctx)->tx_queued == 0U &&
      wl_ctx_impl(ctx)->tx_claim_active == 0U &&
      wl_ctx_impl(ctx)->tx_state == WL_TX_STATE_WAITING_ACK &&
      wl_ctx_impl(ctx)->initialized != 0U && wl_ctx_impl(ctx)->config.ack_timeout_ms != 0U) {
    wl_time_ms_t timeout_ms = now_ms - wl_ctx_impl(ctx)->tx_start_ts;
    if (timeout_ms >= wl_ctx_impl(ctx)->config.ack_timeout_ms) {
      if (wl_ctx_impl(ctx)->tx_retries_left == 0U) {
        wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_FAILED;
        wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
        wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
        wl_ctx_impl(ctx)->tx_result_code = WL_ERR_TIMEOUT;
        (void)wl_push_tx_event(ctx, WL_EVT_TX_TIMEOUT);
      } else {
        int retry = wl_send_tx_payload(ctx);
        if (retry == WL_OK) {
          wl_ctx_impl(ctx)->tx_retries_left--;
          wl_ctx_impl(ctx)->tx_retries_used++;
          if (wl_ctx_impl(ctx)->tx_state == WL_TX_STATE_WAITING_ACK) {
            wl_ctx_impl(ctx)->tx_start_ts = now_ms;
          }
        } else {
          wl_ctx_impl(ctx)->tx_state = WL_TX_STATE_FAILED;
          wl_ctx_impl(ctx)->tx_wait_state = WL_TX_WAIT_NONE;
          wl_ctx_impl(ctx)->tx_waiting_seq = 0U;
          wl_ctx_impl(ctx)->tx_result_code = WL_ERR_IO;
          (void)wl_push_tx_event(ctx, WL_EVT_TX_FAILED);
        }
      }
    }
  }

  if (wl_ctx_impl(ctx)->has_event == 0U &&
      wl_ctx_impl(ctx)->tx_unreliable_completions != 0U &&
      wl_push_event(ctx, WL_EVT_TX_SUCCESS, 0U, NULL, 0U, 0U) == WL_OK) {
    wl_ctx_impl(ctx)->tx_unreliable_completions--;
  }

  if (!wl_ctx_impl(ctx)->has_event) {
    return WL_ERR_NO_DATA;
  }

  *out_event = wl_ctx_impl(ctx)->event;
  wl_ctx_impl(ctx)->has_event = 0;
  if (out_event->type == WL_EVT_UNRELIABLE_RX ||
      out_event->type == WL_EVT_RELIABLE_RX) {
    wl_ctx_impl(ctx)->rx_event_leased = 1U;
  }

  return WL_OK;
}
