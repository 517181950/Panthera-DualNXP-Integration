/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WIRELINK_SRC_CONTEXT_H_
#define WIRELINK_SRC_CONTEXT_H_

#include <stddef.h>
#include <stdatomic.h>
#include <stdint.h>

#include "wirelink/rx_ring_state.h"
#include "wirelink/wirelink.h"

enum {
  WL_RX_SOURCE_NONE = 0,
  WL_RX_SOURCE_RING,
  WL_RX_SOURCE_FALLBACK,
  WL_RX_SOURCE_UNIT,
};

typedef struct {
  uint8_t *storage;
  size_t unit_size;
  size_t lengths[WL_RX_UNIT_QUEUE_MAX_SLOTS];
  _Atomic uint32_t write_cursor;
  _Atomic uint32_t read_cursor;
  uint32_t claim_cursor;
  uint32_t claim_token;
  uint8_t slot_count;
  uint8_t claim_active;
  uint8_t initialized;
} wl_rx_unit_queue_state_t;

typedef struct {
  wl_rx_ring_state_t rx_ring;
  wl_rx_unit_queue_state_t rx_units;

  wl_config_t config;
  wl_storage_t storage;
  uint8_t initialized;

  wl_sink_fn sink;
  void *sink_user_data;

  /* Logical reliable transaction. Unreliable I/O never changes its state. */
  wl_tx_state_t tx_state;
  wl_event_type_t tx_pending_event; /* Publish only after the DATA lease drains. */
  wl_tx_handle_t tx_handle;
  uint32_t tx_unreliable_completions;
  wl_io_token_t tx_token;
  wl_time_ms_t tx_start_ts;
  uint16_t tx_retries_left;
  uint16_t tx_retries_max;
  uint16_t tx_retries_used;
  uint32_t tx_sequence;
  uint32_t tx_waiting_seq;
  uint8_t tx_inflight;
  wl_tx_wait_reason_t tx_wait_state;
  uint8_t tx_cancel_requested;
  uint16_t tx_generation;
  int tx_result_code;
  uint16_t tx_last_message_id;
  uint8_t tx_claim_active;
  uint8_t tx_claim_reliable;
  uint16_t tx_claim_message_id;
  uint32_t tx_claim_token;
  wl_span_t tx_claim_payload;

  uint64_t session_id;
  uint32_t rx_sequence;
  uint64_t rx_session_id;
  uint8_t rx_have_reliable;

  wl_event_t event;
  uint8_t has_event;
  uint8_t in_callback;
  uint8_t in_flight_reliable; /* Kind of the queued or in-flight DATA unit. */
  uint8_t control_pending;
  uint8_t control_inflight;
  uint8_t tx_queued;
  /* Cached reliable image in tx_unit. Telemetry may invalidate this image,
   * but tx_payload then retains the original request for a later retry. */
  size_t tx_encoded_len;
  size_t tx_unreliable_len;
  size_t control_len;
  uint8_t rx_event_leased;
  uint8_t rx_candidate_source;
  uint32_t rx_event_generation;
  size_t rx_pending_consume;
  wl_rx_counters_t rx_counters;

  wl_time_ms_t now_ms;

  wl_span_t rx_payload;
  wl_span_t tx_payload;
} wl_ctx_impl_t;

_Static_assert(sizeof(wl_ctx_impl_t) <= WL_CONTEXT_STORAGE_SIZE,
               "WL_CONTEXT_STORAGE_SIZE is too small for wl_ctx_impl_t");
_Static_assert(_Alignof(wl_ctx_impl_t) <= _Alignof(wl_ctx_t),
               "wl_ctx_t does not satisfy implementation alignment");

static inline wl_ctx_impl_t *wl_ctx_impl(wl_ctx_t *ctx) {
  return (wl_ctx_impl_t *)(void *)ctx;
}

static inline const wl_ctx_impl_t *wl_ctx_impl_const(const wl_ctx_t *ctx) {
  return (const wl_ctx_impl_t *)(const void *)ctx;
}

static inline int wl_tx_retry_due(const wl_ctx_impl_t *impl, wl_time_ms_t now) {
  return impl->tx_state == WL_TX_STATE_WAITING_ACK &&
         impl->tx_wait_state == WL_TX_WAIT_ACK &&
         impl->config.ack_timeout_ms != 0U &&
         (wl_time_ms_t)(now - impl->tx_start_ts) >= impl->config.ack_timeout_ms;
}

#endif /* WIRELINK_SRC_CONTEXT_H_ */
