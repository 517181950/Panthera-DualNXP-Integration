/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_LINK_H_
#define INCLUDE_WIRELINK_LINK_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/alignment.h"
#include "wirelink/clock.h"
#include "wirelink/profile.h"
#include "wirelink/span.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t wl_tx_handle_t;
typedef uint32_t wl_io_token_t;

#define WL_CONTEXT_STORAGE_SIZE 896U
typedef union wl_ctx {
  wl_max_align_t align;
  uint8_t private_bytes[WL_CONTEXT_STORAGE_SIZE];
} wl_ctx_t;

typedef int32_t wl_tx_state_t;
enum {
  WL_TX_STATE_IDLE = 0,
  WL_TX_STATE_SENDING,
  WL_TX_STATE_WAITING_ACK,
  WL_TX_STATE_SUCCESS,
  WL_TX_STATE_FAILED,
  WL_TX_STATE_CANCELLED,
};

typedef int32_t wl_tx_wait_reason_t;
enum { WL_TX_WAIT_NONE = 0, WL_TX_WAIT_ACK };

typedef int32_t wl_event_type_t;
enum {
  WL_EVT_NONE = 0,
  WL_EVT_UNRELIABLE_RX,
  WL_EVT_RELIABLE_RX,
  WL_EVT_TX_SUCCESS,
  WL_EVT_TX_TIMEOUT,
  WL_EVT_TX_FAILED,
};

typedef struct {
  wl_event_type_t type;
  uint16_t message_id;
  const uint8_t *payload;
  size_t payload_len;
  wl_tx_handle_t handle;
  int io_result;
  uint32_t lease;
  uint64_t peer_session_id;
} wl_event_t;

typedef struct {
  wl_tx_state_t state;
  int result;
  uint16_t retries_used;
} wl_tx_result_t;

#define WL_POLL_NO_DEADLINE_MS UINT32_MAX
typedef struct {
  uint32_t work_pending;
  uint32_t next_deadline_ms;
} wl_poll_hint_t;

typedef struct {
  uint16_t max_payload_len;
  wl_envelope_type_t envelope;
  wl_integrity_t integrity;
  uint64_t session_id;
  uint16_t max_retries;
  uint32_t ack_timeout_ms;
  size_t max_transmission_unit;
} wl_config_t;

/* Storage remains reserved for the link until it is quiesced. tx_payload,
 * tx_unit and control_unit must be pairwise disjoint (checked by wl_init):
 * the reliable payload can outlive an intervening unreliable DATA image,
 * and an ACK can be prepared while a DATA unit is owned by the sink.
 * Sinks read units; they must not modify them, even after completing an I/O. */
typedef struct {
  uint8_t *tx_payload;
  size_t tx_payload_size;
  uint8_t *tx_unit;
  size_t tx_unit_size;
  uint8_t *control_unit;
  size_t control_unit_size;
  uint8_t *rx_fifo;
  size_t rx_fifo_size;
  uint8_t *rx_fallback;
  size_t rx_fallback_size;
} wl_storage_t;

typedef struct {
  size_t tx_payload_size;
  size_t tx_unit_size;
  size_t control_unit_size;
  size_t rx_fifo_size;
  size_t rx_fallback_size;
} wl_storage_requirements_t;

typedef struct {
  uint32_t malformed;
  uint32_t bad_integrity;
  uint32_t overflow;
  uint32_t duplicate;
  uint32_t unsupported;
} wl_rx_counters_t;

typedef int32_t wl_delivery_t;
enum { WL_DELIVERY_UNRELIABLE = 0, WL_DELIVERY_RELIABLE = 1 };

/* Writable payload storage owned by the link until commit or abort. A commit
 * attempt consumes a matching active claim even when it returns an error. */
typedef struct wl_tx_payload_claim {
  wl_span_t span;
  uint32_t token;
} wl_tx_payload_claim_t;

wl_err_t wl_config_requirements(const wl_config_t *config,
                                wl_storage_requirements_t *out_requirements);
wl_err_t wl_init(wl_ctx_t *ctx, const wl_config_t *config,
                 const wl_storage_t *storage);
wl_err_t wl_get_config(const wl_ctx_t *ctx, wl_config_t *out_config);
/* Read-only initialized sender identity for generated/integration code.
 * NULL returns zero. This is not a business operation identifier. */
uint64_t wl_link_session_id(const wl_ctx_t *ctx);
wl_err_t wl_rx_get_counters(const wl_ctx_t *ctx,
                            wl_rx_counters_t *out_counters);

wl_err_t wl_send_unreliable(wl_ctx_t *ctx, uint16_t message_id,
                            const uint8_t *payload, size_t payload_len);
/* One reliable transaction remains reserved until take(). While it waits for
 * ACK, unreliable DATA may use the idle physical unit. ACKs and due retries
 * take priority; queued/in-flight I/O still backpressures every DATA sender.
 * Continue polling on the same clock, including during sustained telemetry. */
wl_err_t wl_send_reliable(wl_ctx_t *ctx, uint16_t message_id,
                          const uint8_t *payload, size_t payload_len,
                          wl_time_ms_t now_ms,
                          wl_tx_handle_t *out_handle);
wl_err_t wl_tx_payload_claim(wl_ctx_t *ctx, uint16_t message_id,
                             wl_delivery_t delivery,
                             wl_tx_payload_claim_t *out_claim);
wl_err_t wl_tx_payload_commit(wl_ctx_t *ctx,
                              const wl_tx_payload_claim_t *claim,
                              size_t payload_len,
                              wl_time_ms_t now_ms,
                              wl_tx_handle_t *out_handle);
wl_err_t wl_tx_payload_abort(wl_ctx_t *ctx,
                             const wl_tx_payload_claim_t *claim);

wl_err_t wl_tx_status(const wl_ctx_t *ctx, wl_tx_handle_t handle,
                      wl_tx_state_t *out_state);
wl_err_t wl_tx_take(wl_ctx_t *ctx, wl_tx_handle_t handle,
                    wl_tx_result_t *out_result);
wl_err_t wl_tx_cancel(wl_ctx_t *ctx, wl_tx_handle_t handle);

wl_err_t wl_poll(wl_ctx_t *ctx, wl_time_ms_t now_ms, wl_event_t *out_event);
wl_err_t wl_poll_get_hint(const wl_ctx_t *ctx, wl_time_ms_t now_ms,
                          wl_poll_hint_t *out_hint);
void wl_event_release(wl_ctx_t *ctx, const wl_event_t *event);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_LINK_H_ */
