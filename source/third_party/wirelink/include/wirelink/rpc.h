/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_RPC_H_
#define INCLUDE_WIRELINK_RPC_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/link.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RPC errors are deliberately disjoint from the wl_err_t link error domain. */
typedef int32_t wl_rpc_err_t;
enum {
  WL_RPC_OK = 0,
  WL_RPC_ERR_INVALID_ARG = -1000,
  WL_RPC_ERR_NOT_INITIALIZED = -1001,
  WL_RPC_ERR_NO_SLOT = -1002,
  WL_RPC_ERR_OPERATION_CONFLICT = -1003,
  WL_RPC_ERR_NOT_FOUND = -1004,
  WL_RPC_ERR_INVALID_STATE = -1005,
  WL_RPC_ERR_RESPONSE_MISMATCH = -1006,
  WL_RPC_ERR_RESPONSE_TOO_LARGE = -1007,
  WL_RPC_ERR_CACHE_FULL = -1008,
  WL_RPC_ERR_MALFORMED_METADATA = -1009,
  WL_RPC_ERR_SESSION_MISMATCH = -1010,
  WL_RPC_ERR_ID_EXHAUSTED = -1011,
};

const char *wl_rpc_err_str(wl_rpc_err_t error);

typedef int32_t wl_rpc_client_state_t;
enum {
  WL_RPC_CLIENT_FREE = 0,
  WL_RPC_CLIENT_QUEUED,
  WL_RPC_CLIENT_LINK_PENDING,
  WL_RPC_CLIENT_WAIT_RESPONSE,
  WL_RPC_CLIENT_COMPLETED,
  WL_RPC_CLIENT_LINK_FAILED,
  WL_RPC_CLIENT_TIMED_OUT,
  WL_RPC_CLIENT_CANCELLED,
  WL_RPC_CLIENT_APPLICATION_ERROR,
};

typedef int32_t wl_rpc_cache_policy_t;
enum {
  /* A full cache rejects new execution at begin(), before the handler runs.
     Pending executions already own their response capacity. */
  WL_RPC_CACHE_REJECT_NEW = 0,
  /* Evict the oldest DELIVERED response only. Reserved, ready, acquired and
     in-flight entries remain protected; replay protection ends after eviction.
     TTL is a maximum age, not a guaranteed retention window. */
  WL_RPC_CACHE_EVICT_OLDEST = 1,
};

typedef int32_t wl_rpc_server_disposition_t;
enum {
  WL_RPC_SERVER_NEW = 0,
  WL_RPC_SERVER_PENDING_DUPLICATE,
  WL_RPC_SERVER_REPLAY,
  WL_RPC_SERVER_CONFLICT,
};

/*
 * The private representations are fixed-size ABI storage. Applications allocate
 * arrays of slots but must not inspect or copy initialized objects.
 */
#define WL_RPC_CLIENT_STORAGE_SIZE 64U
#define WL_RPC_CLIENT_SLOT_STORAGE_SIZE 64U
#define WL_RPC_SERVER_STORAGE_SIZE 96U
#define WL_RPC_SERVER_PENDING_SLOT_STORAGE_SIZE 48U
#define WL_RPC_SERVER_CACHE_SLOT_STORAGE_SIZE 96U

typedef union wl_rpc_client {
  wl_max_align_t align;
  uint8_t private_bytes[WL_RPC_CLIENT_STORAGE_SIZE];
} wl_rpc_client_t;

typedef union wl_rpc_client_slot {
  wl_max_align_t align;
  uint8_t private_bytes[WL_RPC_CLIENT_SLOT_STORAGE_SIZE];
} wl_rpc_client_slot_t;

/* Copyable authority for one client slot generation, not a wire call ID.
 * Contents are private. Handles belong to one initialized client at a stable
 * address; reinitializing/destroying that client ends every handle's lifetime.
 * Generated endpoints additionally guard their own close/reinit lifetime. */
#define WL_RPC_CLIENT_HANDLE_STORAGE_SIZE 32U
typedef union wl_rpc_client_handle {
  wl_max_align_t align;
  uint8_t private_bytes[WL_RPC_CLIENT_HANDLE_STORAGE_SIZE];
} wl_rpc_client_handle_t;

typedef union wl_rpc_server {
  wl_max_align_t align;
  uint8_t private_bytes[WL_RPC_SERVER_STORAGE_SIZE];
} wl_rpc_server_t;

typedef union wl_rpc_server_pending_slot {
  wl_max_align_t align;
  uint8_t private_bytes[WL_RPC_SERVER_PENDING_SLOT_STORAGE_SIZE];
} wl_rpc_server_pending_slot_t;

typedef union wl_rpc_server_cache_slot {
  wl_max_align_t align;
  uint8_t private_bytes[WL_RPC_SERVER_CACHE_SLOT_STORAGE_SIZE];
} wl_rpc_server_cache_slot_t;

typedef struct wl_rpc_client_config {
  wl_rpc_client_slot_t *slots;
  uint16_t slot_count;
  uint8_t *response_storage;
  size_t response_storage_size;
  uint16_t response_capacity_per_slot;
  /* Zero selects operation ID 1. Allocation always skips reserved ID zero. */
  uint32_t next_operation_id;
} wl_rpc_client_config_t;

typedef struct wl_rpc_client_result {
  uint32_t operation_id;
  uint16_t request_message_id;
  uint16_t response_message_id;
  wl_rpc_client_state_t state;
  wl_tx_handle_t tx_handle;
  /* A wl_err_t or adapter result; separate from runtime_error. */
  int32_t link_result;
  /* Zero is application success; other values belong to the service schema. */
  int32_t application_status;
  wl_rpc_err_t runtime_error;
  /* One only after a reliable WL_EVT_TX_SUCCESS; local completion leaves zero.
   */
  uint8_t link_delivery_confirmed;
  const uint8_t *response_data;
  size_t response_length;
} wl_rpc_client_result_t;

#define WL_RPC_NO_DEADLINE_MS UINT32_MAX
typedef struct wl_rpc_deadline_hint {
  /* Relative to the now_ms passed to get_deadline_hint(). */
  uint32_t next_deadline_ms;
} wl_rpc_deadline_hint_t;

wl_rpc_err_t wl_rpc_client_init(wl_rpc_client_t *client,
                                const wl_rpc_client_config_t *config);

/* Allocate a nonzero operation ID that does not collide with a retained slot.
 */
wl_rpc_err_t wl_rpc_client_begin(wl_rpc_client_t *client,
                                 uint16_t request_message_id,
                                 uint16_t response_message_id,
                                 uint32_t timeout_ms, wl_time_ms_t now_ms,
                                 uint32_t *out_operation_id);

/* Use a product-provided operation ID. It must remain unique until release. */
wl_rpc_err_t wl_rpc_client_begin_with_id(
    wl_rpc_client_t *client, uint32_t operation_id, uint16_t request_message_id,
    uint16_t response_message_id, uint32_t timeout_ms, wl_time_ms_t now_ms);

/* Bind the handle returned by wl_send_reliable() after request
 * encoding/sending. */
wl_rpc_err_t wl_rpc_client_bind_tx(wl_rpc_client_t *client,
                                   uint32_t operation_id,
                                   wl_tx_handle_t tx_handle);

/*
 * Enter WAIT_RESPONSE after an uncorrelated/local TX completion. This is the
 * normal path for unreliable requests and does not confirm peer delivery.
 */
wl_rpc_err_t wl_rpc_client_tx_completed(wl_rpc_client_t *client,
                                        uint32_t operation_id);

/* Report a send failure that occurred before a Wirelink handle was bound. */
wl_rpc_err_t wl_rpc_client_link_failed(wl_rpc_client_t *client,
                                       uint32_t operation_id,
                                       int32_t link_result);

/*
 * Consume only TX terminal events. LINK success advances to WAIT_RESPONSE; it
 * never creates application success. Returns NOT_FOUND for an unrelated handle.
 */
wl_rpc_err_t wl_rpc_client_on_tx_event(wl_rpc_client_t *client,
                                       const wl_event_t *event);

/*
 * Correlate a decoded application response. A matching response may complete
 * LINK_PENDING before its independent core TX transaction terminates. The
 * payload is moved into caller-supplied storage and retained until release.
 */
wl_rpc_err_t wl_rpc_client_on_response(wl_rpc_client_t *client,
                                       uint16_t response_message_id,
                                       uint32_t operation_id,
                                       int32_t application_status,
                                       const uint8_t *response_payload,
                                       size_t response_length);

/* Advance all end-to-end deadlines. Returns the number newly timed out. */
wl_rpc_err_t wl_rpc_client_poll(wl_rpc_client_t *client, wl_time_ms_t now_ms,
                                uint16_t *out_timed_out);

/* Side-effect free: zero means due, WL_RPC_NO_DEADLINE_MS means none. */
wl_rpc_err_t wl_rpc_client_get_deadline_hint(const wl_rpc_client_t *client,
                                             wl_time_ms_t now_ms,
                                             wl_rpc_deadline_hint_t *out_hint);

/* Best effort: the returned result exposes a bound handle for wl_tx_cancel().
 */
wl_rpc_err_t wl_rpc_client_cancel(wl_rpc_client_t *client,
                                  uint32_t operation_id);

/* The returned response pointer remains valid until this operation is released.
 */
wl_rpc_err_t wl_rpc_client_get(const wl_rpc_client_t *client,
                               uint32_t operation_id,
                               wl_rpc_client_result_t *out_result);

/* Only terminal operations may be released and their ID reused locally. */
wl_rpc_err_t wl_rpc_client_release(wl_rpc_client_t *client,
                                   uint32_t operation_id);

/* Capturing a handle scans retained IDs. Outputs are unchanged on error.
 * Subsequent handle lookups are constant-time and reject cross-client/stale slots.
 * Like the ID-based API, all accesses belong to the single communication owner.
 */
wl_rpc_err_t wl_rpc_client_get_handle(const wl_rpc_client_t *client,
                                      uint32_t operation_id,
                                      wl_rpc_client_handle_t *out_handle);
wl_rpc_err_t wl_rpc_client_get_by_handle(
    const wl_rpc_client_t *client, const wl_rpc_client_handle_t *handle,
    wl_rpc_client_result_t *out_result);
wl_rpc_err_t wl_rpc_client_cancel_handle(
    wl_rpc_client_t *client, const wl_rpc_client_handle_t *handle);
wl_rpc_err_t wl_rpc_client_release_handle(
    wl_rpc_client_t *client, const wl_rpc_client_handle_t *handle);

typedef struct wl_rpc_request_identity {
  uint32_t operation_id;
  uint16_t request_message_id;
  uint16_t response_message_id;
  /*
   * A stable application-supplied fingerprint of the canonical request. The
   * server treats identity fields as exact; the caller owns collision quality.
   */
  uint64_t request_fingerprint;
  /* Wirelink sender session. Zero denotes an unscoped/unreliable request. */
  uint64_t peer_session_id;
} wl_rpc_request_identity_t;

/* Reliable response terminal observation, not proof of application execution
 * at the peer. SUCCESS means link ACK; TIMEOUT/FAILED must not trigger a
 * success-only action such as reboot. Identity and event are borrowed only
 * during this callback. Record intent and act after the owner pass: do not
 * take the TX event, reenter the server, step, close, or reboot here.
 * Retransmitted cached responses may produce additional notifications. */
typedef void (*wl_rpc_response_observer_fn)(void *user_data,
    const wl_rpc_request_identity_t *identity, int32_t application_status,
    const wl_event_t *event);

/*
 * Generation-stamped authority for one accepted server execution. Copy this
 * value when work completes asynchronously. Session discard, abandon, or
 * expiry followed by slot reuse invalidates the old generation, so a late
 * completion can never target a newer request with the same wire identity.
 */
typedef struct wl_rpc_server_request {
  wl_rpc_request_identity_t identity;
  uint64_t generation;
} wl_rpc_server_request_t;

typedef struct wl_rpc_server_config {
  wl_rpc_server_pending_slot_t *pending_slots;
  uint16_t pending_slot_count;
  wl_rpc_server_cache_slot_t *cache_slots;
  uint16_t cache_slot_count;
  uint8_t *response_storage;
  size_t response_storage_size;
  uint16_t response_capacity_per_slot;
  /* Zero disables timed expiry. Nonzero values must be below 2^31 ms. */
  uint32_t pending_timeout_ms;
  uint32_t cache_ttl_ms;
  wl_rpc_cache_policy_t cache_policy;
} wl_rpc_server_config_t;

typedef struct wl_rpc_server_response {
  wl_rpc_request_identity_t identity;
  int32_t application_status;
  const uint8_t *response_data;
  size_t response_length;
  /* Opaque cache generation used to validate lifecycle transitions. */
  uint64_t generation;
} wl_rpc_server_response_t;

/*
 * A generation-scoped writable view of the cache storage reserved by begin().
 * It is borrowed only while request remains pending and must not be retained or
 * written after completion, rejection, abandon, expiry, or session discard.
 */
typedef struct wl_rpc_server_response_buffer {
  wl_rpc_server_request_t request;
  uint8_t *data;
  size_t capacity;
} wl_rpc_server_response_buffer_t;

typedef struct wl_rpc_server_expiry {
  uint16_t pending_expired;
  /* Entries reclaimed by this poll, excluding those already reclaimed by begin. */
  uint16_t cache_expired;
} wl_rpc_server_expiry_t;

typedef void (*wl_rpc_server_cancel_tx_fn_t)(void *context,
                                             wl_tx_handle_t tx_handle);

typedef struct wl_rpc_server_discard_result {
  uint16_t pending_discarded;
  uint16_t responses_discarded;
  uint16_t tx_cancel_requested;
} wl_rpc_server_discard_result_t;

/* Optional point-to-point peer-session tracker. Zero-initialize before use. */
typedef struct wl_rpc_peer {
  uint64_t session_id;
} wl_rpc_peer_t;

typedef struct wl_rpc_peer_observation {
  uint64_t previous_session_id;
  uint64_t current_session_id;
  wl_rpc_server_discard_result_t discarded;
  /* One for an initial binding or a transition; zero for the same session. */
  uint8_t changed;
} wl_rpc_peer_observation_t;

wl_rpc_err_t wl_rpc_server_init(wl_rpc_server_t *server,
                                const wl_rpc_server_config_t *config);

/*
 * NEW reserves a pending slot. PENDING_DUPLICATE suppresses re-execution.
 * REPLAY returns cached response bytes. CONFLICT means the operation ID is in
 * use by a different exact identity within the same peer session. The same
 * operation ID from another peer session is independent. All four are
 * successful classifications. NEW also reserves response-cache storage before
 * the application handler may execute; completion therefore cannot fail with
 * CACHE_FULL. out_request is nonzero only for NEW.
 * Expired delivered responses are reclaimed during lookup; a preceding poll
 * is not required. Reserved, ready, acquired, and in-flight responses remain
 * protected. TTL zero disables expiry, including expiry during begin.
 */
wl_rpc_err_t wl_rpc_server_begin(wl_rpc_server_t *server,
                                 const wl_rpc_request_identity_t *identity,
                                 wl_time_ms_t now_ms,
                                 wl_rpc_server_disposition_t *out_disposition,
                                 wl_rpc_server_request_t *out_request,
                                 wl_rpc_server_response_t *out_replay);

/*
 * Borrow the exact cache segment already reserved for request. Encode directly
 * into data, then publish the encoded prefix with response_commit(). A codec
 * failure needs no rollback: request remains pending and may be retried or
 * abandoned. The returned generation token prevents stale commits after slot
 * reuse; callers must also obey the borrowed-pointer lifetime above.
 */
wl_rpc_err_t wl_rpc_server_response_prepare(
    wl_rpc_server_t *server, const wl_rpc_server_request_t *request,
    wl_rpc_server_response_buffer_t *out_buffer);

/* Publish bytes encoded in a matching prepared buffer without copying them. */
wl_rpc_err_t wl_rpc_server_response_commit(
    wl_rpc_server_t *server, const wl_rpc_server_response_buffer_t *buffer,
    int32_t application_status, size_t response_length, wl_time_ms_t now_ms,
    wl_rpc_server_response_t *out_response);

/* Complete an asynchronous or synchronous operation and cache/move its
 * response. identity must exactly match the token accepted by begin(). This is
 * the copying convenience API; generated codecs should use prepare/commit. */
wl_rpc_err_t wl_rpc_server_complete(wl_rpc_server_t *server,
                                    const wl_rpc_server_request_t *request,
                                    int32_t application_status,
                                    const uint8_t *response_payload,
                                    size_t response_length, wl_time_ms_t now_ms,
                                    wl_rpc_server_response_t *out_response);

/* Application rejection is a completed, replayable nonzero status. */
wl_rpc_err_t wl_rpc_server_reject(wl_rpc_server_t *server,
                                  const wl_rpc_server_request_t *request,
                                  int32_t application_status,
                                  const uint8_t *response_payload,
                                  size_t response_length, wl_time_ms_t now_ms,
                                  wl_rpc_server_response_t *out_response);

/*
 * Acquire the oldest response waiting for transport acceptance. The returned
 * bytes remain stable until exactly one matching defer/submitted/sent call.
 * NOT_FOUND means that no response currently needs service.
 */
wl_rpc_err_t wl_rpc_server_response_acquire(
    wl_rpc_server_t *server, wl_rpc_server_response_t *out_response);

/* Return an acquired response to the ready queue after link backpressure. */
wl_rpc_err_t wl_rpc_server_response_defer(
    wl_rpc_server_t *server, const wl_rpc_server_response_t *response);

/* Bind an acquired reliable response to its nonzero Wirelink TX handle. */
wl_rpc_err_t wl_rpc_server_response_submitted(
    wl_rpc_server_t *server, const wl_rpc_server_response_t *response,
    wl_tx_handle_t tx_handle);

/* Finish an acquired unreliable response, which has no terminal TX event. */
wl_rpc_err_t wl_rpc_server_response_sent(
    wl_rpc_server_t *server, const wl_rpc_server_response_t *response);

/*
 * Consume a matching reliable terminal event. Any terminal result leaves a
 * replayable delivered cache entry; a duplicate request schedules it again.
 * Link-level retry remains bounded by the protocol's reliable TX policy.
 */
wl_rpc_err_t wl_rpc_server_on_tx_event(wl_rpc_server_t *server,
                                       const wl_event_t *event);
/* Owner-side setup. NULL disables observation. No allocation or ownership
 * transfer; callback context must outlive the initialized server. */
wl_rpc_err_t wl_rpc_server_set_response_observer(wl_rpc_server_t *server,
    wl_rpc_response_observer_fn observer, void *user_data);

/*
 * Forget every pending/cached operation owned by one nonzero peer session.
 * The optional callback receives each still in-flight reliable handle after
 * its response has been detached, so a product can call wl_tx_cancel().
 */
wl_rpc_err_t wl_rpc_server_discard_session(
    wl_rpc_server_t *server, uint64_t peer_session_id,
    wl_rpc_server_cancel_tx_fn_t cancel_tx, void *cancel_context,
    wl_rpc_server_discard_result_t *out_result);

/*
 * Observe a nonzero session for a point-to-point peer. On transition, discard
 * all RPC state for the previous session before publishing the new one. The
 * caller remains responsible for product state such as leases and non-RPC TX.
 */
wl_rpc_err_t wl_rpc_peer_observe(
    wl_rpc_server_t *server, wl_rpc_peer_t *peer, uint64_t peer_session_id,
    wl_rpc_server_cancel_tx_fn_t cancel_tx, void *cancel_context,
    wl_rpc_peer_observation_t *out_observation);

/* Drop exact pending metadata without manufacturing or caching a response. */
wl_rpc_err_t wl_rpc_server_abandon(wl_rpc_server_t *server,
                                   const wl_rpc_server_request_t *request);

/*
 * Return one newly expired pending identity without discarding it. The caller
 * must complete/reject it with a typed terminal response, or abandon it. A
 * reported identity is not returned again and no longer contributes a
 * deadline. NOT_FOUND means no unreported expiry is due.
 */
wl_rpc_err_t wl_rpc_server_expired_acquire(
    wl_rpc_server_t *server, wl_time_ms_t now_ms,
    wl_rpc_server_request_t *out_request);

/* Expire delivered replay entries. Pending identities use expired_acquire(). */
wl_rpc_err_t wl_rpc_server_poll(wl_rpc_server_t *server, wl_time_ms_t now_ms,
                                wl_rpc_server_expiry_t *out_expiry);

/* Includes both pending timeouts and cache TTLs without expiring either. */
wl_rpc_err_t wl_rpc_server_get_deadline_hint(const wl_rpc_server_t *server,
                                             wl_time_ms_t now_ms,
                                             wl_rpc_deadline_hint_t *out_hint);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_RPC_H_ */
