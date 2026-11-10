/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_RPC_ASYNC_H
#define WIRELINK_RPC_ASYNC_H

#include "wirelink/rpc.h"
#include "wirelink/rpc_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Generator/integration machinery, not the ordinary business API. It adds
 * bounded request snapshots and notification ownership to the existing RPC
 * client state machine. No allocator, clock provider, thread or second timer. */
typedef struct wl_rpc_async wl_rpc_async_t;
typedef void (*wl_rpc_callback_t)(void);
typedef wl_err_t (*wl_rpc_async_encode_fn)(const void *request,
    uint32_t operation_id, uint64_t session_id,
    uint8_t *out, size_t capacity, size_t *length);

typedef struct {
  /* prepare copies/decodes a terminal result into generator-owned scratch.
   * It must not call user code. After it returns, RPC/job slots are released;
   * notify then calls the typed business callback using that scratch.
   * callback is converted back to its ORIGINAL function-pointer type before
   * invocation; never convert it through an object pointer. */
  void (*prepare)(void *context, const wl_rpc_client_result_t *result);
  void (*notify)(void *context, wl_rpc_callback_t callback, void *user_data);
  void *context;
  wl_rpc_callback_t callback;
  void *user_data;
} wl_rpc_async_observer_t;

typedef struct {
  struct {
    wl_rpc_client_handle_t handle;
    wl_rpc_async_observer_t observer;
    uint64_t order;
    uint32_t operation_id;
    wl_delivery_t delivery;
    size_t length;
  } private_state;
} wl_rpc_async_slot_t;

/* Optional, copyable cancellation authority. No inspect/release obligation.
 * A close/reinit invalidates every handle through the incarnation check. */
typedef struct {
  struct {
    const wl_rpc_async_t *owner;
    uint64_t incarnation;
    wl_rpc_client_handle_t handle;
  } private_state;
} wl_rpc_call_t;

struct wl_rpc_async {
  struct {
    wl_ctx_t *link;
    wl_rpc_client_t *client;
    wl_rpc_async_slot_t *slots;
    uint8_t *requests;
    size_t request_capacity;
    uint64_t incarnation;
    uint64_t next_order;
    wl_tx_handle_t retiring_tx;
    uint16_t count;
    uint8_t servicing;
    uint8_t submitting;
    uint8_t closing;
  } private_state;
};

/* Zero-initialize async once; close before reinitializing it. All objects/storage
 * stay at stable, disjoint addresses until orderly close.
 * The generated endpoint supplies a nonzero, fresh incarnation on each init. */
wl_err_t wl_rpc_async_init(wl_rpc_async_t *async, wl_ctx_t *link,
    wl_rpc_client_t *client, wl_rpc_async_slot_t *slots, uint16_t count,
    uint8_t *requests, size_t storage_size, size_t request_capacity,
    uint64_t incarnation);

/* Success snapshots request before return and starts the existing RPC deadline
 * at now_ms, INCLUDING queue time (1 <= timeout_ms < 2^31). Failure never calls observer. out_call is
 * optional and remains unchanged on failure. Encoding is synchronous; only the
 * observer's callback/context/user_data live until completion or close.
 * Link backpressure queues an accepted call; it is not an admission failure. */
wl_err_t wl_rpc_async_submit(wl_rpc_async_t *async, uint16_t request_id,
    uint16_t response_id, wl_delivery_t delivery, uint32_t timeout_ms,
    wl_time_ms_t now_ms, wl_rpc_async_encode_fn encode, const void *request,
    const wl_rpc_async_observer_t *observer, wl_rpc_call_t *out_call);

wl_err_t wl_rpc_async_cancel(wl_rpc_async_t *async, const wl_rpc_call_t *call);

/* Owner-side synchronous-wait cleanup. Cancel and notify ONLY this call before
 * returning, so its stack callback context cannot escape. Does not step, read
 * time, submit another call or close unrelated calls. Same reentrancy contract
 * as service; independent transport ownership is retained until owner drain. */
wl_err_t wl_rpc_async_cancel_complete(wl_rpc_async_t *async, const wl_rpc_call_t *call);

/* Call after dispatching events and wl_rpc_client_poll with this owner's time
 * sample. At most count notifications and one extra queue submission per pass.
 * Callbacks may submit/cancel, but cannot recursively service/close. The owner
 * MUST continue draining independent link terminal events after RPC release.
 * Cancelled TX has no event; service collects it after its I/O lease drains. */
wl_err_t wl_rpc_async_service(wl_rpc_async_t *async, wl_time_ms_t now_ms,
    uint16_t *out_notified);

/* Merge with RPC/link/adapter deadlines: terminal notifications are immediate;
 * queued link-blocked requests otherwise use existing readiness/deadlines. */
uint32_t wl_rpc_async_notification_deadline(const wl_rpc_async_t *async);

/* Recognize a terminal transaction detached from an already-notified RPC.
 * Returns one exactly once for that handle. The OWNER still calls wl_tx_take;
 * notification/release never reuses link storage or adapter leases early. */
uint8_t wl_rpc_async_retire_tx(wl_rpc_async_t *async, wl_tx_handle_t handle);

/* At an owner safe point: stop admission, cancel unfinished calls, deliver all
 * notifications exactly once. Quiesce the adapter before destroying its/owner
 * storage. Repeated close is harmless; callbacks cannot reopen this object. */
wl_err_t wl_rpc_async_close(wl_rpc_async_t *async);

/* Shared terminal mapping used by typed prepare callbacks before value decode. */
void wl_rpc_async_completion(const wl_rpc_client_result_t *client,
    wl_rpc_completion_t *out);

#ifdef __cplusplus
}
#endif
#endif
