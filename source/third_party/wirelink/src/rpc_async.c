/* SPDX-License-Identifier: Apache-2.0 */
#include "wirelink/rpc_async.h"
#include <string.h>

const char *wl_rpc_status_str(wl_rpc_status_t status) {
  switch (status) {
    case WL_RPC_SUCCESS: return "success";
    case WL_RPC_REJECTED: return "application rejected";
    case WL_RPC_TIMED_OUT: return "RPC timed out";
    case WL_RPC_CANCELLED: return "cancelled locally";
    case WL_RPC_FAILED: return "communication failed";
    default: return "unknown RPC outcome";
  }
}

void wl_rpc_async_completion(const wl_rpc_client_result_t *client,
    wl_rpc_completion_t *out) {
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  if (client == NULL) {
    out->status = WL_RPC_FAILED;
    out->runtime_error = WL_RPC_ERR_INVALID_ARG;
    return;
  }
  out->transport_error = client->link_result;
  out->runtime_error = client->runtime_error;
  switch (client->state) {
    case WL_RPC_CLIENT_COMPLETED: out->status = WL_RPC_SUCCESS; break;
    case WL_RPC_CLIENT_APPLICATION_ERROR:
      /* The advanced engine also uses this state for oversized responses.
       * Do not expose a local framework failure as a remote business reject. */
      out->status = client->runtime_error == WL_RPC_OK ? WL_RPC_REJECTED : WL_RPC_FAILED;
      if (out->status == WL_RPC_REJECTED) out->rejection = client->application_status;
      break;
    case WL_RPC_CLIENT_TIMED_OUT: out->status = WL_RPC_TIMED_OUT; break;
    case WL_RPC_CLIENT_CANCELLED: out->status = WL_RPC_CANCELLED; break;
    case WL_RPC_CLIENT_LINK_FAILED: out->status = WL_RPC_FAILED; break;
    default:
      out->status = WL_RPC_FAILED;
      out->runtime_error = WL_RPC_ERR_INVALID_STATE;
      break;
  }
}

static int terminal(wl_rpc_client_state_t state) {
  return state == WL_RPC_CLIENT_COMPLETED || state == WL_RPC_CLIENT_LINK_FAILED ||
      state == WL_RPC_CLIENT_TIMED_OUT || state == WL_RPC_CLIENT_CANCELLED ||
      state == WL_RPC_CLIENT_APPLICATION_ERROR;
}

static wl_err_t map_error(wl_rpc_err_t result) {
  switch (result) {
    case WL_RPC_OK: return WL_OK;
    case WL_RPC_ERR_NO_SLOT: return WL_ERR_BUSY;
    case WL_RPC_ERR_INVALID_ARG: return WL_ERR_INVALID_ARG;
    case WL_RPC_ERR_NOT_INITIALIZED: return WL_ERR_NOT_INITIALIZED;
    case WL_RPC_ERR_NOT_FOUND: return WL_ERR_NOT_FOUND;
    case WL_RPC_ERR_ID_EXHAUSTED: return WL_ERR_ID_EXHAUSTED;
    default: return WL_ERR_INVALID_STATE;
  }
}

wl_err_t wl_rpc_async_init(wl_rpc_async_t *async, wl_ctx_t *link,
    wl_rpc_client_t *client, wl_rpc_async_slot_t *slots, uint16_t count,
    uint8_t *requests, size_t storage_size, size_t request_capacity,
    uint64_t incarnation) {
  if (async == NULL || link == NULL || client == NULL || slots == NULL ||
      count == 0U || requests == NULL || request_capacity == 0U ||
      incarnation == 0U) return WL_ERR_INVALID_ARG;
  if (async->private_state.client != NULL) return WL_ERR_INVALID_STATE;
  if (request_capacity > storage_size / count) return WL_ERR_BUF_TOO_SMALL;
  memset(async, 0, sizeof(*async));
  memset(slots, 0, (size_t)count * sizeof(*slots));
  async->private_state.link = link;
  async->private_state.client = client;
  async->private_state.slots = slots;
  async->private_state.requests = requests;
  async->private_state.count = count;
  async->private_state.request_capacity = request_capacity;
  async->private_state.incarnation = incarnation;
  async->private_state.next_order = 1U;
  return WL_OK;
}

static wl_err_t submit_oldest(wl_rpc_async_t *async, wl_time_ms_t now_ms) {
  wl_rpc_async_slot_t *oldest = NULL;
  wl_rpc_client_result_t result, candidate;
  size_t index = 0U;
  if (async->private_state.closing) return WL_OK;
  for (uint16_t i = 0U; i < async->private_state.count; ++i) {
    wl_rpc_async_slot_t *slot = &async->private_state.slots[i];
    if (slot->private_state.operation_id == 0U) continue;
    wl_rpc_err_t error = wl_rpc_client_get_by_handle(async->private_state.client,
        &slot->private_state.handle, &candidate);
    if (error != WL_RPC_OK) return map_error(error);
    if (candidate.state != WL_RPC_CLIENT_QUEUED) continue;
    if (oldest == NULL || slot->private_state.order - oldest->private_state.order > UINT64_MAX / 2U) {
      oldest = slot;
      index = i;
      result = candidate;
    }
  }
  if (oldest != NULL) {
    wl_tx_handle_t tx = 0U;
    const uint8_t *payload = async->private_state.requests +
        index * async->private_state.request_capacity;
    wl_err_t error = oldest->private_state.delivery == WL_DELIVERY_RELIABLE
        ? wl_send_reliable(async->private_state.link, result.request_message_id,
            payload, oldest->private_state.length, now_ms, &tx)
        : wl_send_unreliable(async->private_state.link, result.request_message_id,
            payload, oldest->private_state.length);
    if (error == WL_ERR_BUSY || error == WL_ERR_WOULD_BLOCK ||
        error == WL_ERR_QUEUE_FULL || error == WL_ERR_NO_SPACE) return WL_OK;
    if (error != WL_OK)
      return map_error(wl_rpc_client_link_failed(async->private_state.client,
          result.operation_id, error));
    return map_error(oldest->private_state.delivery == WL_DELIVERY_RELIABLE
        ? wl_rpc_client_bind_tx(async->private_state.client, result.operation_id, tx)
        : wl_rpc_client_tx_completed(async->private_state.client, result.operation_id));
  }
  return WL_OK;
}

wl_err_t wl_rpc_async_submit(wl_rpc_async_t *async, uint16_t request_id,
    uint16_t response_id, wl_delivery_t delivery, uint32_t timeout_ms,
    wl_time_ms_t now_ms, wl_rpc_async_encode_fn encode, const void *request,
    const wl_rpc_async_observer_t *observer, wl_rpc_call_t *out_call) {
  wl_rpc_async_slot_t *slot = NULL;
  uint16_t index;
  uint32_t operation_id;
  wl_rpc_err_t rpc;
  wl_err_t error;
  size_t length = 0U;
  if (async == NULL || encode == NULL || request == NULL || observer == NULL || timeout_ms == 0U ||
      observer->prepare == NULL || observer->notify == NULL || observer->callback == NULL ||
      (delivery != WL_DELIVERY_RELIABLE && delivery != WL_DELIVERY_UNRELIABLE))
    return WL_ERR_INVALID_ARG;
  if (async->private_state.client == NULL || async->private_state.closing)
    return WL_ERR_NOT_INITIALIZED;
  if (async->private_state.submitting) return WL_ERR_REENTRANT;
  for (index = 0U; index < async->private_state.count; ++index) {
    if (async->private_state.slots[index].private_state.operation_id == 0U) {
      slot = &async->private_state.slots[index];
      break;
    }
  }
  if (slot == NULL) return WL_ERR_BUSY;
  async->private_state.submitting = 1U;
  rpc = wl_rpc_client_begin(async->private_state.client, request_id,
      response_id, timeout_ms, now_ms, &operation_id);
  if (rpc != WL_RPC_OK) {
    async->private_state.submitting = 0U;
    return map_error(rpc);
  }
  error = encode(request, operation_id, wl_link_session_id(async->private_state.link),
      async->private_state.requests +
      (size_t)index * async->private_state.request_capacity,
      async->private_state.request_capacity, &length);
  if (error == WL_OK && length > async->private_state.request_capacity)
    error = WL_ERR_PAYLOAD_TOO_LONG;
  if (error == WL_OK) {
    rpc = wl_rpc_client_get_handle(async->private_state.client, operation_id,
        &slot->private_state.handle);
    error = map_error(rpc);
  }
  if (error != WL_OK) {
    (void)wl_rpc_client_cancel(async->private_state.client, operation_id);
    (void)wl_rpc_client_release(async->private_state.client, operation_id);
    async->private_state.submitting = 0U;
    return error;
  }
  slot->private_state.operation_id = operation_id;
  slot->private_state.delivery = delivery;
  slot->private_state.length = length;
  slot->private_state.observer = *observer;
  slot->private_state.order = async->private_state.next_order++;
  if (out_call != NULL) {
    out_call->private_state.owner = async;
    out_call->private_state.incarnation = async->private_state.incarnation;
    out_call->private_state.handle = slot->private_state.handle;
  }
  /* From here the call is accepted. A send failure is a terminal RPC result,
   * not a failed admission with an unexpected later callback. */
  if (!async->private_state.servicing) {
    uint16_t expired;
    /* A new submission after idle must not transmit an older queued request
     * whose deadline already passed. Never restart its acceptance timer. */
    (void)wl_rpc_client_poll(async->private_state.client, now_ms, &expired);
  }
  (void)submit_oldest(async, now_ms);
  async->private_state.submitting = 0U;
  return WL_OK;
}

wl_err_t wl_rpc_async_cancel(wl_rpc_async_t *async, const wl_rpc_call_t *call) {
  if (async == NULL || call == NULL) return WL_ERR_INVALID_ARG;
  if (async->private_state.client == NULL || async->private_state.closing)
    return WL_ERR_NOT_INITIALIZED;
  if (call->private_state.owner != async ||
      call->private_state.incarnation != async->private_state.incarnation)
    return WL_ERR_NOT_FOUND;
  return map_error(wl_rpc_client_cancel_handle(async->private_state.client,
      &call->private_state.handle));
}

static void collect_cancelled_tx(wl_rpc_async_t *async) {
  const wl_tx_handle_t handle = async->private_state.retiring_tx;
  wl_tx_state_t state;
  wl_tx_result_t result;
  if (handle == 0U || async->private_state.closing) return;
  /* Explicit link cancellation has no terminal event. Take it ourselves once
   * its physical I/O lease is gone; success/failure still use event dispatch.
   * A pending DMA lease makes take fail, and its completion wakes the owner. */
  if (wl_tx_status(async->private_state.link, handle, &state) == WL_OK &&
      state == WL_TX_STATE_CANCELLED &&
      wl_tx_take(async->private_state.link, handle, &result) == WL_OK)
    async->private_state.retiring_tx = 0U;
}

static wl_err_t notify_terminal(wl_rpc_async_t *async, wl_rpc_async_slot_t *slot,
    const wl_rpc_client_result_t *result) {
  wl_rpc_async_observer_t observer;
  wl_rpc_err_t rpc;
  if (!async->private_state.closing && result->tx_handle != 0U &&
      (result->state == WL_RPC_CLIENT_CANCELLED || result->state == WL_RPC_CLIENT_TIMED_OUT))
    (void)wl_tx_cancel(async->private_state.link, result->tx_handle);
  if (result->tx_handle != 0U) {
    wl_tx_state_t state;
    if (wl_tx_status(async->private_state.link, result->tx_handle, &state) == WL_OK)
      async->private_state.retiring_tx = result->tx_handle;
  }
  collect_cancelled_tx(async);
  observer = slot->private_state.observer;
  observer.prepare(observer.context, result);
  rpc = wl_rpc_client_release_handle(async->private_state.client, &slot->private_state.handle);
  if (rpc != WL_RPC_OK) return map_error(rpc);
  memset(slot, 0, sizeof(*slot));
  observer.notify(observer.context, observer.callback, observer.user_data);
  return WL_OK;
}

wl_err_t wl_rpc_async_cancel_complete(wl_rpc_async_t *async, const wl_rpc_call_t *call) {
  wl_rpc_client_result_t result;
  wl_err_t error;
  wl_rpc_err_t rpc;
  if (async == NULL || call == NULL) return WL_ERR_INVALID_ARG;
  if (async->private_state.servicing || async->private_state.submitting)
    return WL_ERR_REENTRANT;
  if (async->private_state.client == NULL || async->private_state.closing)
    return WL_ERR_NOT_INITIALIZED;
  if (call->private_state.owner != async ||
      call->private_state.incarnation != async->private_state.incarnation)
    return WL_ERR_NOT_FOUND;
  rpc = wl_rpc_client_get_by_handle(async->private_state.client, &call->private_state.handle, &result);
  if (rpc != WL_RPC_OK) return map_error(rpc);
  for (uint16_t i = 0U; i < async->private_state.count; ++i) {
    wl_rpc_async_slot_t *slot = &async->private_state.slots[i];
    if (slot->private_state.operation_id != result.operation_id) continue;
    if (!terminal(result.state)) {
      error = wl_rpc_async_cancel(async, call);
      if (error != WL_OK) return error;
      rpc = wl_rpc_client_get_by_handle(async->private_state.client, &call->private_state.handle, &result);
      if (rpc != WL_RPC_OK) return map_error(rpc);
    }
    async->private_state.servicing = 1U;
    error = notify_terminal(async, slot, &result);
    async->private_state.servicing = 0U;
    return error;
  }
  return WL_ERR_NOT_FOUND;
}

wl_err_t wl_rpc_async_service(wl_rpc_async_t *async, wl_time_ms_t now_ms,
    uint16_t *out_notified) {
  wl_err_t error = WL_OK;
  if (async == NULL || out_notified == NULL) return WL_ERR_INVALID_ARG;
  *out_notified = 0U;
  if (async->private_state.client == NULL) return WL_ERR_NOT_INITIALIZED;
  if (async->private_state.servicing || async->private_state.submitting)
    return WL_ERR_REENTRANT;
  async->private_state.servicing = 1U;
  collect_cancelled_tx(async);
  for (uint16_t i = 0U; i < async->private_state.count; ++i) {
    wl_rpc_async_slot_t *slot = &async->private_state.slots[i];
    wl_rpc_client_result_t result;
    wl_rpc_err_t rpc;
    if (slot->private_state.operation_id == 0U) continue;
    rpc = wl_rpc_client_get_by_handle(async->private_state.client,
        &slot->private_state.handle, &result);
    if (rpc != WL_RPC_OK) { error = map_error(rpc); break; }
    if (!terminal(result.state)) continue;
    error = notify_terminal(async, slot, &result);
    if (error != WL_OK) break;
    ++*out_notified;
  }
  if (error == WL_OK) error = submit_oldest(async, now_ms);
  async->private_state.servicing = 0U;
  return error;
}

uint32_t wl_rpc_async_notification_deadline(const wl_rpc_async_t *async) {
  if (async == NULL || async->private_state.client == NULL) return WL_RPC_NO_DEADLINE_MS;
  for (uint16_t i = 0U; i < async->private_state.count; ++i) {
    const wl_rpc_async_slot_t *slot = &async->private_state.slots[i];
    wl_rpc_client_result_t result;
    if (slot->private_state.operation_id != 0U &&
        wl_rpc_client_get_by_handle(async->private_state.client,
            &slot->private_state.handle, &result) == WL_RPC_OK && terminal(result.state))
      return 0U;
  }
  return WL_RPC_NO_DEADLINE_MS;
}

uint8_t wl_rpc_async_retire_tx(wl_rpc_async_t *async, wl_tx_handle_t handle) {
  if (async == NULL || handle == 0U || async->private_state.retiring_tx != handle)
    return 0U;
  async->private_state.retiring_tx = 0U;
  return 1U;
}

wl_err_t wl_rpc_async_close(wl_rpc_async_t *async) {
  uint16_t notified;
  wl_err_t error;
  if (async == NULL) return WL_ERR_INVALID_ARG;
  if (async->private_state.servicing || async->private_state.submitting)
    return WL_ERR_REENTRANT;
  if (async->private_state.client == NULL) return WL_OK;
  async->private_state.closing = 1U;
  for (uint16_t i = 0U; i < async->private_state.count; ++i) {
    wl_rpc_async_slot_t *slot = &async->private_state.slots[i];
    if (slot->private_state.operation_id != 0U)
      (void)wl_rpc_client_cancel_handle(async->private_state.client, &slot->private_state.handle);
  }
  error = wl_rpc_async_service(async, 0U, &notified);
  if (error == WL_OK) async->private_state.client = NULL;
  return error;
}
