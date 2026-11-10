/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/rpc.h"

#include <stdbool.h>
#include <string.h>

#define WL_RPC_CLIENT_MAGIC UINT32_C(0x52504343)
#define WL_RPC_SERVER_MAGIC UINT32_C(0x52504353)
#define WL_RPC_MAX_INTERVAL UINT32_C(0x80000000)

typedef struct {
  wl_rpc_client_slot_t *slots;
  uint8_t *response_storage;
  uint64_t next_generation;
  uint32_t next_operation_id;
  uint32_t magic;
  uint16_t slot_count;
  uint16_t response_capacity;
} wl_rpc_client_impl_t;

typedef struct {
  size_t response_length;
  uint64_t generation;
  uint32_t operation_id;
  wl_time_ms_t started_at;
  uint32_t timeout_ms;
  wl_tx_handle_t tx_handle;
  int32_t link_result;
  int32_t application_status;
  wl_rpc_err_t runtime_error;
  wl_rpc_client_state_t state;
  uint16_t request_message_id;
  uint16_t response_message_id;
  uint8_t link_delivery_confirmed;
} wl_rpc_client_slot_impl_t;

typedef struct {
  const wl_rpc_client_t *owner;
  uint64_t generation;
  uint16_t index;
} wl_rpc_client_handle_impl_t;

typedef struct {
  wl_rpc_server_pending_slot_t *pending_slots;
  wl_rpc_server_cache_slot_t *cache_slots;
  uint8_t *response_storage;
  wl_rpc_response_observer_fn response_observer;
  void *response_observer_data;
  uint64_t next_generation;
  uint32_t pending_timeout_ms;
  uint32_t cache_ttl_ms;
  uint32_t magic;
  uint16_t pending_slot_count;
  uint16_t cache_slot_count;
  uint16_t response_capacity;
  wl_rpc_cache_policy_t cache_policy;
} wl_rpc_server_impl_t;

typedef struct {
  wl_rpc_request_identity_t identity;
  uint64_t generation;
  wl_time_ms_t started_at;
  uint16_t cache_index;
  uint8_t active;
  uint8_t expiry_reported;
} wl_rpc_server_pending_impl_t;

typedef struct {
  wl_rpc_request_identity_t identity;
  size_t response_length;
  uint64_t generation;
  wl_tx_handle_t tx_handle;
  wl_time_ms_t completed_at;
  int32_t application_status;
  uint8_t active;
  uint8_t delivery_state;
} wl_rpc_server_cache_impl_t;

enum {
  WL_RPC_RESPONSE_RESERVED = 0,
  WL_RPC_RESPONSE_DELIVERED,
  WL_RPC_RESPONSE_READY,
  WL_RPC_RESPONSE_ACQUIRED,
  WL_RPC_RESPONSE_IN_FLIGHT,
};

_Static_assert(sizeof(wl_rpc_client_impl_t) <= WL_RPC_CLIENT_STORAGE_SIZE,
               "WL_RPC_CLIENT_STORAGE_SIZE is too small");
_Static_assert(sizeof(wl_rpc_client_handle_impl_t) <=
                   WL_RPC_CLIENT_HANDLE_STORAGE_SIZE,
               "WL_RPC_CLIENT_HANDLE_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_rpc_client_impl_t) <= _Alignof(wl_rpc_client_t),
               "wl_rpc_client_t alignment is too small");
_Static_assert(sizeof(wl_rpc_client_slot_impl_t) <=
                   WL_RPC_CLIENT_SLOT_STORAGE_SIZE,
               "WL_RPC_CLIENT_SLOT_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_rpc_client_slot_impl_t) <=
                   _Alignof(wl_rpc_client_slot_t),
               "wl_rpc_client_slot_t alignment is too small");
_Static_assert(sizeof(wl_rpc_server_impl_t) <= WL_RPC_SERVER_STORAGE_SIZE,
               "WL_RPC_SERVER_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_rpc_server_impl_t) <= _Alignof(wl_rpc_server_t),
               "wl_rpc_server_t alignment is too small");
_Static_assert(sizeof(wl_rpc_server_pending_impl_t) <=
                   WL_RPC_SERVER_PENDING_SLOT_STORAGE_SIZE,
               "WL_RPC_SERVER_PENDING_SLOT_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_rpc_server_pending_impl_t) <=
                   _Alignof(wl_rpc_server_pending_slot_t),
               "wl_rpc_server_pending_slot_t alignment is too small");
_Static_assert(sizeof(wl_rpc_server_cache_impl_t) <=
                   WL_RPC_SERVER_CACHE_SLOT_STORAGE_SIZE,
               "WL_RPC_SERVER_CACHE_SLOT_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_rpc_server_cache_impl_t) <=
                   _Alignof(wl_rpc_server_cache_slot_t),
               "wl_rpc_server_cache_slot_t alignment is too small");

static wl_rpc_client_impl_t *client_impl(wl_rpc_client_t *client) {
  return (wl_rpc_client_impl_t *)(void *)client;
}

static const wl_rpc_client_impl_t *
client_impl_const(const wl_rpc_client_t *client) {
  return (const wl_rpc_client_impl_t *)(const void *)client;
}

static wl_rpc_client_slot_impl_t *client_slot(wl_rpc_client_impl_t *client,
                                              uint16_t index) {
  return (wl_rpc_client_slot_impl_t *)(void *)&client->slots[index];
}

static const wl_rpc_client_slot_impl_t *
client_slot_const(const wl_rpc_client_impl_t *client, uint16_t index) {
  return (const wl_rpc_client_slot_impl_t *)(const void *)&client->slots[index];
}

static wl_rpc_server_impl_t *server_impl(wl_rpc_server_t *server) {
  return (wl_rpc_server_impl_t *)(void *)server;
}

static const wl_rpc_server_impl_t *
server_impl_const(const wl_rpc_server_t *server) {
  return (const wl_rpc_server_impl_t *)(const void *)server;
}

static wl_rpc_server_pending_impl_t *
server_pending(wl_rpc_server_impl_t *server, uint16_t index) {
  return (wl_rpc_server_pending_impl_t *)(void *)&server->pending_slots[index];
}

static wl_rpc_server_cache_impl_t *server_cache(wl_rpc_server_impl_t *server,
                                                uint16_t index) {
  return (wl_rpc_server_cache_impl_t *)(void *)&server->cache_slots[index];
}

static const wl_rpc_server_pending_impl_t *
server_pending_const(const wl_rpc_server_impl_t *server, uint16_t index) {
  return (const wl_rpc_server_pending_impl_t *)(const void *)&server
      ->pending_slots[index];
}

static const wl_rpc_server_cache_impl_t *
server_cache_const(const wl_rpc_server_impl_t *server, uint16_t index) {
  return (const wl_rpc_server_cache_impl_t *)(const void *)&server
      ->cache_slots[index];
}

static bool interval_valid(uint32_t interval_ms) {
  return interval_ms < WL_RPC_MAX_INTERVAL;
}

static bool elapsed(wl_time_ms_t now_ms, wl_time_ms_t started_at,
                    uint32_t interval_ms) {
  return interval_ms != 0U && (uint32_t)(now_ms - started_at) >= interval_ms;
}

static uint32_t deadline_remaining(wl_time_ms_t now_ms, wl_time_ms_t started_at,
                                   uint32_t interval_ms) {
  uint32_t age = (uint32_t)(now_ms - started_at);

  return age >= interval_ms ? 0U : interval_ms - age;
}

static bool client_initialized(const wl_rpc_client_t *client) {
  return client != NULL &&
         client_impl_const(client)->magic == WL_RPC_CLIENT_MAGIC;
}

static bool server_initialized(const wl_rpc_server_t *server) {
  return server != NULL &&
         server_impl_const(server)->magic == WL_RPC_SERVER_MAGIC;
}

static bool client_terminal(wl_rpc_client_state_t state) {
  return state == WL_RPC_CLIENT_COMPLETED ||
         state == WL_RPC_CLIENT_LINK_FAILED ||
         state == WL_RPC_CLIENT_TIMED_OUT || state == WL_RPC_CLIENT_CANCELLED ||
         state == WL_RPC_CLIENT_APPLICATION_ERROR;
}

static wl_rpc_client_slot_impl_t *client_find(wl_rpc_client_impl_t *client,
                                              uint32_t operation_id,
                                              uint16_t *out_index) {
  uint16_t i;

  for (i = 0U; i < client->slot_count; ++i) {
    wl_rpc_client_slot_impl_t *slot = client_slot(client, i);

    if (slot->state != WL_RPC_CLIENT_FREE &&
        slot->operation_id == operation_id) {
      if (out_index != NULL) {
        *out_index = i;
      }
      return slot;
    }
  }
  return NULL;
}

static const wl_rpc_client_slot_impl_t *
client_find_const(const wl_rpc_client_impl_t *client, uint32_t operation_id,
                  uint16_t *out_index) {
  uint16_t i;

  for (i = 0U; i < client->slot_count; ++i) {
    const wl_rpc_client_slot_impl_t *slot = client_slot_const(client, i);

    if (slot->state != WL_RPC_CLIENT_FREE &&
        slot->operation_id == operation_id) {
      if (out_index != NULL) {
        *out_index = i;
      }
      return slot;
    }
  }
  return NULL;
}

static wl_rpc_client_slot_impl_t *
client_find_free(wl_rpc_client_impl_t *client) {
  uint16_t i;

  for (i = 0U; i < client->slot_count; ++i) {
    wl_rpc_client_slot_impl_t *slot = client_slot(client, i);
    if (slot->state == WL_RPC_CLIENT_FREE) {
      return slot;
    }
  }
  return NULL;
}

const char *wl_rpc_err_str(wl_rpc_err_t error) {
  switch (error) {
  case WL_RPC_OK:
    return "ok";
  case WL_RPC_ERR_INVALID_ARG:
    return "invalid argument";
  case WL_RPC_ERR_NOT_INITIALIZED:
    return "not initialized";
  case WL_RPC_ERR_NO_SLOT:
    return "no free slot";
  case WL_RPC_ERR_OPERATION_CONFLICT:
    return "operation conflict";
  case WL_RPC_ERR_NOT_FOUND:
    return "not found";
  case WL_RPC_ERR_INVALID_STATE:
    return "invalid state";
  case WL_RPC_ERR_RESPONSE_MISMATCH:
    return "response mismatch";
  case WL_RPC_ERR_RESPONSE_TOO_LARGE:
    return "response too large";
  case WL_RPC_ERR_CACHE_FULL:
    return "response cache full";
  case WL_RPC_ERR_MALFORMED_METADATA:
    return "malformed RPC metadata";
  case WL_RPC_ERR_SESSION_MISMATCH:
    return "RPC session mismatch";
  case WL_RPC_ERR_ID_EXHAUSTED:
    return "RPC operation identities exhausted";
  default:
    return "unknown rpc error";
  }
}

wl_rpc_err_t wl_rpc_client_init(wl_rpc_client_t *client,
                                const wl_rpc_client_config_t *config) {
  size_t required;

  if (client == NULL || config == NULL || config->slots == NULL ||
      config->slot_count == 0U || config->response_storage == NULL ||
      config->response_capacity_per_slot == 0U) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  if ((size_t)config->slot_count >
      SIZE_MAX / (size_t)config->response_capacity_per_slot) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  required =
      (size_t)config->slot_count * (size_t)config->response_capacity_per_slot;
  if (config->response_storage_size < required) {
    return WL_RPC_ERR_INVALID_ARG;
  }

  memset(client, 0, sizeof(*client));
  memset(config->slots, 0,
         (size_t)config->slot_count * sizeof(config->slots[0]));
  wl_rpc_client_impl_t *impl = client_impl(client);
  impl->slots = config->slots;
  impl->response_storage = config->response_storage;
  impl->next_generation = 1U;
  impl->next_operation_id =
      config->next_operation_id == 0U ? 1U : config->next_operation_id;
  impl->slot_count = config->slot_count;
  impl->response_capacity = config->response_capacity_per_slot;
  impl->magic = WL_RPC_CLIENT_MAGIC;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_begin_with_id(
    wl_rpc_client_t *client, uint32_t operation_id, uint16_t request_message_id,
    uint16_t response_message_id, uint32_t timeout_ms, wl_time_ms_t now_ms) {
  wl_rpc_client_impl_t *impl;
  wl_rpc_client_slot_impl_t *slot;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U || request_message_id == 0U ||
      response_message_id == 0U || !interval_valid(timeout_ms)) {
    return WL_RPC_ERR_INVALID_ARG;
  }

  impl = client_impl(client);
  if (impl->next_generation == 0U) {
    return WL_RPC_ERR_NO_SLOT; /* Never resurrect an ancient handle on wrap. */
  }
  if (client_find(impl, operation_id, NULL) != NULL) {
    return WL_RPC_ERR_OPERATION_CONFLICT;
  }
  slot = client_find_free(impl);
  if (slot == NULL) {
    return WL_RPC_ERR_NO_SLOT;
  }

  memset(slot, 0, sizeof(*slot));
  slot->generation = impl->next_generation++;
  slot->operation_id = operation_id;
  slot->request_message_id = request_message_id;
  slot->response_message_id = response_message_id;
  slot->started_at = now_ms;
  slot->timeout_ms = timeout_ms;
  slot->state = WL_RPC_CLIENT_QUEUED;
  slot->runtime_error = WL_RPC_OK;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_begin(wl_rpc_client_t *client,
                                 uint16_t request_message_id,
                                 uint16_t response_message_id,
                                 uint32_t timeout_ms, wl_time_ms_t now_ms,
                                 uint32_t *out_operation_id) {
  wl_rpc_client_impl_t *impl;
  uint32_t candidate;
  uint32_t attempts;
  wl_rpc_err_t error;

  if (out_operation_id == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  *out_operation_id = 0U;
  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  impl = client_impl(client);
  candidate = impl->next_operation_id;
  if (candidate == 0U) return WL_RPC_ERR_ID_EXHAUSTED;
  for (attempts = 0U; attempts <= (uint32_t)impl->slot_count; ++attempts) {
    if (candidate == 0U) {
      return WL_RPC_ERR_ID_EXHAUSTED;
    }
    if (client_find(impl, candidate, NULL) == NULL) {
      break;
    }
    ++candidate;
  }
  if (attempts > (uint32_t)impl->slot_count) {
    return WL_RPC_ERR_NO_SLOT;
  }

  error = wl_rpc_client_begin_with_id(client, candidate, request_message_id,
                                      response_message_id, timeout_ms, now_ms);
  if (error != WL_RPC_OK) {
    return error;
  }
  impl->next_operation_id = candidate + 1U;
  *out_operation_id = candidate;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_bind_tx(wl_rpc_client_t *client,
                                   uint32_t operation_id,
                                   wl_tx_handle_t tx_handle) {
  wl_rpc_client_impl_t *impl;
  wl_rpc_client_slot_impl_t *slot;
  uint16_t i;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U || tx_handle == 0U) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = client_impl(client);
  slot = client_find(impl, operation_id, NULL);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  if (slot->state != WL_RPC_CLIENT_QUEUED) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  for (i = 0U; i < impl->slot_count; ++i) {
    const wl_rpc_client_slot_impl_t *candidate = client_slot_const(impl, i);
    if (candidate != slot && candidate->state == WL_RPC_CLIENT_LINK_PENDING &&
        candidate->tx_handle == tx_handle) {
      return WL_RPC_ERR_OPERATION_CONFLICT;
    }
  }
  slot->tx_handle = tx_handle;
  slot->state = WL_RPC_CLIENT_LINK_PENDING;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_tx_completed(wl_rpc_client_t *client,
                                        uint32_t operation_id) {
  wl_rpc_client_slot_impl_t *slot;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  slot = client_find(client_impl(client), operation_id, NULL);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  if (slot->state != WL_RPC_CLIENT_QUEUED) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  slot->link_delivery_confirmed = 0U;
  slot->state = WL_RPC_CLIENT_WAIT_RESPONSE;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_link_failed(wl_rpc_client_t *client,
                                       uint32_t operation_id,
                                       int32_t link_result) {
  wl_rpc_client_slot_impl_t *slot;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U || link_result == WL_OK) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  slot = client_find(client_impl(client), operation_id, NULL);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  if (slot->state != WL_RPC_CLIENT_QUEUED &&
      slot->state != WL_RPC_CLIENT_LINK_PENDING) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  slot->link_result = link_result;
  slot->state = WL_RPC_CLIENT_LINK_FAILED;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_on_tx_event(wl_rpc_client_t *client,
                                       const wl_event_t *event) {
  wl_rpc_client_impl_t *impl;
  uint16_t i;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (event == NULL || event->handle == 0U ||
      (event->type != WL_EVT_TX_SUCCESS && event->type != WL_EVT_TX_TIMEOUT &&
       event->type != WL_EVT_TX_FAILED)) {
    return WL_RPC_ERR_INVALID_ARG;
  }

  impl = client_impl(client);
  for (i = 0U; i < impl->slot_count; ++i) {
    wl_rpc_client_slot_impl_t *slot = client_slot(impl, i);
    if (slot->state != WL_RPC_CLIENT_LINK_PENDING ||
        slot->tx_handle != event->handle) {
      continue;
    }
    if (event->type == WL_EVT_TX_SUCCESS) {
      slot->link_result = event->io_result;
      slot->link_delivery_confirmed = 1U;
      slot->state = WL_RPC_CLIENT_WAIT_RESPONSE;
    } else {
      slot->link_result = event->io_result;
      if (slot->link_result == WL_OK) {
        slot->link_result = event->type == WL_EVT_TX_TIMEOUT ? WL_ERR_TIMEOUT
                                                             : WL_ERR_TX_FAILED;
      }
      slot->state = WL_RPC_CLIENT_LINK_FAILED;
    }
    return WL_RPC_OK;
  }
  return WL_RPC_ERR_NOT_FOUND;
}

wl_rpc_err_t wl_rpc_client_on_response(wl_rpc_client_t *client,
                                       uint16_t response_message_id,
                                       uint32_t operation_id,
                                       int32_t application_status,
                                       const uint8_t *response_payload,
                                       size_t response_length) {
  wl_rpc_client_impl_t *impl;
  wl_rpc_client_slot_impl_t *slot;
  uint16_t index;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U || response_message_id == 0U ||
      (response_length != 0U && response_payload == NULL)) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = client_impl(client);
  slot = client_find(impl, operation_id, &index);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  if (slot->response_message_id != response_message_id) {
    return WL_RPC_ERR_RESPONSE_MISMATCH;
  }
  if (slot->state != WL_RPC_CLIENT_LINK_PENDING &&
      slot->state != WL_RPC_CLIENT_WAIT_RESPONSE) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  if (response_length > (size_t)impl->response_capacity) {
    slot->runtime_error = WL_RPC_ERR_RESPONSE_TOO_LARGE;
    slot->state = WL_RPC_CLIENT_APPLICATION_ERROR;
    return WL_RPC_ERR_RESPONSE_TOO_LARGE;
  }
  if (response_length != 0U) {
    memmove(&impl->response_storage[(size_t)index * impl->response_capacity],
            response_payload, response_length);
  }
  slot->response_length = response_length;
  slot->application_status = application_status;
  slot->state = application_status == 0 ? WL_RPC_CLIENT_COMPLETED
                                        : WL_RPC_CLIENT_APPLICATION_ERROR;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_poll(wl_rpc_client_t *client, wl_time_ms_t now_ms,
                                uint16_t *out_timed_out) {
  wl_rpc_client_impl_t *impl;
  uint16_t i;
  uint16_t timed_out = 0U;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (out_timed_out == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = client_impl(client);
  for (i = 0U; i < impl->slot_count; ++i) {
    wl_rpc_client_slot_impl_t *slot = client_slot(impl, i);
    if ((slot->state == WL_RPC_CLIENT_QUEUED ||
         slot->state == WL_RPC_CLIENT_LINK_PENDING ||
         slot->state == WL_RPC_CLIENT_WAIT_RESPONSE) &&
        elapsed(now_ms, slot->started_at, slot->timeout_ms)) {
      slot->state = WL_RPC_CLIENT_TIMED_OUT;
      ++timed_out;
    }
  }
  *out_timed_out = timed_out;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_get_deadline_hint(const wl_rpc_client_t *client,
                                             wl_time_ms_t now_ms,
                                             wl_rpc_deadline_hint_t *out_hint) {
  const wl_rpc_client_impl_t *impl;
  uint32_t nearest = WL_RPC_NO_DEADLINE_MS;
  uint16_t i;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (out_hint == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = client_impl_const(client);
  for (i = 0U; i < impl->slot_count; ++i) {
    const wl_rpc_client_slot_impl_t *slot = client_slot_const(impl, i);
    uint32_t remaining;

    if (slot->timeout_ms == 0U ||
        (slot->state != WL_RPC_CLIENT_QUEUED &&
         slot->state != WL_RPC_CLIENT_LINK_PENDING &&
         slot->state != WL_RPC_CLIENT_WAIT_RESPONSE)) {
      continue;
    }
    remaining = deadline_remaining(now_ms, slot->started_at, slot->timeout_ms);
    if (remaining < nearest) {
      nearest = remaining;
    }
  }
  out_hint->next_deadline_ms = nearest;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_cancel(wl_rpc_client_t *client,
                                  uint32_t operation_id) {
  wl_rpc_client_slot_impl_t *slot;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  slot = client_find(client_impl(client), operation_id, NULL);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  if (client_terminal(slot->state)) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  slot->state = WL_RPC_CLIENT_CANCELLED;
  return WL_RPC_OK;
}

static void client_result(const wl_rpc_client_impl_t *impl,
                           const wl_rpc_client_slot_impl_t *slot,
                           uint16_t index, wl_rpc_client_result_t *out_result) {
  memset(out_result, 0, sizeof(*out_result));
  out_result->operation_id = slot->operation_id;
  out_result->request_message_id = slot->request_message_id;
  out_result->response_message_id = slot->response_message_id;
  out_result->state = slot->state;
  out_result->tx_handle = slot->tx_handle;
  out_result->link_result = slot->link_result;
  out_result->application_status = slot->application_status;
  out_result->runtime_error = slot->runtime_error;
  out_result->link_delivery_confirmed = slot->link_delivery_confirmed;
  out_result->response_data =
      &impl->response_storage[(size_t)index * impl->response_capacity];
  out_result->response_length = slot->response_length;
}

wl_rpc_err_t wl_rpc_client_get(const wl_rpc_client_t *client,
                               uint32_t operation_id,
                               wl_rpc_client_result_t *out_result) {
  const wl_rpc_client_impl_t *impl;
  const wl_rpc_client_slot_impl_t *slot;
  uint16_t index;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U || out_result == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = client_impl_const(client);
  slot = client_find_const(impl, operation_id, &index);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  client_result(impl, slot, index, out_result);
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_get_handle(const wl_rpc_client_t *client,
                                      uint32_t operation_id,
                                      wl_rpc_client_handle_t *out_handle) {
  const wl_rpc_client_slot_impl_t *slot;
  wl_rpc_client_handle_impl_t value = {0};

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U || out_handle == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  slot = client_find_const(client_impl_const(client), operation_id,
                            &value.index);
  if (slot == NULL) return WL_RPC_ERR_NOT_FOUND;
  value.owner = client;
  value.generation = slot->generation;
  memset(out_handle, 0, sizeof(*out_handle));
  memcpy(out_handle->private_bytes, &value, sizeof(value));
  return WL_RPC_OK;
}

static wl_rpc_err_t client_handle_index(const wl_rpc_client_t *client,
                                         const wl_rpc_client_handle_t *handle,
                                         uint16_t *out_index) {
  wl_rpc_client_handle_impl_t value;
  const wl_rpc_client_impl_t *impl;
  const wl_rpc_client_slot_impl_t *slot;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (handle == NULL) return WL_RPC_ERR_INVALID_ARG;
  memcpy(&value, handle->private_bytes, sizeof(value));
  impl = client_impl_const(client);
  if (value.owner != client || value.generation == 0U ||
      value.index >= impl->slot_count) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  slot = client_slot_const(impl, value.index);
  if (slot->state == WL_RPC_CLIENT_FREE ||
      slot->generation != value.generation) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  *out_index = value.index;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_get_by_handle(
    const wl_rpc_client_t *client, const wl_rpc_client_handle_t *handle,
    wl_rpc_client_result_t *out_result) {
  uint16_t index;
  wl_rpc_err_t error;

  if (out_result == NULL) return WL_RPC_ERR_INVALID_ARG;
  error = client_handle_index(client, handle, &index);
  if (error != WL_RPC_OK) return error;
  client_result(client_impl_const(client),
                  client_slot_const(client_impl_const(client), index), index,
                  out_result);
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_cancel_handle(
    wl_rpc_client_t *client, const wl_rpc_client_handle_t *handle) {
  uint16_t index;
  wl_rpc_err_t error = client_handle_index(client, handle, &index);
  wl_rpc_client_slot_impl_t *slot;

  if (error != WL_RPC_OK) return error;
  slot = client_slot(client_impl(client), index);
  if (client_terminal(slot->state)) return WL_RPC_ERR_INVALID_STATE;
  slot->state = WL_RPC_CLIENT_CANCELLED;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_release_handle(
    wl_rpc_client_t *client, const wl_rpc_client_handle_t *handle) {
  uint16_t index;
  wl_rpc_err_t error = client_handle_index(client, handle, &index);
  wl_rpc_client_slot_impl_t *slot;

  if (error != WL_RPC_OK) return error;
  slot = client_slot(client_impl(client), index);
  if (!client_terminal(slot->state)) return WL_RPC_ERR_INVALID_STATE;
  memset(slot, 0, sizeof(*slot));
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_client_release(wl_rpc_client_t *client,
                                   uint32_t operation_id) {
  wl_rpc_client_slot_impl_t *slot;

  if (!client_initialized(client)) {
    return client == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (operation_id == 0U) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  slot = client_find(client_impl(client), operation_id, NULL);
  if (slot == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  if (!client_terminal(slot->state)) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  memset(slot, 0, sizeof(*slot));
  return WL_RPC_OK;
}

static bool identity_valid(const wl_rpc_request_identity_t *identity) {
  return identity != NULL && identity->operation_id != 0U &&
         identity->request_message_id != 0U &&
         identity->response_message_id != 0U;
}

static bool identity_equal(const wl_rpc_request_identity_t *left,
                           const wl_rpc_request_identity_t *right) {
  return left->operation_id == right->operation_id &&
         left->request_message_id == right->request_message_id &&
         left->response_message_id == right->response_message_id &&
         left->request_fingerprint == right->request_fingerprint &&
         left->peer_session_id == right->peer_session_id;
}

static bool identity_key_equal(const wl_rpc_request_identity_t *left,
                               const wl_rpc_request_identity_t *right) {
  return left->operation_id == right->operation_id &&
         left->peer_session_id == right->peer_session_id;
}

static bool request_valid(const wl_rpc_server_request_t *request) {
  return request != NULL && request->generation != 0U &&
         identity_valid(&request->identity);
}

static bool request_equal(const wl_rpc_server_request_t *request,
                          const wl_rpc_server_pending_impl_t *pending) {
  return request->generation == pending->generation &&
         identity_equal(&request->identity, &pending->identity);
}

static uint64_t server_next_generation(wl_rpc_server_impl_t *server) {
  uint64_t generation = server->next_generation++;

  if (generation == 0U) {
    generation = server->next_generation++;
  }
  return generation;
}

static bool generation_older(uint64_t left, uint64_t right) {
  return left != right && (right - left) < (UINT64_C(1) << 63);
}

static void response_from_cache(const wl_rpc_server_impl_t *server,
                                uint16_t index,
                                wl_rpc_server_response_t *out_response) {
  const wl_rpc_server_cache_impl_t *cache =
      (const wl_rpc_server_cache_impl_t *)(const void *)&server
          ->cache_slots[index];
  out_response->identity = cache->identity;
  out_response->application_status = cache->application_status;
  out_response->response_data =
      &server->response_storage[(size_t)index * server->response_capacity];
  out_response->response_length = cache->response_length;
  out_response->generation = cache->generation;
}

static wl_rpc_server_cache_impl_t *server_response_find(
    wl_rpc_server_impl_t *server, const wl_rpc_server_response_t *response) {
  uint16_t i;

  if (response == NULL || !identity_valid(&response->identity)) {
    return NULL;
  }
  for (i = 0U; i < server->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *cache = server_cache(server, i);
    const uint8_t *data =
        &server->response_storage[(size_t)i * server->response_capacity];
    if (cache->active != 0U && cache->generation == response->generation &&
        identity_equal(&cache->identity, &response->identity) &&
        response->response_data == data &&
        response->response_length == cache->response_length) {
      return cache;
    }
  }
  return NULL;
}

static bool server_cache_expire(wl_rpc_server_cache_impl_t *cache,
                                wl_time_ms_t now_ms, uint32_t ttl_ms) {
  if (cache->active == 0U ||
      cache->delivery_state != WL_RPC_RESPONSE_DELIVERED ||
      !elapsed(now_ms, cache->completed_at, ttl_ms)) {
    return false;
  }
  memset(cache, 0, sizeof(*cache));
  return true;
}

static void server_expire(wl_rpc_server_impl_t *server, wl_time_ms_t now_ms,
                          wl_rpc_server_expiry_t *expiry) {
  uint16_t i;

  memset(expiry, 0, sizeof(*expiry));
  for (i = 0U; i < server->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *cache = server_cache(server, i);
    if (server_cache_expire(cache, now_ms, server->cache_ttl_ms)) {
      ++expiry->cache_expired;
    }
  }
}

wl_rpc_err_t wl_rpc_server_init(wl_rpc_server_t *server,
                                const wl_rpc_server_config_t *config) {
  size_t required;

  if (server == NULL || config == NULL || config->pending_slots == NULL ||
      config->pending_slot_count == 0U || config->cache_slots == NULL ||
      config->cache_slot_count == 0U || config->response_storage == NULL ||
      config->response_capacity_per_slot == 0U ||
      !interval_valid(config->pending_timeout_ms) ||
      !interval_valid(config->cache_ttl_ms) ||
      (config->cache_policy != WL_RPC_CACHE_REJECT_NEW &&
       config->cache_policy != WL_RPC_CACHE_EVICT_OLDEST)) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  if ((size_t)config->cache_slot_count >
      SIZE_MAX / (size_t)config->response_capacity_per_slot) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  required = (size_t)config->cache_slot_count *
             (size_t)config->response_capacity_per_slot;
  if (config->response_storage_size < required) {
    return WL_RPC_ERR_INVALID_ARG;
  }

  memset(server, 0, sizeof(*server));
  memset(config->pending_slots, 0,
         (size_t)config->pending_slot_count * sizeof(config->pending_slots[0]));
  memset(config->cache_slots, 0,
         (size_t)config->cache_slot_count * sizeof(config->cache_slots[0]));
  wl_rpc_server_impl_t *impl = server_impl(server);
  impl->pending_slots = config->pending_slots;
  impl->cache_slots = config->cache_slots;
  impl->response_storage = config->response_storage;
  impl->next_generation = 1U;
  impl->pending_timeout_ms = config->pending_timeout_ms;
  impl->cache_ttl_ms = config->cache_ttl_ms;
  impl->pending_slot_count = config->pending_slot_count;
  impl->cache_slot_count = config->cache_slot_count;
  impl->response_capacity = config->response_capacity_per_slot;
  impl->cache_policy = config->cache_policy;
  impl->magic = WL_RPC_SERVER_MAGIC;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_begin(wl_rpc_server_t *server,
                                 const wl_rpc_request_identity_t *identity,
                                 wl_time_ms_t now_ms,
                                 wl_rpc_server_disposition_t *out_disposition,
                                 wl_rpc_server_request_t *out_request,
                                 wl_rpc_server_response_t *out_replay) {
  wl_rpc_server_impl_t *impl;
  wl_rpc_server_pending_impl_t *free_pending = NULL;
  wl_rpc_server_cache_impl_t *target = NULL;
  uint16_t target_index = 0U;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (!identity_valid(identity) || out_disposition == NULL ||
      out_request == NULL ||
      out_replay == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  memset(out_request, 0, sizeof(*out_request));
  memset(out_replay, 0, sizeof(*out_replay));
  impl = server_impl(server);

  for (i = 0U; i < impl->pending_slot_count; ++i) {
    wl_rpc_server_pending_impl_t *pending = server_pending(impl, i);
    if (pending->active == 0U) {
      if (free_pending == NULL) {
        free_pending = pending;
      }
      continue;
    }
    if (identity_key_equal(&pending->identity, identity)) {
      *out_disposition = identity_equal(&pending->identity, identity)
                             ? WL_RPC_SERVER_PENDING_DUPLICATE
                             : WL_RPC_SERVER_CONFLICT;
      return WL_RPC_OK;
    }
  }
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *cache = server_cache(impl, i);
    /* Dispatch can precede poll after an idle gap. Reclaim only delivered
     * responses, in the existing lookup pass, before deduplication/admission. */
    (void)server_cache_expire(cache, now_ms, impl->cache_ttl_ms);
    if (cache->active != 0U && identity_key_equal(&cache->identity, identity)) {
      if (identity_equal(&cache->identity, identity)) {
        if (cache->delivery_state == WL_RPC_RESPONSE_DELIVERED) {
          cache->delivery_state = WL_RPC_RESPONSE_READY;
        }
        *out_disposition = WL_RPC_SERVER_REPLAY;
        response_from_cache(impl, i, out_replay);
      } else {
        *out_disposition = WL_RPC_SERVER_CONFLICT;
      }
      return WL_RPC_OK;
    }
  }
  if (free_pending == NULL) {
    return WL_RPC_ERR_NO_SLOT;
  }

  /* Reserve stable response storage before the application may execute. */
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *candidate = server_cache(impl, i);
    if (candidate->active == 0U) {
      target = candidate;
      target_index = i;
      break;
    }
  }
  if (target == NULL && impl->cache_policy == WL_RPC_CACHE_EVICT_OLDEST) {
    for (i = 0U; i < impl->cache_slot_count; ++i) {
      wl_rpc_server_cache_impl_t *candidate = server_cache(impl, i);
      if (candidate->delivery_state != WL_RPC_RESPONSE_DELIVERED) {
        continue;
      }
      if (target == NULL ||
          generation_older(candidate->generation, target->generation)) {
        target = candidate;
        target_index = i;
      }
    }
  }
  if (target == NULL) {
    return WL_RPC_ERR_CACHE_FULL;
  }

  memset(target, 0, sizeof(*target));
  target->identity = *identity;
  target->generation = server_next_generation(impl);
  target->active = 1U;
  target->delivery_state = WL_RPC_RESPONSE_RESERVED;
  memset(free_pending, 0, sizeof(*free_pending));
  free_pending->identity = *identity;
  free_pending->generation = target->generation;
  free_pending->started_at = now_ms;
  free_pending->cache_index = target_index;
  free_pending->active = 1U;
  out_request->identity = *identity;
  out_request->generation = target->generation;
  *out_disposition = WL_RPC_SERVER_NEW;
  return WL_RPC_OK;
}

static wl_rpc_err_t server_resolve_pending(
    wl_rpc_server_impl_t *server, const wl_rpc_server_request_t *request,
    wl_rpc_server_pending_impl_t **out_pending,
    wl_rpc_server_cache_impl_t **out_cache, uint16_t *out_cache_index) {
  wl_rpc_server_pending_impl_t *pending = NULL;
  wl_rpc_server_cache_impl_t *cache;
  uint16_t cache_index;
  uint16_t i;

  for (i = 0U; i < server->pending_slot_count; ++i) {
    wl_rpc_server_pending_impl_t *candidate = server_pending(server, i);
    if (candidate->active != 0U && request_equal(request, candidate)) {
      pending = candidate;
      break;
    }
  }
  if (pending == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  cache_index = pending->cache_index;
  if (cache_index >= server->cache_slot_count) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  cache = server_cache(server, cache_index);
  if (cache->active == 0U ||
      cache->delivery_state != WL_RPC_RESPONSE_RESERVED ||
      cache->generation != request->generation ||
      !identity_equal(&cache->identity, &request->identity)) {
    return WL_RPC_ERR_INVALID_STATE;
  }

  *out_pending = pending;
  *out_cache = cache;
  *out_cache_index = cache_index;
  return WL_RPC_OK;
}

static void server_commit_reserved(wl_rpc_server_impl_t *server,
                                   wl_rpc_server_pending_impl_t *pending,
                                   wl_rpc_server_cache_impl_t *cache,
                                   uint16_t cache_index,
                                   int32_t application_status,
                                   size_t response_length,
                                   wl_time_ms_t now_ms,
                                   wl_rpc_server_response_t *out_response) {
  cache->response_length = response_length;
  cache->completed_at = now_ms;
  cache->application_status = application_status;
  cache->delivery_state = WL_RPC_RESPONSE_READY;
  memset(pending, 0, sizeof(*pending));
  response_from_cache(server, cache_index, out_response);
}

wl_rpc_err_t wl_rpc_server_response_prepare(
    wl_rpc_server_t *server, const wl_rpc_server_request_t *request,
    wl_rpc_server_response_buffer_t *out_buffer) {
  wl_rpc_server_impl_t *impl;
  wl_rpc_server_pending_impl_t *pending;
  wl_rpc_server_cache_impl_t *cache;
  uint16_t cache_index;
  wl_rpc_err_t result;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (!request_valid(request) || out_buffer == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  memset(out_buffer, 0, sizeof(*out_buffer));
  impl = server_impl(server);
  result = server_resolve_pending(impl, request, &pending, &cache,
                                  &cache_index);
  if (result != WL_RPC_OK) {
    return result;
  }
  (void)pending;
  (void)cache;
  out_buffer->request = *request;
  out_buffer->data =
      &impl->response_storage[(size_t)cache_index * impl->response_capacity];
  out_buffer->capacity = impl->response_capacity;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_response_commit(
    wl_rpc_server_t *server, const wl_rpc_server_response_buffer_t *buffer,
    int32_t application_status, size_t response_length, wl_time_ms_t now_ms,
    wl_rpc_server_response_t *out_response) {
  wl_rpc_server_impl_t *impl;
  wl_rpc_server_pending_impl_t *pending;
  wl_rpc_server_cache_impl_t *cache;
  uint8_t *expected_data;
  uint16_t cache_index;
  wl_rpc_err_t result;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (buffer == NULL || !request_valid(&buffer->request) ||
      out_response == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = server_impl(server);
  if (response_length > (size_t)impl->response_capacity) {
    return WL_RPC_ERR_RESPONSE_TOO_LARGE;
  }
  result = server_resolve_pending(impl, &buffer->request, &pending, &cache,
                                  &cache_index);
  if (result != WL_RPC_OK) {
    return result;
  }
  expected_data =
      &impl->response_storage[(size_t)cache_index * impl->response_capacity];
  if (buffer->data != expected_data ||
      buffer->capacity != (size_t)impl->response_capacity) {
    return WL_RPC_ERR_RESPONSE_MISMATCH;
  }
  server_commit_reserved(impl, pending, cache, cache_index,
                         application_status, response_length, now_ms,
                         out_response);
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_complete(wl_rpc_server_t *server,
                                    const wl_rpc_server_request_t *request,
                                    int32_t application_status,
                                    const uint8_t *response_payload,
                                    size_t response_length, wl_time_ms_t now_ms,
                                    wl_rpc_server_response_t *out_response) {
  wl_rpc_server_impl_t *impl;
  wl_rpc_server_pending_impl_t *pending;
  wl_rpc_server_cache_impl_t *target;
  uint16_t target_index;
  uint8_t *target_data;
  wl_rpc_err_t result;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (!request_valid(request) || out_response == NULL ||
      (response_length != 0U && response_payload == NULL)) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = server_impl(server);
  if (response_length > (size_t)impl->response_capacity) {
    return WL_RPC_ERR_RESPONSE_TOO_LARGE;
  }
  result = server_resolve_pending(impl, request, &pending, &target,
                                  &target_index);
  if (result != WL_RPC_OK) {
    return result;
  }

  target_data =
      &impl->response_storage[(size_t)target_index * impl->response_capacity];
  if (response_length != 0U && response_payload != target_data) {
    memmove(target_data, response_payload, response_length);
  }
  server_commit_reserved(impl, pending, target, target_index,
                         application_status, response_length, now_ms,
                         out_response);
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_reject(wl_rpc_server_t *server,
                                  const wl_rpc_server_request_t *request,
                                  int32_t application_status,
                                  const uint8_t *response_payload,
                                  size_t response_length, wl_time_ms_t now_ms,
                                  wl_rpc_server_response_t *out_response) {
  if (application_status == 0) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  return wl_rpc_server_complete(server, request, application_status,
                                response_payload, response_length, now_ms,
                                out_response);
}

wl_rpc_err_t wl_rpc_server_response_acquire(
    wl_rpc_server_t *server, wl_rpc_server_response_t *out_response) {
  wl_rpc_server_impl_t *impl;
  wl_rpc_server_cache_impl_t *oldest = NULL;
  uint16_t oldest_index = 0U;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (out_response == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  memset(out_response, 0, sizeof(*out_response));
  impl = server_impl(server);
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *candidate = server_cache(impl, i);
    if (candidate->active == 0U ||
        candidate->delivery_state != WL_RPC_RESPONSE_READY) {
      continue;
    }
    if (oldest == NULL ||
        generation_older(candidate->generation, oldest->generation)) {
      oldest = candidate;
      oldest_index = i;
    }
  }
  if (oldest == NULL) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  oldest->delivery_state = WL_RPC_RESPONSE_ACQUIRED;
  response_from_cache(impl, oldest_index, out_response);
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_response_defer(
    wl_rpc_server_t *server, const wl_rpc_server_response_t *response) {
  wl_rpc_server_cache_impl_t *cache;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  cache = server_response_find(server_impl(server), response);
  if (cache == NULL) {
    return response == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_FOUND;
  }
  if (cache->delivery_state != WL_RPC_RESPONSE_ACQUIRED) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  cache->delivery_state = WL_RPC_RESPONSE_READY;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_response_submitted(
    wl_rpc_server_t *server, const wl_rpc_server_response_t *response,
    wl_tx_handle_t tx_handle) {
  wl_rpc_server_impl_t *impl;
  wl_rpc_server_cache_impl_t *cache;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (tx_handle == 0U) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = server_impl(server);
  cache = server_response_find(impl, response);
  if (cache == NULL) {
    return response == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_FOUND;
  }
  if (cache->delivery_state != WL_RPC_RESPONSE_ACQUIRED) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *candidate = server_cache(impl, i);
    if (candidate != cache && candidate->active != 0U &&
        candidate->delivery_state == WL_RPC_RESPONSE_IN_FLIGHT &&
        candidate->tx_handle == tx_handle) {
      return WL_RPC_ERR_OPERATION_CONFLICT;
    }
  }
  cache->tx_handle = tx_handle;
  cache->delivery_state = WL_RPC_RESPONSE_IN_FLIGHT;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_response_sent(
    wl_rpc_server_t *server, const wl_rpc_server_response_t *response) {
  wl_rpc_server_cache_impl_t *cache;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  cache = server_response_find(server_impl(server), response);
  if (cache == NULL) {
    return response == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_FOUND;
  }
  if (cache->delivery_state != WL_RPC_RESPONSE_ACQUIRED) {
    return WL_RPC_ERR_INVALID_STATE;
  }
  cache->delivery_state = WL_RPC_RESPONSE_DELIVERED;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_set_response_observer(wl_rpc_server_t *server,
    wl_rpc_response_observer_fn observer, void *user_data) {
  if (!server_initialized(server))
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  server_impl(server)->response_observer = observer;
  server_impl(server)->response_observer_data = user_data;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_on_tx_event(wl_rpc_server_t *server,
                                       const wl_event_t *event) {
  wl_rpc_server_impl_t *impl;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (event == NULL ||
      (event->type != WL_EVT_TX_SUCCESS && event->type != WL_EVT_TX_TIMEOUT &&
       event->type != WL_EVT_TX_FAILED)) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  if (event->handle == 0U) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  impl = server_impl(server);
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *cache = server_cache(impl, i);
    if (cache->active == 0U ||
        cache->delivery_state != WL_RPC_RESPONSE_IN_FLIGHT ||
        cache->tx_handle != event->handle) {
      continue;
    }
    cache->tx_handle = 0U;
    cache->delivery_state = WL_RPC_RESPONSE_DELIVERED;
    if (impl->response_observer != NULL)
      impl->response_observer(impl->response_observer_data, &cache->identity,
                              cache->application_status, event);
    return WL_RPC_OK;
  }
  return WL_RPC_ERR_NOT_FOUND;
}

wl_rpc_err_t wl_rpc_server_discard_session(
    wl_rpc_server_t *server, uint64_t peer_session_id,
    wl_rpc_server_cancel_tx_fn_t cancel_tx, void *cancel_context,
    wl_rpc_server_discard_result_t *out_result) {
  wl_rpc_server_impl_t *impl;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (peer_session_id == 0U || out_result == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  memset(out_result, 0, sizeof(*out_result));
  impl = server_impl(server);
  for (i = 0U; i < impl->pending_slot_count; ++i) {
    wl_rpc_server_pending_impl_t *pending = server_pending(impl, i);
    wl_rpc_server_cache_impl_t *reserved;

    if (pending->active == 0U ||
        pending->identity.peer_session_id != peer_session_id) {
      continue;
    }
    reserved = pending->cache_index < impl->cache_slot_count
                   ? server_cache(impl, pending->cache_index)
                   : NULL;
    if (reserved != NULL && reserved->active != 0U &&
        reserved->delivery_state == WL_RPC_RESPONSE_RESERVED &&
        reserved->generation == pending->generation) {
      memset(reserved, 0, sizeof(*reserved));
    }
    memset(pending, 0, sizeof(*pending));
    ++out_result->pending_discarded;
  }
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    wl_rpc_server_cache_impl_t *cache = server_cache(impl, i);
    wl_tx_handle_t cancel_handle;

    if (cache->active == 0U ||
        cache->identity.peer_session_id != peer_session_id) {
      continue;
    }
    cancel_handle = cache->delivery_state == WL_RPC_RESPONSE_IN_FLIGHT
                        ? cache->tx_handle
                        : 0U;
    memset(cache, 0, sizeof(*cache));
    ++out_result->responses_discarded;
    if (cancel_handle != 0U) {
      ++out_result->tx_cancel_requested;
      if (cancel_tx != NULL) {
        cancel_tx(cancel_context, cancel_handle);
      }
    }
  }
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_peer_observe(
    wl_rpc_server_t *server, wl_rpc_peer_t *peer, uint64_t peer_session_id,
    wl_rpc_server_cancel_tx_fn_t cancel_tx, void *cancel_context,
    wl_rpc_peer_observation_t *out_observation) {
  wl_rpc_err_t result;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (peer == NULL || peer_session_id == 0U || out_observation == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  memset(out_observation, 0, sizeof(*out_observation));
  out_observation->previous_session_id = peer->session_id;
  out_observation->current_session_id = peer_session_id;
  if (peer->session_id == peer_session_id) {
    return WL_RPC_OK;
  }
  if (peer->session_id != 0U) {
    result = wl_rpc_server_discard_session(
        server, peer->session_id, cancel_tx, cancel_context,
        &out_observation->discarded);
    if (result != WL_RPC_OK) {
      return result;
    }
  }
  peer->session_id = peer_session_id;
  out_observation->changed = 1U;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_abandon(wl_rpc_server_t *server,
                                   const wl_rpc_server_request_t *request) {
  wl_rpc_server_impl_t *impl;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (!request_valid(request)) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = server_impl(server);
  for (i = 0U; i < impl->pending_slot_count; ++i) {
    wl_rpc_server_pending_impl_t *pending = server_pending(impl, i);
    if (pending->active != 0U && request_equal(request, pending)) {
      if (pending->cache_index < impl->cache_slot_count) {
        wl_rpc_server_cache_impl_t *reserved =
            server_cache(impl, pending->cache_index);
        if (reserved->active != 0U &&
            reserved->delivery_state == WL_RPC_RESPONSE_RESERVED &&
            reserved->generation == pending->generation) {
          memset(reserved, 0, sizeof(*reserved));
        }
      }
      memset(pending, 0, sizeof(*pending));
      return WL_RPC_OK;
    }
  }
  return WL_RPC_ERR_NOT_FOUND;
}

wl_rpc_err_t wl_rpc_server_expired_acquire(
    wl_rpc_server_t *server, wl_time_ms_t now_ms,
    wl_rpc_server_request_t *out_request) {
  wl_rpc_server_impl_t *impl;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (out_request == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  memset(out_request, 0, sizeof(*out_request));
  impl = server_impl(server);
  if (impl->pending_timeout_ms == 0U) {
    return WL_RPC_ERR_NOT_FOUND;
  }
  for (i = 0U; i < impl->pending_slot_count; ++i) {
    wl_rpc_server_pending_impl_t *pending = server_pending(impl, i);
    if (pending->active != 0U && pending->expiry_reported == 0U &&
        elapsed(now_ms, pending->started_at, impl->pending_timeout_ms)) {
      pending->expiry_reported = 1U;
      out_request->identity = pending->identity;
      out_request->generation = pending->generation;
      return WL_RPC_OK;
    }
  }
  return WL_RPC_ERR_NOT_FOUND;
}

wl_rpc_err_t wl_rpc_server_poll(wl_rpc_server_t *server, wl_time_ms_t now_ms,
                                wl_rpc_server_expiry_t *out_expiry) {
  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (out_expiry == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  server_expire(server_impl(server), now_ms, out_expiry);
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_server_get_deadline_hint(const wl_rpc_server_t *server,
                                             wl_time_ms_t now_ms,
                                             wl_rpc_deadline_hint_t *out_hint) {
  const wl_rpc_server_impl_t *impl;
  uint32_t nearest = WL_RPC_NO_DEADLINE_MS;
  uint16_t i;

  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  if (out_hint == NULL) {
    return WL_RPC_ERR_INVALID_ARG;
  }
  impl = server_impl_const(server);
  for (i = 0U; i < impl->cache_slot_count; ++i) {
    const wl_rpc_server_cache_impl_t *cache = server_cache_const(impl, i);

    if (cache->active != 0U &&
        cache->delivery_state == WL_RPC_RESPONSE_READY) {
      out_hint->next_deadline_ms = 0U;
      return WL_RPC_OK;
    }
    if (impl->cache_ttl_ms != 0U && cache->active != 0U &&
        cache->delivery_state == WL_RPC_RESPONSE_DELIVERED) {
      const uint32_t remaining =
          deadline_remaining(now_ms, cache->completed_at, impl->cache_ttl_ms);
      if (remaining < nearest) nearest = remaining;
    }
  }
  if (impl->pending_timeout_ms != 0U) {
    for (i = 0U; i < impl->pending_slot_count; ++i) {
      const wl_rpc_server_pending_impl_t *pending =
          server_pending_const(impl, i);
      uint32_t remaining;

      if (pending->active == 0U || pending->expiry_reported != 0U) {
        continue;
      }
      remaining = deadline_remaining(now_ms, pending->started_at,
                                     impl->pending_timeout_ms);
      if (remaining < nearest) {
        nearest = remaining;
      }
    }
  }
  out_hint->next_deadline_ms = nearest;
  return WL_RPC_OK;
}

#ifdef WL_RPC_TEST_HOOKS
wl_rpc_err_t wl_rpc_test_client_set_next_generation(wl_rpc_client_t *client,
                                                    uint64_t generation) {
  if (!client_initialized(client)) return WL_RPC_ERR_NOT_INITIALIZED;
  client_impl(client)->next_generation = generation;
  return WL_RPC_OK;
}

wl_rpc_err_t wl_rpc_test_server_set_next_generation(wl_rpc_server_t *server,
                                                    uint64_t generation) {
  if (!server_initialized(server)) {
    return server == NULL ? WL_RPC_ERR_INVALID_ARG : WL_RPC_ERR_NOT_INITIALIZED;
  }
  server_impl(server)->next_generation = generation;
  return WL_RPC_OK;
}
#endif
