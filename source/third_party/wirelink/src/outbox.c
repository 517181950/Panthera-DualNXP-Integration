/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/outbox.h"

#include <stdbool.h>
#include <string.h>

#define WL_OUTBOX_MAGIC UINT32_C(0x4f555442)

typedef struct {
  wl_outbox_slot_t *slots;
  uint8_t *payload_storage;
  uint64_t next_generation;
  uint64_t submitted;
  uint64_t coalesced;
  uint64_t queue_full;
  uint64_t acquired;
  uint64_t accepted;
  uint64_t deferred;
  uint64_t rejected;
  uint64_t superseded;
  uint64_t resets;
  uint64_t active_token;
  uint32_t magic;
  uint16_t slot_count;
  uint16_t payload_capacity;
  uint16_t cursor;
  uint16_t depth;
  uint16_t active_index;
  uint8_t active;
} wl_outbox_impl_t;

typedef struct {
  uint64_t generation;
  size_t payload_length;
  uint16_t message_id;
  uint8_t valid;
} wl_outbox_slot_impl_t;

_Static_assert(sizeof(wl_outbox_impl_t) <= WL_OUTBOX_STORAGE_SIZE,
               "WL_OUTBOX_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_outbox_impl_t) <= _Alignof(wl_outbox_t),
               "wl_outbox_t alignment is too small");
_Static_assert(sizeof(wl_outbox_slot_impl_t) <= WL_OUTBOX_SLOT_STORAGE_SIZE,
               "WL_OUTBOX_SLOT_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_outbox_slot_impl_t) <= _Alignof(wl_outbox_slot_t),
               "wl_outbox_slot_t alignment is too small");

static wl_outbox_impl_t *impl(wl_outbox_t *outbox) {
  return (wl_outbox_impl_t *)(void *)outbox;
}

static const wl_outbox_impl_t *impl_const(const wl_outbox_t *outbox) {
  return (const wl_outbox_impl_t *)(const void *)outbox;
}

static wl_outbox_slot_impl_t *slot(wl_outbox_impl_t *outbox, uint16_t index) {
  return (wl_outbox_slot_impl_t *)(void *)&outbox->slots[index];
}

static const wl_outbox_slot_impl_t *
slot_const(const wl_outbox_impl_t *outbox, uint16_t index) {
  return (const wl_outbox_slot_impl_t *)(const void *)&outbox->slots[index];
}

static bool initialized(const wl_outbox_t *outbox) {
  return outbox != NULL && impl_const(outbox)->magic == WL_OUTBOX_MAGIC;
}

static uint8_t *payload_at(wl_outbox_impl_t *outbox, uint16_t index) {
  return outbox->payload_storage +
         (size_t)index * (size_t)outbox->payload_capacity;
}

wl_err_t wl_outbox_init(wl_outbox_t *outbox,
                        const wl_outbox_config_t *config) {
  size_t required;
  wl_outbox_impl_t *state;

  if (outbox == NULL || config == NULL || config->slots == NULL ||
      config->slot_count == 0U || config->payload_storage == NULL ||
      config->payload_capacity_per_slot == 0U ||
      (size_t)config->slot_count >
          SIZE_MAX / (size_t)config->payload_capacity_per_slot) {
    return WL_ERR_INVALID_ARG;
  }
  required = (size_t)config->slot_count *
             (size_t)config->payload_capacity_per_slot;
  if (config->payload_storage_size < required) {
    return WL_ERR_INVALID_ARG;
  }

  memset(outbox, 0, sizeof(*outbox));
  memset(config->slots, 0,
         (size_t)config->slot_count * sizeof(config->slots[0]));
  state = impl(outbox);
  state->slots = config->slots;
  state->payload_storage = config->payload_storage;
  state->next_generation = config->initial_generation;
  state->slot_count = config->slot_count;
  state->payload_capacity = config->payload_capacity_per_slot;
  state->magic = WL_OUTBOX_MAGIC;
  return WL_OK;
}

wl_err_t wl_outbox_submit_latest(wl_outbox_t *outbox, uint16_t message_id,
                                 const uint8_t *payload, size_t payload_length,
                                 uint8_t *out_coalesced) {
  wl_outbox_impl_t *state;
  wl_outbox_slot_impl_t *selected = NULL;
  uint16_t selected_index = 0U;
  uint16_t index;
  bool replaced = false;

  if (out_coalesced != NULL) {
    *out_coalesced = 0U;
  }
  if (!initialized(outbox)) {
    return outbox == NULL ? WL_ERR_INVALID_ARG : WL_ERR_NOT_INITIALIZED;
  }
  state = impl(outbox);
  if (message_id == 0U || payload_length > state->payload_capacity ||
      (payload == NULL && payload_length != 0U)) {
    return WL_ERR_INVALID_ARG;
  }

  for (index = 0U; index < state->slot_count; ++index) {
    wl_outbox_slot_impl_t *candidate = slot(state, index);
    if (candidate->valid != 0U && candidate->message_id == message_id) {
      selected = candidate;
      selected_index = index;
      replaced = true;
      break;
    }
  }
  if (selected == NULL) {
    for (index = 0U; index < state->slot_count; ++index) {
      wl_outbox_slot_impl_t *candidate = slot(state, index);
      if (candidate->valid == 0U) {
        selected = candidate;
        selected_index = index;
        break;
      }
    }
  }
  if (selected == NULL) {
    ++state->queue_full;
    return WL_ERR_QUEUE_FULL;
  }

  ++state->next_generation;
  if (state->next_generation == 0U) {
    ++state->next_generation;
  }
  if (payload_length != 0U) {
    memcpy(payload_at(state, selected_index), payload, payload_length);
  }
  selected->generation = state->next_generation;
  selected->payload_length = payload_length;
  selected->message_id = message_id;
  selected->valid = 1U;
  ++state->submitted;
  if (replaced) {
    ++state->coalesced;
  } else {
    ++state->depth;
  }
  if (out_coalesced != NULL) {
    *out_coalesced = replaced ? 1U : 0U;
  }
  return WL_OK;
}

wl_err_t wl_outbox_acquire_copy(wl_outbox_t *outbox, uint8_t *payload,
                                size_t payload_capacity,
                                wl_outbox_item_t *out_item) {
  wl_outbox_impl_t *state;
  uint16_t offset;

  if (!initialized(outbox)) {
    return outbox == NULL ? WL_ERR_INVALID_ARG : WL_ERR_NOT_INITIALIZED;
  }
  if (payload == NULL || out_item == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  state = impl(outbox);
  if (state->active != 0U) {
    return WL_ERR_BUSY;
  }

  for (offset = 0U; offset < state->slot_count; ++offset) {
    const uint16_t index =
        (uint16_t)((state->cursor + offset) % state->slot_count);
    const wl_outbox_slot_impl_t *candidate = slot_const(state, index);
    if (candidate->valid == 0U) {
      continue;
    }
    if (candidate->payload_length > payload_capacity) {
      return WL_ERR_NO_SPACE;
    }
    if (candidate->payload_length != 0U) {
      memcpy(payload, payload_at(state, index), candidate->payload_length);
    }
    *out_item = (wl_outbox_item_t){
        .token = candidate->generation,
        .slot_index = index,
        .message_id = candidate->message_id,
        .payload_length = candidate->payload_length,
    };
    state->active = 1U;
    state->active_index = index;
    state->active_token = candidate->generation;
    ++state->acquired;
    return WL_OK;
  }
  return WL_ERR_NO_DATA;
}

wl_err_t wl_outbox_complete(wl_outbox_t *outbox,
                            const wl_outbox_item_t *item,
                            wl_outbox_completion_t completion) {
  wl_outbox_impl_t *state;
  wl_outbox_slot_impl_t *current;

  if (!initialized(outbox)) {
    return outbox == NULL ? WL_ERR_INVALID_ARG : WL_ERR_NOT_INITIALIZED;
  }
  state = impl(outbox);
  if (item == NULL || item->slot_index >= state->slot_count ||
      (completion != WL_OUTBOX_ACCEPTED &&
       completion != WL_OUTBOX_DEFERRED &&
       completion != WL_OUTBOX_REJECTED)) {
    return WL_ERR_INVALID_ARG;
  }
  if (state->active == 0U || state->active_index != item->slot_index ||
      state->active_token != item->token) {
    return WL_ERR_INVALID_STATE;
  }
  state->active = 0U;
  state->active_token = 0U;
  if (completion == WL_OUTBOX_DEFERRED) {
    ++state->deferred;
    return WL_OK;
  }

  current = slot(state, item->slot_index);
  if (current->valid != 0U && current->generation == item->token) {
    memset(current, 0, sizeof(*current));
    --state->depth;
  } else {
    ++state->superseded;
  }
  state->cursor = (uint16_t)((item->slot_index + 1U) % state->slot_count);
  if (completion == WL_OUTBOX_ACCEPTED) {
    ++state->accepted;
  } else {
    ++state->rejected;
  }
  return WL_OK;
}

wl_err_t wl_outbox_reset(wl_outbox_t *outbox, uint16_t *out_discarded) {
  wl_outbox_impl_t *state;

  if (!initialized(outbox)) {
    return outbox == NULL ? WL_ERR_INVALID_ARG : WL_ERR_NOT_INITIALIZED;
  }
  state = impl(outbox);
  if (state->active != 0U) {
    return WL_ERR_BUSY;
  }
  if (out_discarded != NULL) {
    *out_discarded = state->depth;
  }
  memset(state->slots, 0,
         (size_t)state->slot_count * sizeof(state->slots[0]));
  state->depth = 0U;
  state->cursor = 0U;
  ++state->resets;
  return WL_OK;
}

wl_err_t wl_outbox_get_stats(const wl_outbox_t *outbox,
                             wl_outbox_stats_t *out_stats) {
  const wl_outbox_impl_t *state;

  if (!initialized(outbox)) {
    return outbox == NULL ? WL_ERR_INVALID_ARG : WL_ERR_NOT_INITIALIZED;
  }
  if (out_stats == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  state = impl_const(outbox);
  *out_stats = (wl_outbox_stats_t){
      .submitted = state->submitted,
      .coalesced = state->coalesced,
      .queue_full = state->queue_full,
      .acquired = state->acquired,
      .accepted = state->accepted,
      .deferred = state->deferred,
      .rejected = state->rejected,
      .superseded = state->superseded,
      .resets = state->resets,
      .depth = state->depth,
  };
  return WL_OK;
}
