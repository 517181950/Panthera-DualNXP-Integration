/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/latest.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define WL_LATEST_MAGIC UINT32_C(0x574c4c54)
#define WL_LATEST_DIRTY UINT32_C(0x80000000)
#define WL_LATEST_INDEX_MASK UINT32_C(0x00000003)

typedef struct wl_latest_impl {
  uint32_t magic;
  uint8_t *storage;
  size_t value_size;
  size_t slot_stride;

  /* writer_back and write_* are owned only by the single producer. */
  uint32_t writer_back;
  uint32_t write_token;
  bool write_claimed;

  /* reader_front and read_* are owned only by the single consumer. */
  uint32_t reader_front;
  uint32_t read_token;
  bool read_borrowed;

  uint32_t slot_generation[WL_LATEST_SLOT_COUNT];

  /* Atomic middle transfers exclusive slot ownership between the roles. */
  _Atomic uint32_t middle;
  _Atomic uint32_t generation;
  _Atomic uint32_t publishes;
  _Atomic uint32_t reads;
  _Atomic uint32_t coalesced;
  _Atomic uint32_t empty_reads;
  _Atomic uint32_t resets;
  _Atomic uint32_t errors;
} wl_latest_impl_t;

_Static_assert(sizeof(wl_latest_impl_t) <= WL_LATEST_CONTEXT_STORAGE_SIZE,
               "WL_LATEST_CONTEXT_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_latest_impl_t) <= _Alignof(wl_latest_t),
               "wl_latest_t alignment is too small");

static wl_latest_impl_t *wl_latest_impl(wl_latest_t *mailbox) {
  return (wl_latest_impl_t *)(void *)mailbox;
}

static const wl_latest_impl_t *
wl_latest_impl_const(const wl_latest_t *mailbox) {
  return (const wl_latest_impl_t *)(const void *)mailbox;
}

static bool wl_latest_initialized(const wl_latest_impl_t *impl) {
  return impl->magic == WL_LATEST_MAGIC;
}

static void *wl_latest_slot(const wl_latest_impl_t *impl, uint32_t index) {
  return impl->storage + ((size_t)index * impl->slot_stride);
}

static void wl_latest_counter_increment(_Atomic uint32_t *counter) {
  uint32_t current = atomic_load_explicit(counter, memory_order_relaxed);

  while (current != UINT32_MAX &&
         !atomic_compare_exchange_weak_explicit(counter, &current, current + 1U,
                                                memory_order_relaxed,
                                                memory_order_relaxed)) {
  }
}

static int wl_latest_error(wl_latest_impl_t *impl, int error) {
  wl_latest_counter_increment(&impl->errors);
  return error;
}

static uint32_t wl_latest_next_token(uint32_t token) {
  token++;
  if (token == 0U) {
    token = 1U;
  }
  return token;
}

static bool wl_latest_claim_matches(const wl_latest_impl_t *impl,
                                    const wl_latest_write_claim_t *claim) {
  return claim != NULL && impl->write_claimed && claim->token != 0U &&
         claim->token == impl->write_token &&
         claim->private_slot == impl->writer_back &&
         claim->value == wl_latest_slot(impl, impl->writer_back) &&
         claim->value_size == impl->value_size;
}

static bool wl_latest_view_matches(const wl_latest_impl_t *impl,
                                   const wl_latest_view_t *view) {
  return view != NULL && impl->read_borrowed && view->token != 0U &&
         view->token == impl->read_token &&
         view->private_slot == impl->reader_front &&
         view->value == wl_latest_slot(impl, impl->reader_front) &&
         view->value_size == impl->value_size &&
         view->generation == impl->slot_generation[impl->reader_front];
}

static bool wl_latest_atomics_are_lock_free(const wl_latest_impl_t *impl) {
  return atomic_is_lock_free(&impl->middle) &&
         atomic_is_lock_free(&impl->generation) &&
         atomic_is_lock_free(&impl->publishes) &&
         atomic_is_lock_free(&impl->reads) &&
         atomic_is_lock_free(&impl->coalesced) &&
         atomic_is_lock_free(&impl->empty_reads) &&
         atomic_is_lock_free(&impl->resets) &&
         atomic_is_lock_free(&impl->errors);
}

int wl_latest_requirements(const wl_latest_config_t *config,
                           wl_latest_requirements_t *out_requirements) {
  size_t stride;
  size_t alignment;

  if (out_requirements == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  *out_requirements = (wl_latest_requirements_t){0};
  if (config == NULL || config->value_size == 0U ||
      config->value_alignment == 0U) {
    return WL_ERR_INVALID_ARG;
  }

  alignment = config->value_alignment;
  if ((alignment & (alignment - 1U)) != 0U ||
      config->value_size > SIZE_MAX - (alignment - 1U)) {
    return WL_ERR_INVALID_ARG;
  }

  stride = (config->value_size + (alignment - 1U)) & ~(alignment - 1U);
  if (stride > SIZE_MAX / WL_LATEST_SLOT_COUNT) {
    return WL_ERR_INVALID_ARG;
  }

  out_requirements->storage_size = stride * WL_LATEST_SLOT_COUNT;
  out_requirements->slot_stride = stride;
  out_requirements->slot_count = WL_LATEST_SLOT_COUNT;
  return WL_OK;
}

int wl_latest_init(wl_latest_t *mailbox, const wl_latest_config_t *config,
                   const wl_latest_storage_t *storage) {
  wl_latest_config_t config_copy;
  wl_latest_requirements_t requirements;
  wl_latest_storage_t storage_copy;
  wl_latest_impl_t *impl;
  uintptr_t mailbox_address;
  uintptr_t storage_address;
  int result;

  if (mailbox == NULL || config == NULL || storage == NULL ||
      storage->data == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  config_copy = *config;
  storage_copy = *storage;
  result = wl_latest_requirements(&config_copy, &requirements);
  if (result != WL_OK) {
    return result;
  }
  if (storage_copy.size < requirements.storage_size) {
    return WL_ERR_BUF_TOO_SMALL;
  }
  storage_address = (uintptr_t)storage_copy.data;
  mailbox_address = (uintptr_t)mailbox;
  if ((storage_address & (config_copy.value_alignment - 1U)) != 0U) {
    return WL_ERR_INVALID_ARG;
  }
  if ((storage_address <= mailbox_address &&
       mailbox_address - storage_address < requirements.storage_size) ||
      (mailbox_address < storage_address &&
       storage_address - mailbox_address < sizeof(*mailbox))) {
    return WL_ERR_INVALID_ARG;
  }

  memset(mailbox, 0, sizeof(*mailbox));
  impl = wl_latest_impl(mailbox);
  impl->storage = storage_copy.data;
  impl->value_size = config_copy.value_size;
  impl->slot_stride = requirements.slot_stride;
  impl->reader_front = 0U;
  impl->writer_back = 2U;

  atomic_init(&impl->middle, 1U);
  atomic_init(&impl->generation, config_copy.initial_generation);
  atomic_init(&impl->publishes, 0U);
  atomic_init(&impl->reads, 0U);
  atomic_init(&impl->coalesced, 0U);
  atomic_init(&impl->empty_reads, 0U);
  atomic_init(&impl->resets, 0U);
  atomic_init(&impl->errors, 0U);
  if (!wl_latest_atomics_are_lock_free(impl)) {
    return WL_ERR_NOT_SUPPORTED;
  }

  impl->magic = WL_LATEST_MAGIC;
  return WL_OK;
}

int wl_latest_write_claim(wl_latest_t *mailbox,
                          wl_latest_write_claim_t *out_claim) {
  wl_latest_impl_t *impl;

  if (out_claim != NULL) {
    *out_claim = (wl_latest_write_claim_t){0};
  }
  if (mailbox == NULL || out_claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (impl->write_claimed) {
    return wl_latest_error(impl, WL_ERR_BUSY);
  }

  impl->write_token = wl_latest_next_token(impl->write_token);
  impl->write_claimed = true;
  out_claim->value = wl_latest_slot(impl, impl->writer_back);
  out_claim->value_size = impl->value_size;
  out_claim->token = impl->write_token;
  out_claim->private_slot = impl->writer_back;
  return WL_OK;
}

int wl_latest_write_publish(wl_latest_t *mailbox,
                            const wl_latest_write_claim_t *claim) {
  wl_latest_impl_t *impl;
  uint32_t old_middle;
  uint32_t generation;

  if (mailbox == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!wl_latest_claim_matches(impl, claim)) {
    return wl_latest_error(impl, WL_ERR_INVALID_STATE);
  }

  generation =
      atomic_fetch_add_explicit(&impl->generation, 1U, memory_order_relaxed) +
      1U;
  impl->slot_generation[impl->writer_back] = generation;
  old_middle = atomic_exchange_explicit(
      &impl->middle, impl->writer_back | WL_LATEST_DIRTY, memory_order_acq_rel);
  impl->writer_back = old_middle & WL_LATEST_INDEX_MASK;
  impl->write_claimed = false;

  wl_latest_counter_increment(&impl->publishes);
  if ((old_middle & WL_LATEST_DIRTY) != 0U) {
    wl_latest_counter_increment(&impl->coalesced);
  }
  return WL_OK;
}

int wl_latest_write_abort(wl_latest_t *mailbox,
                          const wl_latest_write_claim_t *claim) {
  wl_latest_impl_t *impl;

  if (mailbox == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!wl_latest_claim_matches(impl, claim)) {
    return wl_latest_error(impl, WL_ERR_INVALID_STATE);
  }

  impl->write_claimed = false;
  return WL_OK;
}

int wl_latest_read_acquire(wl_latest_t *mailbox, wl_latest_view_t *out_view) {
  wl_latest_impl_t *impl;
  uint32_t middle;

  if (out_view != NULL) {
    *out_view = (wl_latest_view_t){0};
  }
  if (mailbox == NULL || out_view == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (impl->read_borrowed) {
    return wl_latest_error(impl, WL_ERR_BUSY);
  }

  middle = atomic_load_explicit(&impl->middle, memory_order_acquire);
  if ((middle & WL_LATEST_DIRTY) == 0U) {
    wl_latest_counter_increment(&impl->empty_reads);
    return WL_ERR_NO_DATA;
  }

  middle = atomic_exchange_explicit(&impl->middle, impl->reader_front,
                                    memory_order_acq_rel);
  impl->reader_front = middle & WL_LATEST_INDEX_MASK;
  impl->read_token = wl_latest_next_token(impl->read_token);
  impl->read_borrowed = true;

  out_view->value = wl_latest_slot(impl, impl->reader_front);
  out_view->value_size = impl->value_size;
  out_view->generation = impl->slot_generation[impl->reader_front];
  out_view->token = impl->read_token;
  out_view->private_slot = impl->reader_front;
  wl_latest_counter_increment(&impl->reads);
  return WL_OK;
}

int wl_latest_read_release(wl_latest_t *mailbox, const wl_latest_view_t *view) {
  wl_latest_impl_t *impl;

  if (mailbox == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!wl_latest_view_matches(impl, view)) {
    return wl_latest_error(impl, WL_ERR_INVALID_STATE);
  }

  impl->read_borrowed = false;
  return WL_OK;
}

int wl_latest_reset(wl_latest_t *mailbox) {
  wl_latest_impl_t *impl;

  if (mailbox == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (impl->write_claimed || impl->read_borrowed) {
    return wl_latest_error(impl, WL_ERR_BUSY);
  }

  impl->reader_front = 0U;
  impl->writer_back = 2U;
  (void)atomic_exchange_explicit(&impl->middle, 1U, memory_order_acq_rel);
  wl_latest_counter_increment(&impl->resets);
  return WL_OK;
}

int wl_latest_get_stats(const wl_latest_t *mailbox,
                        wl_latest_stats_t *out_stats) {
  const wl_latest_impl_t *impl;

  if (out_stats != NULL) {
    *out_stats = (wl_latest_stats_t){0};
  }
  if (mailbox == NULL || out_stats == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_latest_impl_const(mailbox);
  if (!wl_latest_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }

  out_stats->generation =
      atomic_load_explicit(&impl->generation, memory_order_relaxed);
  out_stats->publishes =
      atomic_load_explicit(&impl->publishes, memory_order_relaxed);
  out_stats->reads = atomic_load_explicit(&impl->reads, memory_order_relaxed);
  out_stats->coalesced =
      atomic_load_explicit(&impl->coalesced, memory_order_relaxed);
  out_stats->empty_reads =
      atomic_load_explicit(&impl->empty_reads, memory_order_relaxed);
  out_stats->resets = atomic_load_explicit(&impl->resets, memory_order_relaxed);
  out_stats->errors = atomic_load_explicit(&impl->errors, memory_order_relaxed);
  return WL_OK;
}

#ifdef WL_LATEST_TEST_HOOKS
void wl_latest_test_seed_saturating_counters(wl_latest_t *mailbox,
                                             const wl_latest_stats_t *stats);

void wl_latest_test_seed_saturating_counters(wl_latest_t *mailbox,
                                             const wl_latest_stats_t *stats) {
  wl_latest_impl_t *impl = wl_latest_impl(mailbox);

  atomic_store_explicit(&impl->publishes, stats->publishes,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->reads, stats->reads, memory_order_relaxed);
  atomic_store_explicit(&impl->coalesced, stats->coalesced,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->empty_reads, stats->empty_reads,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->resets, stats->resets, memory_order_relaxed);
  atomic_store_explicit(&impl->errors, stats->errors, memory_order_relaxed);
}
#endif
