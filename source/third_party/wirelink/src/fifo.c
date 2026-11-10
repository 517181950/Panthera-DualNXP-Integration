/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/fifo.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define WL_FIFO_MAGIC UINT32_C(0x574c4651)
#define WL_FIFO_MAX_CAPACITY UINT32_C(0x80000000)

typedef struct wl_fifo_impl {
  uint32_t magic;
  uint8_t *storage;
  size_t value_size;
  size_t slot_stride;
  uint32_t capacity;

  /* write_cursor/index and write_* belong only to the single producer. */
  uint32_t write_cursor;
  uint32_t write_index;
  uint32_t write_token;
  bool write_claimed;

  /* read_cursor/index and read_* belong only to the single consumer. */
  uint32_t read_cursor;
  uint32_t read_index;
  uint32_t read_token;
  bool read_borrowed;

  /* These cursors transfer slot ownership between the SPSC roles. */
  _Atomic uint32_t published_cursor;
  _Atomic uint32_t consumed_cursor;

  _Atomic uint32_t high_watermark;
  _Atomic uint32_t publishes;
  _Atomic uint32_t consumes;
  _Atomic uint32_t full_rejections;
  _Atomic uint32_t empty_reads;
  _Atomic uint32_t aborts;
  _Atomic uint32_t resets;
  _Atomic uint32_t errors;
} wl_fifo_impl_t;

_Static_assert(sizeof(wl_fifo_impl_t) <= WL_FIFO_CONTEXT_STORAGE_SIZE,
               "WL_FIFO_CONTEXT_STORAGE_SIZE is too small");
_Static_assert(_Alignof(wl_fifo_impl_t) <= _Alignof(wl_fifo_t),
               "wl_fifo_t alignment is too small");

static wl_fifo_impl_t *wl_fifo_impl(wl_fifo_t *fifo) {
  return (wl_fifo_impl_t *)(void *)fifo;
}

static const wl_fifo_impl_t *wl_fifo_impl_const(const wl_fifo_t *fifo) {
  return (const wl_fifo_impl_t *)(const void *)fifo;
}

static bool wl_fifo_initialized(const wl_fifo_impl_t *impl) {
  return impl->magic == WL_FIFO_MAGIC;
}

static void *wl_fifo_slot(const wl_fifo_impl_t *impl, uint32_t index) {
  return impl->storage + ((size_t)index * impl->slot_stride);
}

static uint32_t wl_fifo_next_token(uint32_t token) {
  token++;
  if (token == 0U) {
    token = 1U;
  }
  return token;
}

static uint32_t wl_fifo_next_index(uint32_t index, uint32_t capacity) {
  index++;
  if (index == capacity) {
    index = 0U;
  }
  return index;
}

static void wl_fifo_counter_increment(_Atomic uint32_t *counter) {
  uint32_t current = atomic_load_explicit(counter, memory_order_relaxed);

  while (current != UINT32_MAX &&
         !atomic_compare_exchange_weak_explicit(counter, &current, current + 1U,
                                                memory_order_relaxed,
                                                memory_order_relaxed)) {
  }
}

static void wl_fifo_counter_max(_Atomic uint32_t *counter, uint32_t value) {
  uint32_t current = atomic_load_explicit(counter, memory_order_relaxed);

  while (current < value && !atomic_compare_exchange_weak_explicit(
                                counter, &current, value, memory_order_relaxed,
                                memory_order_relaxed)) {
  }
}

static int wl_fifo_error(wl_fifo_impl_t *impl, int error) {
  wl_fifo_counter_increment(&impl->errors);
  return error;
}

static bool wl_fifo_claim_matches(const wl_fifo_impl_t *impl,
                                  const wl_fifo_write_claim_t *claim) {
  return claim != NULL && impl->write_claimed && claim->token != 0U &&
         claim->token == impl->write_token &&
         claim->private_slot == impl->write_index &&
         claim->value == wl_fifo_slot(impl, impl->write_index) &&
         claim->value_size == impl->value_size;
}

static bool wl_fifo_view_matches(const wl_fifo_impl_t *impl,
                                 const wl_fifo_view_t *view) {
  return view != NULL && impl->read_borrowed && view->token != 0U &&
         view->token == impl->read_token &&
         view->private_slot == impl->read_index &&
         view->value == wl_fifo_slot(impl, impl->read_index) &&
         view->value_size == impl->value_size;
}

static bool wl_fifo_atomics_are_lock_free(const wl_fifo_impl_t *impl) {
  return atomic_is_lock_free(&impl->published_cursor) &&
         atomic_is_lock_free(&impl->consumed_cursor) &&
         atomic_is_lock_free(&impl->high_watermark) &&
         atomic_is_lock_free(&impl->publishes) &&
         atomic_is_lock_free(&impl->consumes) &&
         atomic_is_lock_free(&impl->full_rejections) &&
         atomic_is_lock_free(&impl->empty_reads) &&
         atomic_is_lock_free(&impl->aborts) &&
         atomic_is_lock_free(&impl->resets) &&
         atomic_is_lock_free(&impl->errors);
}

int wl_fifo_requirements(const wl_fifo_config_t *config,
                         wl_fifo_requirements_t *out_requirements) {
  size_t alignment;
  size_t stride;

  if (out_requirements == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  *out_requirements = (wl_fifo_requirements_t){0};
  if (config == NULL || config->value_size == 0U ||
      config->value_alignment == 0U || config->capacity == 0U ||
      config->capacity >= WL_FIFO_MAX_CAPACITY) {
    return WL_ERR_INVALID_ARG;
  }

  alignment = config->value_alignment;
  if ((alignment & (alignment - 1U)) != 0U || alignment > (size_t)UINTPTR_MAX ||
      config->value_size > SIZE_MAX - (alignment - 1U)) {
    return WL_ERR_INVALID_ARG;
  }

  stride = (config->value_size + (alignment - 1U)) & ~(alignment - 1U);
  if (stride > SIZE_MAX / config->capacity) {
    return WL_ERR_INVALID_ARG;
  }

  out_requirements->storage_size = stride * config->capacity;
  out_requirements->slot_stride = stride;
  out_requirements->slot_count = config->capacity;
  return WL_OK;
}

int wl_fifo_init(wl_fifo_t *fifo, const wl_fifo_config_t *config,
                 const wl_fifo_storage_t *storage) {
  wl_fifo_config_t config_copy;
  wl_fifo_requirements_t requirements;
  wl_fifo_storage_t storage_copy;
  wl_fifo_impl_t *impl;
  uintptr_t fifo_address;
  uintptr_t storage_address;
  int result;

  if (fifo == NULL || config == NULL || storage == NULL ||
      storage->data == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  config_copy = *config;
  storage_copy = *storage;
  result = wl_fifo_requirements(&config_copy, &requirements);
  if (result != WL_OK) {
    return result;
  }
  if (storage_copy.size < requirements.storage_size) {
    return WL_ERR_BUF_TOO_SMALL;
  }

  storage_address = (uintptr_t)storage_copy.data;
  fifo_address = (uintptr_t)fifo;
  if ((storage_address & (config_copy.value_alignment - 1U)) != 0U) {
    return WL_ERR_INVALID_ARG;
  }
  if ((storage_address <= fifo_address &&
       fifo_address - storage_address < requirements.storage_size) ||
      (fifo_address < storage_address &&
       storage_address - fifo_address < sizeof(*fifo))) {
    return WL_ERR_INVALID_ARG;
  }

  memset(fifo, 0, sizeof(*fifo));
  impl = wl_fifo_impl(fifo);
  impl->storage = storage_copy.data;
  impl->value_size = config_copy.value_size;
  impl->slot_stride = requirements.slot_stride;
  impl->capacity = config_copy.capacity;

  atomic_init(&impl->published_cursor, 0U);
  atomic_init(&impl->consumed_cursor, 0U);
  atomic_init(&impl->high_watermark, 0U);
  atomic_init(&impl->publishes, 0U);
  atomic_init(&impl->consumes, 0U);
  atomic_init(&impl->full_rejections, 0U);
  atomic_init(&impl->empty_reads, 0U);
  atomic_init(&impl->aborts, 0U);
  atomic_init(&impl->resets, 0U);
  atomic_init(&impl->errors, 0U);
  if (!wl_fifo_atomics_are_lock_free(impl)) {
    return WL_ERR_NOT_SUPPORTED;
  }

  impl->magic = WL_FIFO_MAGIC;
  return WL_OK;
}

int wl_fifo_write_claim(wl_fifo_t *fifo, wl_fifo_write_claim_t *out_claim) {
  wl_fifo_impl_t *impl;
  uint32_t consumed;

  if (out_claim != NULL) {
    *out_claim = (wl_fifo_write_claim_t){0};
  }
  if (fifo == NULL || out_claim == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (impl->write_claimed) {
    return wl_fifo_error(impl, WL_ERR_BUSY);
  }

  consumed = atomic_load_explicit(&impl->consumed_cursor, memory_order_acquire);
  if ((uint32_t)(impl->write_cursor - consumed) >= impl->capacity) {
    wl_fifo_counter_increment(&impl->full_rejections);
    return WL_ERR_QUEUE_FULL;
  }

  impl->write_token = wl_fifo_next_token(impl->write_token);
  impl->write_claimed = true;
  out_claim->value = wl_fifo_slot(impl, impl->write_index);
  out_claim->value_size = impl->value_size;
  out_claim->token = impl->write_token;
  out_claim->private_slot = impl->write_index;
  return WL_OK;
}

int wl_fifo_write_publish(wl_fifo_t *fifo, const wl_fifo_write_claim_t *claim) {
  wl_fifo_impl_t *impl;
  uint32_t consumed;
  uint32_t next_cursor;
  uint32_t depth;

  if (fifo == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!wl_fifo_claim_matches(impl, claim)) {
    return wl_fifo_error(impl, WL_ERR_INVALID_STATE);
  }

  next_cursor = impl->write_cursor + 1U;
  consumed = atomic_load_explicit(&impl->consumed_cursor, memory_order_acquire);
  depth = (uint32_t)(next_cursor - consumed);
  impl->write_cursor = next_cursor;
  impl->write_index = wl_fifo_next_index(impl->write_index, impl->capacity);
  impl->write_claimed = false;
  atomic_store_explicit(&impl->published_cursor, next_cursor,
                        memory_order_release);

  wl_fifo_counter_max(&impl->high_watermark, depth);
  wl_fifo_counter_increment(&impl->publishes);
  return WL_OK;
}

int wl_fifo_write_abort(wl_fifo_t *fifo, const wl_fifo_write_claim_t *claim) {
  wl_fifo_impl_t *impl;

  if (fifo == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!wl_fifo_claim_matches(impl, claim)) {
    return wl_fifo_error(impl, WL_ERR_INVALID_STATE);
  }

  impl->write_claimed = false;
  wl_fifo_counter_increment(&impl->aborts);
  return WL_OK;
}

int wl_fifo_read_acquire(wl_fifo_t *fifo, wl_fifo_view_t *out_view) {
  wl_fifo_impl_t *impl;
  uint32_t published;

  if (out_view != NULL) {
    *out_view = (wl_fifo_view_t){0};
  }
  if (fifo == NULL || out_view == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (impl->read_borrowed) {
    return wl_fifo_error(impl, WL_ERR_BUSY);
  }

  published =
      atomic_load_explicit(&impl->published_cursor, memory_order_acquire);
  if (impl->read_cursor == published) {
    wl_fifo_counter_increment(&impl->empty_reads);
    return WL_ERR_NO_DATA;
  }

  impl->read_token = wl_fifo_next_token(impl->read_token);
  impl->read_borrowed = true;
  out_view->value = wl_fifo_slot(impl, impl->read_index);
  out_view->value_size = impl->value_size;
  out_view->token = impl->read_token;
  out_view->private_slot = impl->read_index;
  return WL_OK;
}

int wl_fifo_read_release(wl_fifo_t *fifo, const wl_fifo_view_t *view) {
  wl_fifo_impl_t *impl;

  if (fifo == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (!wl_fifo_view_matches(impl, view)) {
    return wl_fifo_error(impl, WL_ERR_INVALID_STATE);
  }

  impl->read_cursor++;
  impl->read_index = wl_fifo_next_index(impl->read_index, impl->capacity);
  impl->read_borrowed = false;
  atomic_store_explicit(&impl->consumed_cursor, impl->read_cursor,
                        memory_order_release);
  wl_fifo_counter_increment(&impl->consumes);
  return WL_OK;
}

int wl_fifo_reset(wl_fifo_t *fifo) {
  wl_fifo_impl_t *impl;

  if (fifo == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (impl->write_claimed || impl->read_borrowed) {
    return wl_fifo_error(impl, WL_ERR_BUSY);
  }

  impl->write_cursor = 0U;
  impl->write_index = 0U;
  impl->read_cursor = 0U;
  impl->read_index = 0U;
  atomic_store_explicit(&impl->published_cursor, 0U, memory_order_release);
  atomic_store_explicit(&impl->consumed_cursor, 0U, memory_order_release);
  wl_fifo_counter_increment(&impl->resets);
  return WL_OK;
}

int wl_fifo_get_stats(const wl_fifo_t *fifo, wl_fifo_stats_t *out_stats) {
  const wl_fifo_impl_t *impl;
  uint32_t consumed;
  uint32_t published;
  uint32_t depth;

  if (out_stats != NULL) {
    *out_stats = (wl_fifo_stats_t){0};
  }
  if (fifo == NULL || out_stats == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  impl = wl_fifo_impl_const(fifo);
  if (!wl_fifo_initialized(impl)) {
    return WL_ERR_NOT_INITIALIZED;
  }

  /* Acquire the reclaimed cursor first.  Its publication follows the read of
   * a corresponding published cursor, keeping this modular depth bounded for
   * conforming SPSC callers even while either role is active. */
  consumed = atomic_load_explicit(&impl->consumed_cursor, memory_order_acquire);
  published =
      atomic_load_explicit(&impl->published_cursor, memory_order_acquire);
  depth = (uint32_t)(published - consumed);
  if (depth > impl->capacity) {
    depth = impl->capacity;
  }

  out_stats->depth = depth;
  out_stats->high_watermark =
      atomic_load_explicit(&impl->high_watermark, memory_order_relaxed);
  out_stats->publishes =
      atomic_load_explicit(&impl->publishes, memory_order_relaxed);
  out_stats->consumes =
      atomic_load_explicit(&impl->consumes, memory_order_relaxed);
  out_stats->full_rejections =
      atomic_load_explicit(&impl->full_rejections, memory_order_relaxed);
  out_stats->empty_reads =
      atomic_load_explicit(&impl->empty_reads, memory_order_relaxed);
  out_stats->aborts = atomic_load_explicit(&impl->aborts, memory_order_relaxed);
  out_stats->resets = atomic_load_explicit(&impl->resets, memory_order_relaxed);
  out_stats->errors = atomic_load_explicit(&impl->errors, memory_order_relaxed);
  return WL_OK;
}

#ifdef WL_FIFO_TEST_HOOKS
void wl_fifo_test_seed_saturating_counters(wl_fifo_t *fifo,
                                           const wl_fifo_stats_t *stats);
void wl_fifo_test_seed_cursors(wl_fifo_t *fifo, uint32_t cursor);

void wl_fifo_test_seed_saturating_counters(wl_fifo_t *fifo,
                                           const wl_fifo_stats_t *stats) {
  wl_fifo_impl_t *impl = wl_fifo_impl(fifo);

  atomic_store_explicit(&impl->high_watermark, stats->high_watermark,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->publishes, stats->publishes,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->consumes, stats->consumes, memory_order_relaxed);
  atomic_store_explicit(&impl->full_rejections, stats->full_rejections,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->empty_reads, stats->empty_reads,
                        memory_order_relaxed);
  atomic_store_explicit(&impl->aborts, stats->aborts, memory_order_relaxed);
  atomic_store_explicit(&impl->resets, stats->resets, memory_order_relaxed);
  atomic_store_explicit(&impl->errors, stats->errors, memory_order_relaxed);
}

void wl_fifo_test_seed_cursors(wl_fifo_t *fifo, uint32_t cursor) {
  wl_fifo_impl_t *impl = wl_fifo_impl(fifo);
  uint32_t index = cursor % impl->capacity;

  impl->write_cursor = cursor;
  impl->write_index = index;
  impl->read_cursor = cursor;
  impl->read_index = index;
  atomic_store_explicit(&impl->published_cursor, cursor, memory_order_relaxed);
  atomic_store_explicit(&impl->consumed_cursor, cursor, memory_order_relaxed);
}
#endif
