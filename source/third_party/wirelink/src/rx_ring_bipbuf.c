/* SPDX-License-Identifier: Apache-2.0 */

#include "rx_ring.h"

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "wirelink/types.h"

/*
 * RX-only SPSC BipBuffer.
 *
 * The producer advances write_cursor and the consumer advances read_cursor.
 * The sole exception is direct-DMA normalization of an observed-empty ring;
 * it atomically moves both cursors to an equivalent physical-zero epoch. The
 * snapshot order below makes that transition appear empty or temporarily
 * invalid, never readable. Consequently the byte stores preceding a producer
 * commit are visible to the consumer, and a producer cannot reuse bytes until
 * the consumer publishes a consume.
 *
 * Each cursor ranges over two buffer lengths.  The extra epoch distinguishes
 * a full buffer from an empty one, while bounding cursor arithmetic so it
 * remains correct after rollover for capacities that are not powers of two.
 * All memory_size bytes are therefore usable.  A reservation or peek never
 * crosses the physical end of the backing memory; a caller that needs more
 * data obtains the second contiguous region with its next operation.
 */
typedef struct {
  size_t cursor;
  size_t length;
  size_t published;
  uint32_t token;
  uint8_t active;
} wl_rx_dma_claim_state_t;

typedef struct {
  uint8_t *memory;
  size_t capacity;
  _Atomic size_t read_cursor;
  _Atomic size_t write_cursor;
  _Atomic unsigned int overflow_events;

  /* Producer-private reservation state. */
  size_t reservation_cursor;
  size_t reservation_length;
  uint8_t reservation_active;

  /* Producer-private direct-DMA claim state. */
  uint32_t dma_next_token;
  wl_rx_dma_claim_state_t dma_claim;
} wl_rx_bipbuf_state_t;

_Static_assert(sizeof(wl_rx_bipbuf_state_t) <= WL_RX_RING_STATE_SIZE,
               "WL_RX_RING_STATE_SIZE is too small for the BipBuffer backend");

static wl_rx_bipbuf_state_t *rx_state(wl_rx_ring_state_t *state) {
  return (wl_rx_bipbuf_state_t *)(void *)state->bytes;
}

static const wl_rx_bipbuf_state_t *rx_state_const(
    const wl_rx_ring_state_t *state) {
  return (const wl_rx_bipbuf_state_t *)(const void *)state->bytes;
}

static size_t cursor_physical(size_t capacity, size_t cursor) {
  return cursor < capacity ? cursor : cursor - capacity;
}

static size_t cursor_distance(size_t period, size_t from, size_t to) {
  return to >= from ? to - from : period - from + to;
}

static size_t cursor_advance(size_t period, size_t cursor, size_t length) {
  return length >= period - cursor ? length - (period - cursor)
                                   : cursor + length;
}

static size_t contiguous_length(size_t capacity, size_t cursor,
                                size_t available) {
  const size_t to_end = capacity - cursor_physical(capacity, cursor);
  return available < to_end ? available : to_end;
}

static wl_rx_dma_claim_state_t *find_dma_claim(
    wl_rx_bipbuf_state_t *backend, uint32_t token) {
  return backend->dma_claim.active != 0U && backend->dma_claim.token == token
             ? &backend->dma_claim
             : NULL;
}

static size_t readable_snapshot(const wl_rx_bipbuf_state_t *backend,
                                size_t *out_read_cursor) {
  const size_t write_cursor =
      atomic_load_explicit(&backend->write_cursor, memory_order_acquire);
  const size_t read_cursor =
      atomic_load_explicit(&backend->read_cursor, memory_order_relaxed);
  const size_t readable =
      cursor_distance(backend->capacity * 2U, read_cursor, write_cursor);

  if (out_read_cursor != NULL) {
    *out_read_cursor = read_cursor;
  }
  return readable <= backend->capacity ? readable : 0U;
}

size_t wl_rx_ring_storage_size(size_t usable_capacity) {
  return usable_capacity;
}

int wl_rx_ring_init(wl_rx_ring_state_t *state, uint8_t *memory,
                    size_t memory_size) {
  wl_rx_bipbuf_state_t *backend;

  if (state == NULL || memory == NULL || memory_size == 0U ||
      memory_size > SIZE_MAX / 2U) {
    return WL_ERR_INVALID_ARG;
  }

  memset(state, 0, sizeof(*state));
  backend = rx_state(state);
  backend->memory = memory;
  backend->capacity = memory_size;
  atomic_init(&backend->read_cursor, 0U);
  atomic_init(&backend->write_cursor, 0U);
  atomic_init(&backend->overflow_events, 0U);
  backend->dma_next_token = 1U;

  if (!atomic_is_lock_free(&backend->read_cursor) ||
      !atomic_is_lock_free(&backend->write_cursor) ||
      !atomic_is_lock_free(&backend->overflow_events)) {
    return WL_ERR_NOT_SUPPORTED;
  }
  return WL_OK;
}

void wl_rx_ring_producer_note_overflow(wl_rx_ring_state_t *state) {
  wl_rx_bipbuf_state_t *backend;

  if (state == NULL) {
    return;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return;
  }
  (void)atomic_fetch_add_explicit(&backend->overflow_events, 1U,
                                  memory_order_release);
}

int wl_rx_ring_producer_reserve(wl_rx_ring_state_t *state,
                                wl_span_t *out_span) {
  wl_rx_bipbuf_state_t *backend;
  size_t read_cursor;
  size_t write_cursor;
  size_t readable;
  size_t free_space;

  if (state == NULL || out_span == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (backend->reservation_active != 0U ||
      backend->dma_claim.active != 0U) {
    return WL_ERR_INVALID_STATE;
  }

  write_cursor =
      atomic_load_explicit(&backend->write_cursor, memory_order_relaxed);
  read_cursor =
      atomic_load_explicit(&backend->read_cursor, memory_order_acquire);
  readable = cursor_distance(backend->capacity * 2U, read_cursor,
                             write_cursor);
  if (readable > backend->capacity) {
    return WL_ERR_INVALID_STATE;
  }

  free_space = backend->capacity - readable;
  backend->reservation_cursor = write_cursor;
  backend->reservation_length =
      contiguous_length(backend->capacity, write_cursor, free_space);
  backend->reservation_active = 1U;
  out_span->data =
      backend->memory + cursor_physical(backend->capacity, write_cursor);
  out_span->length = backend->reservation_length;
  return WL_OK;
}

int wl_rx_ring_dma_claim(wl_rx_ring_state_t *state, size_t maximum_length,
                         wl_rx_dma_claim_t *out_claim) {
  wl_rx_bipbuf_state_t *backend;
  wl_rx_dma_claim_state_t *claim;
  size_t read_cursor;
  size_t reserve_cursor;
  size_t reserved;
  size_t free_space;
  size_t length;
  size_t reset_cursor;

  if (state == NULL || out_claim == NULL || maximum_length == 0U) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (backend->reservation_active != 0U ||
      backend->dma_claim.active != 0U) {
    return WL_ERR_WOULD_BLOCK;
  }
  claim = &backend->dma_claim;

  reserve_cursor =
      atomic_load_explicit(&backend->write_cursor, memory_order_relaxed);
  read_cursor = atomic_load_explicit(&backend->read_cursor, memory_order_acquire);
  if (reserve_cursor == read_cursor && maximum_length <= backend->capacity &&
      contiguous_length(backend->capacity, reserve_cursor, backend->capacity) <
          maximum_length) {
    /*
     * Empty rings have no borrowed consumer bytes. Normalize both epochs to
     * offset zero so a DMA claim can use the larger physical A region instead
     * of being artificially limited by the old tail position.
     */
    /* Pick the opposite epoch's physical zero. A consumer that captured the
     * old write cursor before this read-cursor store computes a distance over
     * capacity and treats the snapshot as unavailable. A consumer that sees
     * the following release-store to write_cursor also observes this reset. */
    reset_cursor = reserve_cursor < backend->capacity ? backend->capacity : 0U;
    atomic_store_explicit(&backend->read_cursor, reset_cursor,
                          memory_order_release);
    atomic_store_explicit(&backend->write_cursor, reset_cursor,
                          memory_order_release);
    reserve_cursor = reset_cursor;
    read_cursor = reset_cursor;
  }
  reserved = cursor_distance(backend->capacity * 2U, read_cursor, reserve_cursor);
  if (reserved > backend->capacity) {
    return WL_ERR_INVALID_STATE;
  }
  free_space = backend->capacity - reserved;
  length = contiguous_length(backend->capacity, reserve_cursor, free_space);
  if (length > maximum_length) {
    length = maximum_length;
  }
  if (length == 0U) {
    return WL_ERR_WOULD_BLOCK;
  }

  if (backend->dma_next_token == 0U) {
    ++backend->dma_next_token;
  }
  *claim = (wl_rx_dma_claim_state_t){
      .cursor = reserve_cursor,
      .length = length,
      .published = 0U,
      .token = backend->dma_next_token++,
      .active = 1U,
  };
  out_claim->span = (wl_span_t){
      backend->memory + cursor_physical(backend->capacity, reserve_cursor),
      length};
  out_claim->token = claim->token;
  return WL_OK;
}

int wl_rx_ring_dma_publish(wl_rx_ring_state_t *state,
                           const wl_rx_dma_claim_t *claim, size_t offset,
                           size_t length) {
  wl_rx_bipbuf_state_t *backend;
  wl_rx_dma_claim_state_t *stored;

  if (state == NULL || claim == NULL || claim->token == 0U) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  stored = find_dma_claim(backend, claim->token);
  if (stored == NULL || offset != stored->published ||
      length > stored->length - offset) {
    return WL_ERR_INVALID_STATE;
  }

  stored->published += length;
  atomic_store_explicit(&backend->write_cursor,
                        cursor_advance(backend->capacity * 2U,
                                       stored->cursor, stored->published),
                        memory_order_release);
  return WL_OK;
}

int wl_rx_ring_dma_finish(wl_rx_ring_state_t *state,
                          const wl_rx_dma_claim_t *claim) {
  wl_rx_bipbuf_state_t *backend;
  wl_rx_dma_claim_state_t *stored;

  if (state == NULL || claim == NULL || claim->token == 0U) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  stored = find_dma_claim(backend, claim->token);
  if (stored == NULL) {
    return WL_ERR_INVALID_STATE;
  }
  memset(stored, 0, sizeof(*stored));
  return WL_OK;
}

int wl_rx_ring_dma_abort(wl_rx_ring_state_t *state) {
  wl_rx_bipbuf_state_t *backend;

  if (state == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (backend->reservation_active != 0U) {
    return WL_ERR_INVALID_STATE;
  }
  memset(&backend->dma_claim, 0, sizeof(backend->dma_claim));
  wl_rx_ring_producer_note_overflow(state);
  return WL_OK;
}

int wl_rx_ring_producer_commit(wl_rx_ring_state_t *state, size_t length) {
  wl_rx_bipbuf_state_t *backend;

  if (state == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }
  if (backend->reservation_active == 0U) {
    return WL_ERR_INVALID_STATE;
  }
  if (length > backend->reservation_length) {
    return WL_ERR_INVALID_ARG;
  }

  atomic_store_explicit(&backend->write_cursor,
                        cursor_advance(backend->capacity * 2U,
                                       backend->reservation_cursor, length),
                        memory_order_release);
  backend->reservation_cursor = 0U;
  backend->reservation_length = 0U;
  backend->reservation_active = 0U;
  return WL_OK;
}

size_t wl_rx_ring_readable(const wl_rx_ring_state_t *state) {
  const wl_rx_bipbuf_state_t *backend;

  if (state == NULL) {
    return 0U;
  }
  backend = rx_state_const(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return 0U;
  }
  return readable_snapshot(backend, NULL);
}

int wl_rx_ring_consumer_overflow_pending(const wl_rx_ring_state_t *state) {
  const wl_rx_bipbuf_state_t *backend;

  if (state == NULL) {
    return 0;
  }
  backend = rx_state_const(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return 0;
  }
  return atomic_load_explicit(&backend->overflow_events,
                              memory_order_acquire) != 0U;
}

wl_span_t wl_rx_ring_consumer_peek(wl_rx_ring_state_t *state) {
  wl_rx_bipbuf_state_t *backend;
  size_t read_cursor;
  size_t readable;

  if (state == NULL) {
    return (wl_span_t){NULL, 0U};
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return (wl_span_t){NULL, 0U};
  }

  readable = readable_snapshot(backend, &read_cursor);
  if (readable == 0U) {
    return (wl_span_t){NULL, 0U};
  }
  return (wl_span_t){
      backend->memory + cursor_physical(backend->capacity, read_cursor),
      contiguous_length(backend->capacity, read_cursor, readable)};
}

int wl_rx_ring_consumer_find(const wl_rx_ring_state_t *state, uint8_t value,
                             size_t *out_offset) {
  const wl_rx_bipbuf_state_t *backend;
  size_t read_cursor;
  size_t readable;
  size_t first_length;
  const uint8_t *match;

  if (state == NULL || out_offset == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state_const(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }

  readable = readable_snapshot(backend, &read_cursor);
  first_length = contiguous_length(backend->capacity, read_cursor, readable);
  match = memchr(backend->memory +
                     cursor_physical(backend->capacity, read_cursor),
                 value, first_length);
  if (match != NULL) {
    *out_offset =
        (size_t)(match -
                 (backend->memory +
                  cursor_physical(backend->capacity, read_cursor)));
    return WL_OK;
  }

  if (first_length < readable) {
    const size_t second_length = readable - first_length;
    match = memchr(backend->memory, value, second_length);
    if (match != NULL) {
      *out_offset = first_length + (size_t)(match - backend->memory);
      return WL_OK;
    }
  }
  return WL_ERR_NO_DATA;
}

int wl_rx_ring_consumer_copy(const wl_rx_ring_state_t *state, size_t offset,
                             uint8_t *output, size_t length) {
  const wl_rx_bipbuf_state_t *backend;
  size_t read_cursor;
  size_t readable;
  size_t cursor;
  size_t first_length;

  if (state == NULL || (output == NULL && length != 0U)) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state_const(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }

  readable = readable_snapshot(backend, &read_cursor);
  if (offset > readable || length > readable - offset) {
    return WL_ERR_NO_DATA;
  }
  if (length == 0U) {
    return WL_OK;
  }

  cursor = cursor_advance(backend->capacity * 2U, read_cursor, offset);
  first_length = contiguous_length(backend->capacity, cursor, length);
  memcpy(output,
         backend->memory + cursor_physical(backend->capacity, cursor),
         first_length);
  if (first_length < length) {
    memcpy(output + first_length, backend->memory, length - first_length);
  }
  return WL_OK;
}

int wl_rx_ring_consumer_consume(wl_rx_ring_state_t *state, size_t length) {
  wl_rx_bipbuf_state_t *backend;
  size_t read_cursor;
  size_t readable;

  if (state == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return WL_ERR_NOT_INITIALIZED;
  }

  readable = readable_snapshot(backend, &read_cursor);
  if (length > readable) {
    return WL_ERR_NO_DATA;
  }
  atomic_store_explicit(&backend->read_cursor,
                        cursor_advance(backend->capacity * 2U, read_cursor,
                                       length),
                        memory_order_release);
  return WL_OK;
}

unsigned int
wl_rx_ring_consumer_take_overflow(wl_rx_ring_state_t *state) {
  wl_rx_bipbuf_state_t *backend;

  if (state == NULL) {
    return 0U;
  }
  backend = rx_state(state);
  if (backend->memory == NULL || backend->capacity == 0U) {
    return 0U;
  }
  return atomic_exchange_explicit(&backend->overflow_events, 0U,
                                  memory_order_acq_rel);
}
