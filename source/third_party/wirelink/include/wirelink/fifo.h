/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_FIFO_H_
#define INCLUDE_WIRELINK_FIFO_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/alignment.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * An allocation-free, lock-free single-producer/single-consumer FIFO.
 *
 * The opaque context owns only queue metadata.  The fixed-size value slots
 * remain in caller-owned storage.  Do not copy or move an initialized FIFO.
 */
#define WL_FIFO_CONTEXT_STORAGE_SIZE 128U

typedef union wl_fifo {
  wl_max_align_t align;
  uint8_t private_bytes[WL_FIFO_CONTEXT_STORAGE_SIZE];
} wl_fifo_t;

typedef struct wl_fifo_config {
  size_t value_size;
  size_t value_alignment;
  /* Must be nonzero and below 2^31 so wrapped cursor subtraction is valid. */
  uint32_t capacity;
} wl_fifo_config_t;

typedef struct wl_fifo_requirements {
  size_t storage_size;
  size_t slot_stride;
  uint32_t slot_count;
} wl_fifo_requirements_t;

typedef struct wl_fifo_storage {
  void *data;
  size_t size;
} wl_fifo_storage_t;

/* A write claim and read view are opaque tokens as well as typed spans. */
typedef struct wl_fifo_write_claim {
  void *value;
  size_t value_size;
  uint32_t token;
  uint32_t private_slot;
} wl_fifo_write_claim_t;

typedef struct wl_fifo_view {
  const void *value;
  size_t value_size;
  uint32_t token;
  uint32_t private_slot;
} wl_fifo_view_t;

/*
 * depth is the current number of published values, including a borrowed head.
 * high_watermark is the greatest producer-observed depth since init.  All
 * cumulative counters saturate at UINT32_MAX and survive reset().  A snapshot
 * is safe during SPSC operation but is not transactional across fields.
 */
typedef struct wl_fifo_stats {
  uint32_t depth;
  uint32_t high_watermark;
  uint32_t publishes;
  uint32_t consumes;
  uint32_t full_rejections;
  uint32_t empty_reads;
  uint32_t aborts;
  uint32_t resets;
  uint32_t errors;
} wl_fifo_stats_t;

/*
 * value_alignment must be a nonzero power of two.  requirements() rounds
 * value_size up to an aligned slot stride and checks all size arithmetic.
 */
int wl_fifo_requirements(const wl_fifo_config_t *config,
                         wl_fifo_requirements_t *out_requirements);

/*
 * storage must meet the returned size and base-alignment requirements.
 * Initialization returns WL_ERR_NOT_SUPPORTED unless every C11 atomic used
 * by the FIFO is lock-free on the target.  Initialization and reset must be
 * externally serialized with producer and consumer activity.
 */
int wl_fifo_init(wl_fifo_t *fifo, const wl_fifo_config_t *config,
                 const wl_fifo_storage_t *storage);

/*
 * Single-producer API.  A successful claim is writable until its matching
 * publish or abort.  A full FIFO rejects the new value with WL_ERR_QUEUE_FULL;
 * it never discards or overwrites an unread value.
 */
int wl_fifo_write_claim(wl_fifo_t *fifo, wl_fifo_write_claim_t *out_claim);
int wl_fifo_write_publish(wl_fifo_t *fifo, const wl_fifo_write_claim_t *claim);
int wl_fifo_write_abort(wl_fifo_t *fifo, const wl_fifo_write_claim_t *claim);

/*
 * Single-consumer API.  acquire() returns the oldest published value or
 * WL_ERR_NO_DATA.  A successful view remains immutable and valid through its
 * matching release(), including while the producer fills other free slots.
 * Claiming/acquiring twice without finishing the active token returns
 * WL_ERR_BUSY.
 */
int wl_fifo_read_acquire(wl_fifo_t *fifo, wl_fifo_view_t *out_view);
int wl_fifo_read_release(wl_fifo_t *fifo, const wl_fifo_view_t *view);

/*
 * reset() discards every unread value and makes depth zero.  It preserves the
 * lifetime high-water mark and cumulative counters except for incrementing
 * resets.  It is valid only with no active claim/view and no concurrent calls.
 */
int wl_fifo_reset(wl_fifo_t *fifo);
int wl_fifo_get_stats(const wl_fifo_t *fifo, wl_fifo_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_FIFO_H_ */
