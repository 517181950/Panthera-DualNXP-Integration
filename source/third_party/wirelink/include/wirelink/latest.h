/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_LATEST_H_
#define INCLUDE_WIRELINK_LATEST_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/alignment.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * An allocation-free, single-producer/single-consumer LATEST mailbox.
 *
 * The implementation owns exactly three caller-provided value slots.  The
 * opaque context contains the lock-free ownership exchange, but never the
 * values themselves.  Do not copy or move an initialized context.
 */
#define WL_LATEST_SLOT_COUNT 3U
#define WL_LATEST_CONTEXT_STORAGE_SIZE 128U

typedef union wl_latest {
  wl_max_align_t align;
  uint8_t private_bytes[WL_LATEST_CONTEXT_STORAGE_SIZE];
} wl_latest_t;

typedef struct wl_latest_config {
  size_t value_size;
  size_t value_alignment;
  /* The first publish receives initial_generation + 1 modulo 2^32. */
  uint32_t initial_generation;
} wl_latest_config_t;

typedef struct wl_latest_requirements {
  size_t storage_size;
  size_t slot_stride;
  uint32_t slot_count;
} wl_latest_requirements_t;

typedef struct wl_latest_storage {
  void *data;
  size_t size;
} wl_latest_storage_t;

/* A write claim and read view are opaque tokens as well as typed spans. */
typedef struct wl_latest_write_claim {
  void *value;
  size_t value_size;
  uint32_t token;
  uint32_t private_slot;
} wl_latest_write_claim_t;

typedef struct wl_latest_view {
  const void *value;
  size_t value_size;
  uint32_t generation;
  uint32_t token;
  uint32_t private_slot;
} wl_latest_view_t;

/*
 * All counters except generation saturate at UINT32_MAX.  generation is
 * assigned to the most recently published value and wraps modulo 2^32 (zero
 * is valid after wrap).  A statistics snapshot is not transactional across
 * fields.
 */
typedef struct wl_latest_stats {
  uint32_t generation;
  uint32_t publishes;
  uint32_t reads;
  uint32_t coalesced;
  uint32_t empty_reads;
  uint32_t resets;
  uint32_t errors;
} wl_latest_stats_t;

/*
 * value_alignment must be a nonzero power of two.  requirements() rounds
 * value_size up to a valid slot stride and checks all size arithmetic.
 */
int wl_latest_requirements(const wl_latest_config_t *config,
                           wl_latest_requirements_t *out_requirements);

/*
 * storage must meet the returned size and base-alignment requirements.
 * Initialization returns WL_ERR_NOT_SUPPORTED unless every C11 atomic used by
 * the mailbox is lock-free on the target.  Initialization/reset must be
 * externally serialized with producer and consumer activity.
 */
int wl_latest_init(wl_latest_t *mailbox, const wl_latest_config_t *config,
                   const wl_latest_storage_t *storage);

/* Single-producer API.  The claimed pointer is writable until publish/abort. */
int wl_latest_write_claim(wl_latest_t *mailbox,
                          wl_latest_write_claim_t *out_claim);
int wl_latest_write_publish(wl_latest_t *mailbox,
                            const wl_latest_write_claim_t *claim);
int wl_latest_write_abort(wl_latest_t *mailbox,
                          const wl_latest_write_claim_t *claim);

/*
 * Single-consumer API.  acquire() returns only a value not acquired before;
 * otherwise it returns WL_ERR_NO_DATA.  A successful view remains immutable
 * and valid through its matching release(), even while the producer publishes
 * newer values.  The consumer must not access its pointer after release.
 * Claiming/acquiring twice without finishing the outstanding token returns
 * WL_ERR_BUSY.
 */
int wl_latest_read_acquire(wl_latest_t *mailbox, wl_latest_view_t *out_view);
int wl_latest_read_release(wl_latest_t *mailbox, const wl_latest_view_t *view);

/*
 * reset() makes the mailbox empty and discards an unread value.  It preserves
 * generation and cumulative counters except for incrementing resets.  It is
 * valid only with no active write claim/read view and no concurrent calls.
 */
int wl_latest_reset(wl_latest_t *mailbox);
int wl_latest_get_stats(const wl_latest_t *mailbox,
                        wl_latest_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_LATEST_H_ */
