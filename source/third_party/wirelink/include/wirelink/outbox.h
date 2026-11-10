/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_OUTBOX_H_
#define INCLUDE_WIRELINK_OUTBOX_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/alignment.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WL_OUTBOX_STORAGE_SIZE 128U
#define WL_OUTBOX_SLOT_STORAGE_SIZE 32U

typedef union wl_outbox {
  wl_max_align_t align;
  uint8_t private_bytes[WL_OUTBOX_STORAGE_SIZE];
} wl_outbox_t;

typedef union wl_outbox_slot {
  wl_max_align_t align;
  uint8_t private_bytes[WL_OUTBOX_SLOT_STORAGE_SIZE];
} wl_outbox_slot_t;

typedef struct wl_outbox_config {
  wl_outbox_slot_t *slots;
  uint16_t slot_count;
  uint8_t *payload_storage;
  size_t payload_storage_size;
  uint16_t payload_capacity_per_slot;
  uint64_t initial_generation;
} wl_outbox_config_t;

typedef struct wl_outbox_item {
  uint64_t token;
  uint16_t slot_index;
  uint16_t message_id;
  size_t payload_length;
} wl_outbox_item_t;

typedef int32_t wl_outbox_completion_t;
enum {
  WL_OUTBOX_ACCEPTED = 0,
  WL_OUTBOX_DEFERRED,
  WL_OUTBOX_REJECTED,
};

typedef struct wl_outbox_stats {
  uint64_t submitted;
  uint64_t coalesced;
  uint64_t queue_full;
  uint64_t acquired;
  uint64_t accepted;
  uint64_t deferred;
  uint64_t rejected;
  uint64_t superseded;
  uint64_t resets;
  uint16_t depth;
} wl_outbox_stats_t;

/*
 * The outbox owns no memory and acquires no lock.  All calls must be
 * externally serialized.  Hosted runtimes may place a mutex around this API;
 * a single-owner embedded loop needs no synchronization.
 */
wl_err_t wl_outbox_init(wl_outbox_t *outbox,
                        const wl_outbox_config_t *config);

/* Replace an unsent value in the same message-id lane, or allocate a lane. */
wl_err_t wl_outbox_submit_latest(wl_outbox_t *outbox, uint16_t message_id,
                                 const uint8_t *payload, size_t payload_length,
                                 uint8_t *out_coalesced);

/* Copy one stable snapshot into caller storage and acquire its completion. */
wl_err_t wl_outbox_acquire_copy(wl_outbox_t *outbox, uint8_t *payload,
                                size_t payload_capacity,
                                wl_outbox_item_t *out_item);

/*
 * Finish the active snapshot.  ACCEPTED/REJECTED remove it only if no newer
 * value replaced the same lane while it was being sent.  DEFERRED retains it
 * for a later retry.  In every case the copied snapshot becomes invalid.
 */
wl_err_t wl_outbox_complete(wl_outbox_t *outbox,
                            const wl_outbox_item_t *item,
                            wl_outbox_completion_t completion);

/* Reset requires no active acquired snapshot. */
wl_err_t wl_outbox_reset(wl_outbox_t *outbox, uint16_t *out_discarded);
wl_err_t wl_outbox_get_stats(const wl_outbox_t *outbox,
                             wl_outbox_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_OUTBOX_H_ */
