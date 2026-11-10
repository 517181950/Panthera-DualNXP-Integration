#ifndef INCLUDE_WIRELINK_RX_RING_STATE_H_
#define INCLUDE_WIRELINK_RX_RING_STATE_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/alignment.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Private state of Wirelink's fixed SPSC BipBuffer. Keeping the storage opaque
 * permits the core to evolve its direct-DMA claim bookkeeping without exposing
 * cursor layout to applications.
 */
#define WL_RX_RING_STATE_SIZE 256U

typedef union {
  wl_max_align_t align;
  uint8_t bytes[WL_RX_RING_STATE_SIZE];
} wl_rx_ring_state_t;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_RX_RING_STATE_H_ */
