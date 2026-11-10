#ifndef WIRELINK_SRC_RX_RING_H_
#define WIRELINK_SRC_RX_RING_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/wirelink.h"
#include "wirelink/rx_ring_state.h"
#include "wirelink/span.h"

/*
 * Internal API of Wirelink's fixed atomic SPSC BipBuffer implementation.
 * It is deliberately not a selectable backend boundary.
 */
size_t wl_rx_ring_storage_size(size_t usable_capacity);
int wl_rx_ring_init(wl_rx_ring_state_t *state, uint8_t *memory,
                    size_t memory_size);

int wl_rx_ring_producer_reserve(wl_rx_ring_state_t *state,
                                wl_span_t *out_span);
int wl_rx_ring_producer_commit(wl_rx_ring_state_t *state, size_t length);
void wl_rx_ring_producer_note_overflow(wl_rx_ring_state_t *state);

int wl_rx_ring_dma_claim(wl_rx_ring_state_t *state, size_t maximum_length,
                         wl_rx_dma_claim_t *out_claim);
int wl_rx_ring_dma_publish(wl_rx_ring_state_t *state,
                           const wl_rx_dma_claim_t *claim, size_t offset,
                           size_t length);
int wl_rx_ring_dma_finish(wl_rx_ring_state_t *state,
                          const wl_rx_dma_claim_t *claim);
int wl_rx_ring_dma_abort(wl_rx_ring_state_t *state);

size_t wl_rx_ring_readable(const wl_rx_ring_state_t *state);
int wl_rx_ring_consumer_overflow_pending(const wl_rx_ring_state_t *state);
wl_span_t wl_rx_ring_consumer_peek(wl_rx_ring_state_t *state);
int wl_rx_ring_consumer_find(const wl_rx_ring_state_t *state, uint8_t value,
                             size_t *out_offset);
int wl_rx_ring_consumer_copy(const wl_rx_ring_state_t *state, size_t offset,
                             uint8_t *output, size_t length);
int wl_rx_ring_consumer_consume(wl_rx_ring_state_t *state, size_t length);
unsigned int
wl_rx_ring_consumer_take_overflow(wl_rx_ring_state_t *state);

#endif /* WIRELINK_SRC_RX_RING_H_ */
