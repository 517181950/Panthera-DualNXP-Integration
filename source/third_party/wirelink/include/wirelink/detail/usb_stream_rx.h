/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_DETAIL_USB_STREAM_RX_H_
#define WIRELINK_DETAIL_USB_STREAM_RX_H_

#include <string.h>
#include "wirelink/port.h"

/* Adapter-private SPSC producer helper, not a core ABI. All calls must be
 * serialized on the RX producer. Keep USB buffers at least one full packet:
 * a partial COBS frame cannot be drained to normalize a short ring tail.
 * Only that tail uses staging; ordinary receives remain direct-to-ring. */
typedef struct wl_usb_stream_rx {
  wl_ctx_t *link;
  uint8_t *staging;
  size_t staging_size;
  size_t pending_length;
  size_t pending_offset;
  wl_rx_dma_claim_t claim;
  wl_span_t active_span;
  uint32_t token;
  uint8_t active;
  uint8_t staged;
} wl_usb_stream_rx_t;

static inline void wl_usb_stream_rx_init(wl_usb_stream_rx_t *rx, wl_ctx_t *link,
                                         uint8_t *staging, size_t size) {
  memset(rx, 0, sizeof(*rx));
  rx->link = link;
  rx->staging = staging;
  rx->staging_size = size;
}

static inline void wl_usb_stream_rx_abort(wl_usb_stream_rx_t *rx) {
  if (rx->active && !rx->staged) (void)wl_rx_dma_abort(rx->link);
  else if (rx->active || rx->pending_length != 0U) wl_rx_note_overflow(rx->link);
  rx->active = 0U;
  rx->pending_length = rx->pending_offset = 0U;
}

static inline int wl_usb_stream_rx_drain(wl_usb_stream_rx_t *rx) {
  if (rx->active) return WL_ERR_INVALID_STATE;
  while (rx->pending_offset < rx->pending_length) {
    wl_span_t span;
    size_t count = rx->pending_length - rx->pending_offset;
    int result = wl_rx_reserve(rx->link, &span);
    if (result != WL_OK) return result;
    if (count > span.length) count = span.length;
    if (count != 0U) memcpy(span.data, rx->staging + rx->pending_offset, count);
    result = wl_rx_commit(rx->link, count);
    if (result != WL_OK) return result;
    rx->pending_offset += count;
    /* Do not report overflow or rearm USB while retained bytes are blocked. */
    if (count == 0U) return WL_ERR_WOULD_BLOCK;
  }
  rx->pending_length = rx->pending_offset = 0U;
  return WL_OK;
}

static inline int wl_usb_stream_rx_acquire(wl_usb_stream_rx_t *rx,
                                           size_t maximum, size_t packet_size) {
  int result;
  if (!packet_size || maximum < packet_size || rx->staging_size < packet_size)
    return WL_ERR_INVALID_ARG;
  result = wl_usb_stream_rx_drain(rx);
  if (result != WL_OK) return result;
  result = wl_rx_dma_claim(rx->link, maximum, &rx->claim);
  if (result != WL_OK) return result;
  rx->staged = rx->claim.span.length < packet_size;
  if (rx->staged) {
    result = wl_rx_dma_finish(rx->link, &rx->claim);
    if (result != WL_OK) {
      (void)wl_rx_dma_abort(rx->link);
      return result;
    }
    rx->active_span.data = rx->staging;
    rx->active_span.length = packet_size;
  } else {
    rx->active_span = rx->claim.span;
    rx->active_span.length -= rx->active_span.length % packet_size;
  }
  if (++rx->token == 0U) ++rx->token;
  rx->active = 1U;
  return WL_OK;
}

static inline int wl_usb_stream_rx_complete(wl_usb_stream_rx_t *rx,
                                            uint32_t token, const uint8_t *data,
                                            size_t length) {
  int result = WL_OK;
  if (!rx->active || token != rx->token || data != rx->active_span.data ||
      length > rx->active_span.length) {
    wl_usb_stream_rx_abort(rx);
    return WL_ERR_INVALID_ARG;
  }
  if (rx->staged) {
    rx->pending_length = length;
    rx->pending_offset = 0U;
  } else {
    if (length != 0U) result = wl_rx_dma_publish(rx->link, &rx->claim, 0U, length);
    if (result == WL_OK) result = wl_rx_dma_finish(rx->link, &rx->claim);
    if (result != WL_OK) (void)wl_rx_dma_abort(rx->link);
  }
  rx->active = 0U;
  return result;
}

#endif
