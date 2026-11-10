/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_DIAGNOSTICS_H_
#define INCLUDE_WIRELINK_DIAGNOSTICS_H_

#include <stddef.h>

#include "wirelink/adapter.h"
#include "wirelink/bulk.h"
#include "wirelink/fifo.h"
#include "wirelink/latest.h"
#include "wirelink/link.h"
#include "wirelink/outbox.h"
#include "wirelink/rpc.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Allocation-free key/value diagnostics writer. length is the bytes currently
 * stored, while required is the bytes that would have been stored with an
 * unlimited buffer; both exclude the trailing NUL. A nonzero-capacity buffer
 * is always NUL terminated, including after truncation.
 */
typedef struct wl_diag_writer {
  char *data;
  size_t capacity;
  size_t length;
  size_t required;
} wl_diag_writer_t;

/* data may be NULL only for a zero-capacity sizing pass. */
wl_err_t wl_diag_writer_init(wl_diag_writer_t *writer, char *data,
                             size_t capacity);
void wl_diag_writer_reset(wl_diag_writer_t *writer);

/* Each formatter appends one newline-terminated record. */
wl_err_t wl_diag_format_rx_counters(wl_diag_writer_t *writer,
                                    const wl_rx_counters_t *counters);
wl_err_t wl_diag_format_adapter_stats(wl_diag_writer_t *writer,
                                      const wl_adapter_stats_t *stats);
wl_err_t wl_diag_format_fifo_stats(wl_diag_writer_t *writer,
                                   const wl_fifo_stats_t *stats);
wl_err_t wl_diag_format_latest_stats(wl_diag_writer_t *writer,
                                     const wl_latest_stats_t *stats);
wl_err_t wl_diag_format_outbox_stats(wl_diag_writer_t *writer,
                                     const wl_outbox_stats_t *stats);
wl_err_t
wl_diag_format_bulk_receiver_stats(wl_diag_writer_t *writer,
                                   const wl_bulk_receiver_stats_t *stats);
wl_err_t wl_diag_format_bulk_sender_stats(wl_diag_writer_t *writer,
                                          const wl_bulk_sender_stats_t *stats);
wl_err_t wl_diag_format_rpc_client_result(wl_diag_writer_t *writer,
                                          const wl_rpc_client_result_t *result);
wl_err_t wl_diag_format_rpc_peer_observation(
    wl_diag_writer_t *writer, const wl_rpc_peer_observation_t *observation);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_DIAGNOSTICS_H_ */
