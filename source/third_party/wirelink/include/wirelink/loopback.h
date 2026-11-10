/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_LOOPBACK_H_
#define INCLUDE_WIRELINK_LOOPBACK_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/adapter.h"
#include "wirelink/endpoint.h"
#include "wirelink/alignment.h"
#include "wirelink/port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WL_LOOPBACK_STORAGE_SIZE 256U

typedef union wl_loopback {
  wl_max_align_t align;
  uint8_t private_bytes[WL_LOOPBACK_STORAGE_SIZE];
} wl_loopback_t;

typedef int32_t wl_loopback_endpoint_t;
enum {
  WL_LOOPBACK_ENDPOINT_A = 0,
  WL_LOOPBACK_ENDPOINT_B = 1,
};

typedef struct wl_loopback_service_result {
  size_t attempts;
  size_t delivered;
  size_t blocked;
  uint32_t errors;
} wl_loopback_service_result_t;

/*
 * Bind an allocation-free asynchronous packet link between two initialized
 * native-packet contexts. The adapter exclusively owns both sink bindings
 * until wl_loopback_quiesce().
 */
wl_err_t wl_loopback_init(wl_loopback_t *loopback, wl_ctx_t *endpoint_a,
                          wl_ctx_t *endpoint_b);

/* Default assembly: connect two endpoints and install bounded transport
 * service/close hooks. Both endpoints must have no adapter attached. They
 * and loopback must share one owner; close both before releasing loopback.
 * The cable must be unused, or quiesced with all former endpoints closed. */
wl_err_t wl_loopback_connect(wl_loopback_t *loopback, wl_endpoint_t *endpoint_a,
                            wl_endpoint_t *endpoint_b);

/*
 * Attempt at most unit_budget pending unit deliveries. A destination with an
 * unreleased RX event applies backpressure without completing or copying the
 * source unit. Returns WL_OK after progress, WL_ERR_WOULD_BLOCK when every
 * pending direction is blocked, or WL_ERR_NO_DATA when idle.
 */
wl_err_t wl_loopback_service(wl_loopback_t *loopback, size_t unit_budget,
                             wl_loopback_service_result_t *out_result);

/* Unbind both sinks and fail any unit that has not completed. Idempotent. */
void wl_loopback_quiesce(wl_loopback_t *loopback);

wl_err_t wl_loopback_get_stats(const wl_loopback_t *loopback,
                               wl_loopback_endpoint_t endpoint,
                               wl_adapter_stats_t *out_stats);
wl_err_t wl_loopback_reset_stats(wl_loopback_t *loopback);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_LOOPBACK_H_ */
