/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_WAIT_H
#define WIRELINK_WAIT_H

#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Platform-owned waiting, not a clock or a protocol timer. Descriptors are
 * copied at setup; user_data remains alive until endpoint close. wait sleeps
 * until activity/notification or at most maximum_ms (UINT32_MAX = unbounded).
 * It never invokes Wirelink/user callbacks or drives the endpoint. Readiness
 * must be level-triggered or latched so notification before wait is not lost.
 * Return WL_OK or WL_ERR_NO_DATA to drive another owner pass; other errors
 * interrupt the synchronous call (WL_ERR_CANCELLED is an orderly stop).
 * Manual-clock providers must advance that same clock and notify the waiter.
 * All protocol operations remain on one owner; the C core never waits. */
typedef struct {
  wl_err_t (*wait)(void *user_data, uint32_t maximum_ms);
  void *user_data;
  /* Optional thread-safe, latched wake for background executors. No endpoint
   * work is performed here. Producers must stop before destroying the waiter. */
  void (*notify)(void *user_data);
} wl_waiter_t;

#ifdef __cplusplus
}
#endif
#endif
