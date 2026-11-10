/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_ENVIRONMENT_H
#define WIRELINK_ENVIRONMENT_H
#include "wirelink/clock.h"
#include "wirelink/types.h"
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Produce a fresh, nonzero communication-instance identity. Not an address or
 * authentication credential. Platforms normally use OS/hardware randomness;
 * a bare-metal integration may supply a durable identity allocator instead.
 * Called only during owner-side initialization, never from an ISR/hot path.
 * Return WL_OK on success, otherwise a wl_err_t. No exceptions/reentrancy.
 * A copied descriptor borrows context for as long as that descriptor is used. */
typedef struct {
  wl_err_t (*next)(void *context, uint64_t *identity);
  void *context;
} wl_session_source_t;

typedef struct {
  wl_clock_t clock;
  wl_session_source_t session;
} wl_environment_t;

/* No OS/default/global fallback. Failure leaves out unchanged. previous is
 * zero for first initialization; comparing one prior ID is not a uniqueness
 * proof or a replacement for a correctly provisioned source. */
static inline wl_err_t wl_session_next(wl_session_source_t source,
                                      uint64_t previous, uint64_t *out) {
  uint64_t value = 0U;
  wl_err_t error;
  if (source.next == NULL || out == NULL) return WL_ERR_INVALID_ARG;
  error = source.next(source.context, &value);
  if (error != WL_OK) return error;
  if (value == 0U || value == previous) return WL_ERR_INVALID_STATE;
  *out = value;
  return WL_OK;
}
#ifdef __cplusplus
}
#endif
#endif
