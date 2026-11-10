/* SPDX-License-Identifier: Apache-2.0 */
#ifndef INCLUDE_WIRELINK_CLOCK_H_
#define INCLUDE_WIRELINK_CLOCK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t wl_time_ms_t;
/* Local monotonic milliseconds, modulo 2^32. Read on the owner only. The
 * callback must return promptly, never reenter Wirelink or throw across C.
 * Use one time domain throughout an endpoint's lifetime. */
typedef wl_time_ms_t (*wl_clock_now_fn)(void *user_data);
typedef struct wl_clock {
  wl_clock_now_fn now_ms;
  /* Borrowed until endpoint close; the descriptor itself is copied at init. */
  void *user_data;
} wl_clock_t;

#ifdef __cplusplus
}
#endif
#endif
