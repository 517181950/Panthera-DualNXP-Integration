/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_ALIGNMENT_H_
#define INCLUDE_WIRELINK_ALIGNMENT_H_

#include <stddef.h>

/*
 * MSVC's C frontend does not expose C11 max_align_t. Reconstruct its maximum
 * scalar alignment so C and C++ callers agree on opaque context layout.
 */
#if defined(_MSC_VER) && !defined(__clang__)
typedef union wl_max_align {
  long double floating;
  void *pointer;
  __int64 integer;
} wl_max_align_t;
#else
typedef max_align_t wl_max_align_t;
#endif

#endif /* INCLUDE_WIRELINK_ALIGNMENT_H_ */
