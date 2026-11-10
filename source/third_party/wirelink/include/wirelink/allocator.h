/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_ALLOCATOR_H
#define WIRELINK_ALLOCATOR_H

#include <stddef.h>
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Optional generated endpoint creation, never protocol hot-path allocation.
 * allocate returns NULL or at least size writable bytes aligned to alignment
 * (a nonzero power of two). Memory need not be zeroed. No implicit heap fallback.
 * deallocate receives the same pointer, size and alignment, exactly once.
 * The descriptor is copied; context stays alive through destroy. Callbacks must
 * not throw or reenter this endpoint. Sharing an allocator across owners requires
 * external synchronization or a thread-safe implementation. Ordinary RAM here
 * is not a promise of DMA reachability/cache coherency for an adapter. */
typedef struct {
  void *(*allocate)(void *context, size_t size, size_t alignment);
  void (*deallocate)(void *context, void *pointer, size_t size, size_t alignment);
  void *context;
} wl_allocator_t;

#ifdef __cplusplus
}
#endif
#endif
