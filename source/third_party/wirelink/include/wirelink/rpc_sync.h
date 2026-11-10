/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_RPC_SYNC_H
#define WIRELINK_RPC_SYNC_H

#include "wirelink/rpc_async.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Generated/platform bridge, not business configuration. The blocked caller
 * owns context until invoke returns. submit runs only on the endpoint owner;
 * deadline uses the endpoint clock and includes time queued in the executor.
 * An expired queued submission returns WL_ERR_TIMEOUT without sending.
 * Accepted submissions notify exactly once. The bridge copies typed output
 * before notify, which may wake and release the blocked caller immediately. */
typedef void (*wl_rpc_sync_notify_fn)(void *context, const wl_rpc_completion_t *result);
typedef struct {
  void *context;
  wl_err_t (*submit)(void *context, wl_time_ms_t deadline,
      wl_rpc_sync_notify_fn notify, void *notify_context, wl_rpc_call_t *call);
} wl_rpc_sync_call_t;

typedef struct wl_rpc_executor {
  wl_rpc_completion_t (*invoke)(void *context, const wl_rpc_sync_call_t *call,
      uint32_t timeout_ms);
  void *context;
} wl_rpc_executor_t;

#ifdef __cplusplus
}
#endif
#endif
