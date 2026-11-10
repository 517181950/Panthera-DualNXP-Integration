/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_RPC_RESULT_H
#define WIRELINK_RPC_RESULT_H

#include "wirelink/codec.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Terminal business outcomes, never intermediate link states. */
typedef int32_t wl_rpc_status_t;
enum {
  WL_RPC_SUCCESS = 0,
  WL_RPC_REJECTED,
  WL_RPC_TIMED_OUT,
  WL_RPC_CANCELLED,
  WL_RPC_FAILED,
};

/* Per-call diagnostics. rejection is meaningful only for REJECTED; a typed
 * response is meaningful only for SUCCESS. The other fields retain disjoint
 * local admission/platform wait, link (wl_err_t), advanced RPC (wl_rpc_err_t),
 * and codec error domains. local_error is zero in accepted async completions. */
typedef struct {
  wl_rpc_status_t status;
  int32_t rejection;
  wl_err_t local_error;
  wl_err_t transport_error;
  int32_t runtime_error;
  wl_codec_status_t codec_error;
} wl_rpc_completion_t;

const char *wl_rpc_status_str(wl_rpc_status_t status);

#ifdef __cplusplus
}
#endif
#endif
