/* SPDX-License-Identifier: Apache-2.0 */
#ifndef INCLUDE_WIRELINK_ENDPOINT_H_
#define INCLUDE_WIRELINK_ENDPOINT_H_

#include "wirelink/pump.h"
#include "wirelink/wait.h"
#include "wirelink/environment.h"

#ifdef __cplusplus
extern "C" {
#endif

struct wl_rpc_executor;

/* Optional owner-local product policy, composed with generated dispatch.
 * Observe events before dispatch; never release RX or take the observed TX.
 * progress runs after all events, before generated RPC service. Return true
 * only for work that needs another pass, not for already consumed work.
 * No callback may recursively step/close the endpoint. */
typedef struct {
  void *user_data;
  void (*on_event)(void *user_data, wl_ctx_t *link,
                   const wl_event_t *event, wl_time_ms_t now_ms);
  wl_pump_application_progress_fn progress;
  wl_pump_deadline_hint_fn deadline_hint;
} wl_endpoint_policy_t;

/* Statically assembled owner-local services. The array and contexts are
 * borrowed until close. Callbacks run on the endpoint owner, never recursively
 * step/close it or release/take observed events. Progress follows generated
 * RPC progress, in rotating order; each callback should do bounded work and
 * return true only when another immediate pass can make progress. A blocked
 * service must supply a deadline or arrange a transport/owner wakeup.
 * A nonzero RX session is observed before dispatch, including direct routes.
 * The initial observation has previous_session == 0. Close runs in reverse
 * array order, after adapter quiescence, while the link is still valid. */
typedef struct {
  void *user_data;
  wl_pump_application_progress_fn progress;
  wl_pump_deadline_hint_fn deadline_hint;
  void (*on_peer_session)(void *user_data, wl_ctx_t *link,
                         uint64_t previous_session, uint64_t session,
                         wl_time_ms_t now_ms);
  void (*on_close)(void *user_data, wl_ctx_t *link);
} wl_endpoint_service_t;

/* Default owner-side assembly, normally embedded by WLC. Zero-initialize
 * before first init; never copy/move while initialized. Members are private.
 * This object creates no thread and owns no transport or external buffers. */
typedef struct wl_endpoint {
  wl_ctx_t private_link;
  wl_pump_hooks_t private_hooks;
  wl_endpoint_policy_t private_policy;
  const wl_endpoint_service_t *private_services;
  size_t private_service_count;
  size_t private_service_cursor;
  uint64_t private_peer_session;
  wl_pump_result_t private_step;
  wl_clock_t private_clock;
  wl_waiter_t private_waiter;
  const struct wl_rpc_executor *private_executor;
  wl_time_ms_t private_now;
  uint8_t private_stepping;
  uint8_t private_policy_pending;
  uint8_t private_ready;
  uint8_t private_started;
} wl_endpoint_t;

/* Generated lifecycle bridge; storage/transport remain owned by the caller.
 * A platform executor drives step and close, never also dispatches core events. */
typedef struct {
  wl_endpoint_t *endpoint;
  void *context;
  wl_err_t (*step)(void *context);
  wl_err_t (*close)(void *context);
  /* One means get_hint includes all follow-up work. Zero preserves the
   * conservative progress contract of custom drivers. */
  uint8_t readiness_complete;
} wl_endpoint_driver_t;

wl_err_t wl_endpoint_init(wl_endpoint_t *endpoint, const wl_config_t *config,
                         const wl_storage_t *storage,
                         const wl_clock_t *clock,
                         const wl_pump_hooks_t *application);
wl_ctx_t *wl_endpoint_link(wl_endpoint_t *endpoint);
uint8_t wl_endpoint_has_adapter(const wl_endpoint_t *endpoint);
/* Adapter integration: only adapter fields are installed. Application hooks
 * remain owned by the generated assembly. Attach before driving the owner. */
wl_err_t wl_endpoint_attach(wl_endpoint_t *endpoint,
                           const wl_pump_hooks_t *adapter);
/* Setup-only. Copies the descriptor, not its context; NULL clears it.
 * Generated dispatch/storage ownership is never replaced by this policy. */
wl_err_t wl_endpoint_set_policy(wl_endpoint_t *endpoint,
                                const wl_endpoint_policy_t *policy);
/* Before the first step only. NULL with zero count clears the binding. */
wl_err_t wl_endpoint_set_services(wl_endpoint_t *endpoint,
    const wl_endpoint_service_t *services, size_t count);
/* Setup-only platform integration; a NULL descriptor disables sync waiting.
 * Storing this descriptor does not create a thread or perform a wait. */
wl_err_t wl_endpoint_set_waiter(wl_endpoint_t *endpoint, const wl_waiter_t *waiter);
const wl_waiter_t *wl_endpoint_waiter(const wl_endpoint_t *endpoint);
/* Setup-only borrowed executor binding, immutable until callers have joined.
 * close deliberately retains it: late proxy calls are rejected by the stopped
 * executor without touching mutable endpoint state. Executor storage must
 * outlive its bound endpoint and all callers. init clears an old binding. */
wl_err_t wl_endpoint_set_rpc_executor(wl_endpoint_t *endpoint,
    const struct wl_rpc_executor *executor);
const struct wl_rpc_executor *wl_endpoint_rpc_executor(const wl_endpoint_t *endpoint);
wl_err_t wl_endpoint_get_clock(const wl_endpoint_t *endpoint, wl_clock_t *clock);
wl_err_t wl_endpoint_step(wl_endpoint_t *endpoint, size_t event_budget);
wl_err_t wl_endpoint_get_hint(const wl_endpoint_t *endpoint,
                             wl_poll_hint_t *hint);
/* Assembly bridge, not a business operation. Inside a step returns the pass
 * snapshot; outside a step reads the configured clock once. Does not poll. */
wl_err_t wl_endpoint_now(const wl_endpoint_t *endpoint, wl_time_ms_t *now_ms);
const wl_pump_result_t *wl_endpoint_last_step(const wl_endpoint_t *endpoint);
/* Stops an attached adapter before invalidating the endpoint. Idempotent.
 * Close every endpoint before releasing a shared adapter's storage. */
void wl_endpoint_close(wl_endpoint_t *endpoint);

#ifdef __cplusplus
}
#endif
#endif
