/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/pump.h"
#include "context.h"

#include <string.h>

static int service_result_expected(int result) {
  return result == WL_OK || result == WL_ERR_NO_DATA ||
         result == WL_ERR_WOULD_BLOCK;
}

static int is_rx_event(const wl_event_t *event) {
  return event->type == WL_EVT_UNRELIABLE_RX ||
         event->type == WL_EVT_RELIABLE_RX;
}

static int is_terminal_tx_event(const wl_event_t *event) {
  return event->type == WL_EVT_TX_SUCCESS ||
         event->type == WL_EVT_TX_TIMEOUT ||
         event->type == WL_EVT_TX_FAILED;
}

wl_err_t wl_pump_step(wl_ctx_t *ctx, wl_time_ms_t now_ms,
                      size_t event_budget, const wl_pump_hooks_t *hooks,
                      wl_pump_result_t *out_result) {
  wl_pump_result_t result;
  size_t index;

  if (ctx == NULL || event_budget == 0U || out_result == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (wl_ctx_impl(ctx)->initialized == 0U) return WL_ERR_NOT_INITIALIZED;
  /* Service can observe asynchronous TX completion. Its ACK timer must use
   * this owner pass, not the time retained by the preceding poll. */
  wl_ctx_impl(ctx)->now_ms = now_ms;
  memset(&result, 0, sizeof(result));
  result.service_result = WL_ERR_NO_DATA;
  result.poll_result = WL_ERR_NO_DATA;

  if (hooks != NULL && hooks->service != NULL) {
    result.service_result = hooks->service(hooks->adapter_user_data);
    if (!service_result_expected(result.service_result)) {
      result.service_errors = 1U;
    }
  }

  for (index = 0U; index < event_budget; ++index) {
    wl_event_t event;
    wl_pump_event_disposition_t disposition = WL_PUMP_EVENT_UNHANDLED;
    int poll_result = wl_poll(ctx, now_ms, &event);

    result.poll_result = poll_result;
    if (poll_result == WL_ERR_NO_DATA) {
      break;
    }
    if (poll_result != WL_OK) {
      result.poll_errors = 1U;
      break;
    }

    ++result.events;
    result.progress = 1U;
    if (is_rx_event(&event)) {
      ++result.rx_events;
      if (hooks != NULL && hooks->on_event != NULL) {
        disposition = hooks->on_event(hooks->application_user_data, ctx, &event,
                                      now_ms);
      }
      if (disposition == WL_PUMP_EVENT_UNHANDLED) {
        wl_event_release(ctx, &event);
      }
      continue;
    }

    if (!is_terminal_tx_event(&event) || event.handle == 0U) {
      continue;
    }
    if (hooks != NULL && hooks->on_event != NULL) {
      disposition = hooks->on_event(hooks->application_user_data, ctx, &event,
                                    now_ms);
    }
    if (disposition == WL_PUMP_EVENT_UNHANDLED) {
      wl_tx_result_t ignored;
      (void)wl_tx_take(ctx, event.handle, &ignored);
    }
  }

  if (hooks != NULL && hooks->application_progress != NULL &&
      hooks->application_progress(hooks->application_user_data, ctx, now_ms) !=
          0U) {
    result.progress = 1U;
  }
  *out_result = result;
  return WL_OK;
}

wl_err_t wl_pump_get_hint(const wl_ctx_t *ctx, wl_time_ms_t now_ms,
                          const wl_pump_hooks_t *hooks,
                          wl_poll_hint_t *out_hint) {
  wl_poll_hint_t hint;
  int result;

  if (ctx == NULL || out_hint == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  result = wl_poll_get_hint(ctx, now_ms, &hint);
  if (result != WL_OK) {
    *out_hint = hint;
    return result;
  }
  if (hooks != NULL && hooks->application_deadline_hint != NULL) {
    const uint32_t deadline =
        hooks->application_deadline_hint(hooks->application_user_data, now_ms);
    if (deadline < hint.next_deadline_ms) {
      hint.next_deadline_ms = deadline;
    }
  }
  if (hooks != NULL && hooks->adapter_deadline_hint != NULL) {
    const uint32_t deadline =
        hooks->adapter_deadline_hint(hooks->adapter_user_data, now_ms);
    if (deadline < hint.next_deadline_ms) {
      hint.next_deadline_ms = deadline;
    }
  }
  *out_hint = hint;
  return WL_OK;
}

void wl_pump_quiesce(const wl_pump_hooks_t *hooks) {
  if (hooks != NULL && hooks->quiesce != NULL) {
    hooks->quiesce(hooks->adapter_user_data);
  }
}
