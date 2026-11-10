#include "robot_messages_bindings.h"

#include <limits.h>

static void robot_messages_count(uint32_t *counter) {
  if (*counter != UINT32_MAX) ++*counter;
}

robot_messages_dispatch_result_t robot_messages_dispatch_event(wl_ctx_t *ctx, const wl_event_t *event, robot_messages_router_t *router) {
  robot_messages_dispatch_result_t result = { ROBOT_MESSAGES_DISPATCH_INVALID_ARGUMENT, 0U, WL_EVT_NONE, WL_CODEC_OK, 0 };
  if (event == NULL) return result;
  result.message_id = event->message_id;
  result.event_type = event->type;
  if (event->type != WL_EVT_UNRELIABLE_RX && event->type != WL_EVT_RELIABLE_RX) {
    result.domain = ROBOT_MESSAGES_DISPATCH_NON_RX;
    if (router != NULL) robot_messages_count(&router->counters.non_rx);
    return result;
  }
  if (ctx == NULL) return result;
  switch (event->message_id) {
    case STREAM_HEADER_MESSAGE_ID:
      if (router == NULL || router->stream_header.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->stream_header.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = stream_header_decode(event->payload, event->payload_len, router->stream_header.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->stream_header.handler(router->stream_header.user_data, router->stream_header.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case ARM_COMMAND_MESSAGE_ID:
      if (router == NULL || router->arm_command.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->arm_command.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = arm_command_decode(event->payload, event->payload_len, router->arm_command.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->arm_command.handler(router->arm_command.user_data, router->arm_command.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case HAND_COMMAND_MESSAGE_ID:
      if (router == NULL || router->hand_command.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->hand_command.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = hand_command_decode(event->payload, event->payload_len, router->hand_command.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->hand_command.handler(router->hand_command.user_data, router->hand_command.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case ARM_STATE_MESSAGE_ID:
      if (router == NULL || router->arm_state.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->arm_state.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = arm_state_decode(event->payload, event->payload_len, router->arm_state.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->arm_state.handler(router->arm_state.user_data, router->arm_state.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case HAND_STATE_MESSAGE_ID:
      if (router == NULL || router->hand_state.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->hand_state.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = hand_state_decode(event->payload, event->payload_len, router->hand_state.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->hand_state.handler(router->hand_state.user_data, router->hand_state.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case FAULT_SNAPSHOT_MESSAGE_ID:
      if (router == NULL || router->fault_snapshot.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->fault_snapshot.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = fault_snapshot_decode(event->payload, event->payload_len, router->fault_snapshot.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->fault_snapshot.handler(router->fault_snapshot.user_data, router->fault_snapshot.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case DIAGNOSTICS_MESSAGE_ID:
      if (router == NULL || router->diagnostics.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->diagnostics.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = diagnostics_decode(event->payload, event->payload_len, router->diagnostics.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->diagnostics.handler(router->diagnostics.user_data, router->diagnostics.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case FAULT_EVENT_MESSAGE_ID:
      if (router == NULL || router->fault_event.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->fault_event.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = fault_event_decode(event->payload, event->payload_len, router->fault_event.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->fault_event.handler(router->fault_event.user_data, router->fault_event.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case OPERATION_REQUEST_MESSAGE_ID:
      if (router == NULL || router->operation_request.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->operation_request.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = operation_request_decode(event->payload, event->payload_len, router->operation_request.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->operation_request.handler(router->operation_request.user_data, router->operation_request.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case OPERATION_RESULT_MESSAGE_ID:
      if (router == NULL || router->operation_result.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->operation_result.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = operation_result_decode(event->payload, event->payload_len, router->operation_result.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->operation_result.handler(router->operation_result.user_data, router->operation_result.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case CONFIGURE_REQUEST_MESSAGE_ID:
      if (router == NULL || router->configure_request.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->configure_request.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = configure_request_decode(event->payload, event->payload_len, router->configure_request.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->configure_request.handler(router->configure_request.user_data, router->configure_request.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case CONFIGURE_RESULT_MESSAGE_ID:
      if (router == NULL || router->configure_result.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->configure_result.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = configure_result_decode(event->payload, event->payload_len, router->configure_result.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->configure_result.handler(router->configure_result.user_data, router->configure_result.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    case PEER_ARM_STATE_MESSAGE_ID:
      if (router == NULL || router->peer_arm_state.handler == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE;
        if (router != NULL) robot_messages_count(&router->counters.missing_route);
        break;
      }
      if (router->peer_arm_state.scratch == NULL) {
        result.domain = ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH;
        robot_messages_count(&router->counters.missing_scratch);
        break;
      }
      result.codec_status = peer_arm_state_decode(event->payload, event->payload_len, router->peer_arm_state.scratch);
      if (result.codec_status != WL_CODEC_OK) {
        result.domain = ROBOT_MESSAGES_DISPATCH_CODEC_ERROR;
        robot_messages_count(&router->counters.codec_failure);
        break;
      }
      result.handler_result = router->peer_arm_state.handler(router->peer_arm_state.user_data, router->peer_arm_state.scratch, event->type == WL_EVT_RELIABLE_RX ? WL_DELIVERY_RELIABLE : WL_DELIVERY_UNRELIABLE);
      if (result.handler_result != 0) {
        result.domain = ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR;
        robot_messages_count(&router->counters.handler_failure);
        break;
      }
      result.domain = ROBOT_MESSAGES_DISPATCH_OK;
      robot_messages_count(&router->counters.delivered);
      break;
    default:
      result.domain = ROBOT_MESSAGES_DISPATCH_UNKNOWN_MESSAGE;
      if (router != NULL) robot_messages_count(&router->counters.unknown_message);
      break;
  }
  wl_event_release(ctx, event);
  return result;
}

robot_messages_send_result_t robot_messages_stream_header_send(wl_ctx_t *ctx, const stream_header_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, STREAM_HEADER_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = stream_header_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_arm_command_send(wl_ctx_t *ctx, const arm_command_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, ARM_COMMAND_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = arm_command_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_hand_command_send(wl_ctx_t *ctx, const hand_command_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, HAND_COMMAND_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = hand_command_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_arm_state_send(wl_ctx_t *ctx, const arm_state_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, ARM_STATE_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = arm_state_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_hand_state_send(wl_ctx_t *ctx, const hand_state_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, HAND_STATE_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = hand_state_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_fault_snapshot_send(wl_ctx_t *ctx, const fault_snapshot_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, FAULT_SNAPSHOT_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = fault_snapshot_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_diagnostics_send(wl_ctx_t *ctx, const diagnostics_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, DIAGNOSTICS_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = diagnostics_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_fault_event_send(wl_ctx_t *ctx, const fault_event_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, FAULT_EVENT_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = fault_event_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_operation_request_send(wl_ctx_t *ctx, const operation_request_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, OPERATION_REQUEST_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = operation_request_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_operation_result_send(wl_ctx_t *ctx, const operation_result_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, OPERATION_RESULT_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = operation_result_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_configure_request_send(wl_ctx_t *ctx, const configure_request_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, CONFIGURE_REQUEST_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = configure_request_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_configure_result_send(wl_ctx_t *ctx, const configure_result_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, CONFIGURE_RESULT_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = configure_result_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}

robot_messages_send_result_t robot_messages_peer_arm_state_send(wl_ctx_t *ctx, const peer_arm_state_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms) {
  robot_messages_send_result_t result = { ROBOT_MESSAGES_SEND_CORE_ERROR, WL_CODEC_OK, WL_OK, 0U, 0U };
  wl_tx_payload_claim_t claim = {0};
  result.core_result = wl_tx_payload_claim(ctx, PEER_ARM_STATE_MESSAGE_ID, delivery, &claim);
  if (result.core_result != WL_OK) return result;
  result.codec_status = peer_arm_state_encode(message, claim.span.data, claim.span.length, &result.payload_length);
  if (result.codec_status != WL_CODEC_OK) {
    result.domain = ROBOT_MESSAGES_SEND_CODEC_ERROR;
    (void)wl_tx_payload_abort(ctx, &claim);
    return result;
  }
  result.core_result = wl_tx_payload_commit(ctx, &claim, result.payload_length, now_ms, delivery == WL_DELIVERY_RELIABLE ? &result.handle : NULL);
  if (result.core_result != WL_OK) return result;
  result.domain = ROBOT_MESSAGES_SEND_OK;
  return result;
}
