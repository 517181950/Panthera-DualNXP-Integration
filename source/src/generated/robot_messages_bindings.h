#ifndef WIRELINK_GENERATED_ROBOT_MESSAGES_H_BINDINGS
#define WIRELINK_GENERATED_ROBOT_MESSAGES_H_BINDINGS

#include "robot_messages.h"
#include <wirelink/link.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t robot_messages_dispatch_domain_t;
enum {
  ROBOT_MESSAGES_DISPATCH_OK = 0,
  ROBOT_MESSAGES_DISPATCH_NON_RX,
  ROBOT_MESSAGES_DISPATCH_UNKNOWN_MESSAGE,
  ROBOT_MESSAGES_DISPATCH_MISSING_ROUTE,
  ROBOT_MESSAGES_DISPATCH_MISSING_SCRATCH,
  ROBOT_MESSAGES_DISPATCH_CODEC_ERROR,
  ROBOT_MESSAGES_DISPATCH_HANDLER_ERROR,
  ROBOT_MESSAGES_DISPATCH_INVALID_ARGUMENT
};

typedef struct {
  robot_messages_dispatch_domain_t domain;
  uint16_t message_id;
  wl_event_type_t event_type;
  wl_codec_status_t codec_status;
  int32_t handler_result;
} robot_messages_dispatch_result_t;

typedef struct {
  uint32_t delivered;
  uint32_t non_rx;
  uint32_t unknown_message;
  uint32_t missing_route;
  uint32_t missing_scratch;
  uint32_t codec_failure;
  uint32_t handler_failure;
} robot_messages_dispatch_counters_t;

typedef int32_t robot_messages_send_domain_t;
enum {
  ROBOT_MESSAGES_SEND_OK = 0,
  ROBOT_MESSAGES_SEND_CODEC_ERROR,
  ROBOT_MESSAGES_SEND_CORE_ERROR
};

typedef struct {
  uint8_t *data;
  size_t capacity;
} robot_messages_encode_scratch_t;

typedef struct {
  robot_messages_send_domain_t domain;
  wl_codec_status_t codec_status;
  int core_result;
  size_t payload_length;
  wl_tx_handle_t handle;
} robot_messages_send_result_t;

typedef int32_t (*robot_messages_stream_header_handler_fn)(void *user_data, const stream_header_t *message, wl_delivery_t delivery);
typedef struct {
  stream_header_t *scratch;
  robot_messages_stream_header_handler_fn handler;
  void *user_data;
} robot_messages_stream_header_route_t;

typedef int32_t (*robot_messages_arm_command_handler_fn)(void *user_data, const arm_command_t *message, wl_delivery_t delivery);
typedef struct {
  arm_command_t *scratch;
  robot_messages_arm_command_handler_fn handler;
  void *user_data;
} robot_messages_arm_command_route_t;

typedef int32_t (*robot_messages_hand_command_handler_fn)(void *user_data, const hand_command_t *message, wl_delivery_t delivery);
typedef struct {
  hand_command_t *scratch;
  robot_messages_hand_command_handler_fn handler;
  void *user_data;
} robot_messages_hand_command_route_t;

typedef int32_t (*robot_messages_arm_state_handler_fn)(void *user_data, const arm_state_t *message, wl_delivery_t delivery);
typedef struct {
  arm_state_t *scratch;
  robot_messages_arm_state_handler_fn handler;
  void *user_data;
} robot_messages_arm_state_route_t;

typedef int32_t (*robot_messages_hand_state_handler_fn)(void *user_data, const hand_state_t *message, wl_delivery_t delivery);
typedef struct {
  hand_state_t *scratch;
  robot_messages_hand_state_handler_fn handler;
  void *user_data;
} robot_messages_hand_state_route_t;

typedef int32_t (*robot_messages_fault_snapshot_handler_fn)(void *user_data, const fault_snapshot_t *message, wl_delivery_t delivery);
typedef struct {
  fault_snapshot_t *scratch;
  robot_messages_fault_snapshot_handler_fn handler;
  void *user_data;
} robot_messages_fault_snapshot_route_t;

typedef int32_t (*robot_messages_diagnostics_handler_fn)(void *user_data, const diagnostics_t *message, wl_delivery_t delivery);
typedef struct {
  diagnostics_t *scratch;
  robot_messages_diagnostics_handler_fn handler;
  void *user_data;
} robot_messages_diagnostics_route_t;

typedef int32_t (*robot_messages_fault_event_handler_fn)(void *user_data, const fault_event_t *message, wl_delivery_t delivery);
typedef struct {
  fault_event_t *scratch;
  robot_messages_fault_event_handler_fn handler;
  void *user_data;
} robot_messages_fault_event_route_t;

typedef int32_t (*robot_messages_operation_request_handler_fn)(void *user_data, const operation_request_t *message, wl_delivery_t delivery);
typedef struct {
  operation_request_t *scratch;
  robot_messages_operation_request_handler_fn handler;
  void *user_data;
} robot_messages_operation_request_route_t;

typedef int32_t (*robot_messages_operation_result_handler_fn)(void *user_data, const operation_result_t *message, wl_delivery_t delivery);
typedef struct {
  operation_result_t *scratch;
  robot_messages_operation_result_handler_fn handler;
  void *user_data;
} robot_messages_operation_result_route_t;

typedef int32_t (*robot_messages_configure_request_handler_fn)(void *user_data, const configure_request_t *message, wl_delivery_t delivery);
typedef struct {
  configure_request_t *scratch;
  robot_messages_configure_request_handler_fn handler;
  void *user_data;
} robot_messages_configure_request_route_t;

typedef int32_t (*robot_messages_configure_result_handler_fn)(void *user_data, const configure_result_t *message, wl_delivery_t delivery);
typedef struct {
  configure_result_t *scratch;
  robot_messages_configure_result_handler_fn handler;
  void *user_data;
} robot_messages_configure_result_route_t;

typedef int32_t (*robot_messages_peer_arm_state_handler_fn)(void *user_data, const peer_arm_state_t *message, wl_delivery_t delivery);
typedef struct {
  peer_arm_state_t *scratch;
  robot_messages_peer_arm_state_handler_fn handler;
  void *user_data;
} robot_messages_peer_arm_state_route_t;

typedef struct {
  robot_messages_stream_header_route_t stream_header;
  robot_messages_arm_command_route_t arm_command;
  robot_messages_hand_command_route_t hand_command;
  robot_messages_arm_state_route_t arm_state;
  robot_messages_hand_state_route_t hand_state;
  robot_messages_fault_snapshot_route_t fault_snapshot;
  robot_messages_diagnostics_route_t diagnostics;
  robot_messages_fault_event_route_t fault_event;
  robot_messages_operation_request_route_t operation_request;
  robot_messages_operation_result_route_t operation_result;
  robot_messages_configure_request_route_t configure_request;
  robot_messages_configure_result_route_t configure_result;
  robot_messages_peer_arm_state_route_t peer_arm_state;
  robot_messages_dispatch_counters_t counters;
} robot_messages_router_t;

robot_messages_dispatch_result_t robot_messages_dispatch_event(wl_ctx_t *ctx, const wl_event_t *event, robot_messages_router_t *router);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_stream_header_send(wl_ctx_t *ctx, const stream_header_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_arm_command_send(wl_ctx_t *ctx, const arm_command_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_hand_command_send(wl_ctx_t *ctx, const hand_command_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_arm_state_send(wl_ctx_t *ctx, const arm_state_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_hand_state_send(wl_ctx_t *ctx, const hand_state_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_fault_snapshot_send(wl_ctx_t *ctx, const fault_snapshot_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_diagnostics_send(wl_ctx_t *ctx, const diagnostics_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_fault_event_send(wl_ctx_t *ctx, const fault_event_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_operation_request_send(wl_ctx_t *ctx, const operation_request_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_operation_result_send(wl_ctx_t *ctx, const operation_result_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_configure_request_send(wl_ctx_t *ctx, const configure_request_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_configure_result_send(wl_ctx_t *ctx, const configure_result_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

/* Encodes directly into Wirelink-owned TX storage. */
robot_messages_send_result_t robot_messages_peer_arm_state_send(wl_ctx_t *ctx, const peer_arm_state_t *message, wl_delivery_t delivery, wl_time_ms_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
