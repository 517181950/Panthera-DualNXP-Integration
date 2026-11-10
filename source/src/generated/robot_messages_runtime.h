#ifndef WIRELINK_GENERATED_ROBOT_MESSAGES_RUNTIME_H
#define WIRELINK_GENERATED_ROBOT_MESSAGES_RUNTIME_H

#include "robot_messages_bindings.h"
#include <wirelink/pump.h>
#include <wirelink/endpoint.h>
#include <wirelink/allocator.h>
#include <wirelink/frame.h>
#include <string.h>
#include <wirelink/latest.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_MESSAGES_SCHEMA_IDENTITY UINT64_C(0xCBC84DAA7C64C216)
#define ROBOT_MESSAGES_BINDING_PROFILE_IDENTITY UINT64_C(0xD43C9427F1DC7724)
#define ROBOT_MESSAGES_BINDING_PROFILE_VERSION 1U
#define ROBOT_MESSAGES_IDENTITY_ALGORITHM "fnv1a64-v1"

#define ROBOT_MESSAGES_RUNTIME_CODEGEN_ABI_VERSION 32U

/* Generated capabilities, not application overrides. */
#define ROBOT_MESSAGES_RUNTIME_HAS_RPC_CLIENT 0
#define ROBOT_MESSAGES_RUNTIME_HAS_RPC_SERVER 0

typedef int32_t robot_messages_runtime_domain_t;
enum {
  ROBOT_MESSAGES_RUNTIME_OK = 0,
  ROBOT_MESSAGES_RUNTIME_NON_RX,
  ROBOT_MESSAGES_RUNTIME_UNKNOWN_MESSAGE,
  ROBOT_MESSAGES_RUNTIME_MISSING_ROUTE,
  ROBOT_MESSAGES_RUNTIME_MISSING_SCRATCH,
  ROBOT_MESSAGES_RUNTIME_DELIVERY_MISMATCH,
  ROBOT_MESSAGES_RUNTIME_CODEC_ERROR,
  ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR,
  ROBOT_MESSAGES_RUNTIME_RPC_ERROR,
  ROBOT_MESSAGES_RUNTIME_CORE_ERROR,
  ROBOT_MESSAGES_RUNTIME_APPLICATION_ERROR,
  ROBOT_MESSAGES_RUNTIME_INVALID_ARGUMENT
};

typedef uint8_t robot_messages_runtime_detail_kind_t;
enum {
  ROBOT_MESSAGES_RUNTIME_DETAIL_NONE = 0,
  ROBOT_MESSAGES_RUNTIME_DETAIL_RETAINED = 1,
  ROBOT_MESSAGES_RUNTIME_DETAIL_RPC = 2
};

typedef struct {
  wl_codec_status_t codec_status;
  int32_t storage_result;
  int32_t abort_result;
} robot_messages_runtime_retained_detail_t;

typedef union {
  robot_messages_runtime_retained_detail_t retained;
} robot_messages_runtime_detail_t;

/* Inspect detail only through the member selected by detail_kind. domain
 * classifies the outcome; zero-initialized unused detail fields retain their
 * corresponding success values. event_consumed is nonzero only when dispatch
 * released an RX event or reclaimed a terminal TX handle. */
typedef struct {
  robot_messages_runtime_domain_t domain;
  wl_event_type_t event_type;
  uint16_t message_id;
  robot_messages_runtime_detail_kind_t detail_kind;
  uint8_t event_consumed;
  robot_messages_runtime_detail_t detail;
} robot_messages_runtime_result_t;

/* Convenience helpers preserve the full diagnostic result. Detail accessors
 * return null unless detail_kind selects the requested member. Result strings
 * are diagnostic text and must not be parsed as a stable machine interface. */
static inline bool robot_messages_runtime_result_ok(const robot_messages_runtime_result_t *result) {
  return result != NULL && result->domain == ROBOT_MESSAGES_RUNTIME_OK;
}

const char *robot_messages_runtime_result_str(const robot_messages_runtime_result_t *result);

static inline const robot_messages_runtime_retained_detail_t *robot_messages_runtime_result_retained_detail(const robot_messages_runtime_result_t *result) {
  return result != NULL && result->detail_kind == ROBOT_MESSAGES_RUNTIME_DETAIL_RETAINED ? &result->detail.retained : NULL;
}

#define ROBOT_MESSAGES_RUNTIME_HAS_MANAGED_RPC 0
/* The typed value remains borrowed until the matching release. */
typedef struct {
  const arm_command_t *value;
  uint32_t generation;
  wl_latest_view_t lease;
} robot_messages_arm_command_latest_view_t;

/* The typed value remains borrowed until the matching release. */
typedef struct {
  const hand_command_t *value;
  uint32_t generation;
  wl_latest_view_t lease;
} robot_messages_hand_command_latest_view_t;

typedef struct {
  uint8_t _reserved;
  wl_latest_t *arm_command_latest;
  wl_latest_t *hand_command_latest;
} robot_messages_runtime_t;

typedef void (*robot_messages_runtime_result_fn)(void *user_data, const robot_messages_runtime_result_t *result);

typedef struct {
  robot_messages_runtime_t *runtime;
  void *user_data;
  robot_messages_runtime_result_fn on_result;
} robot_messages_runtime_pump_t;

/* Static runtime assembly. requirements() validates every sizing field and
 * reports the exact caller-owned byte storage needed by init(). Configuration
 * and storage descriptors may be temporary; instance and storage must outlive
 * all runtime activity and must not be copied after successful initialization. */
typedef struct {
  uint8_t _reserved;
  uint32_t arm_command_latest_initial_generation;
  uint32_t hand_command_latest_initial_generation;
} robot_messages_runtime_config_t;

#define ROBOT_MESSAGES_RUNTIME_HAS_DEFAULT_STORAGE 1
typedef union {
  uint8_t byte;
  arm_command_t arm_command_latest;
  hand_command_t hand_command_latest;
} robot_messages_runtime_default_storage_alignment_t;

#if defined(__cplusplus)
#define ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT alignof(robot_messages_runtime_default_storage_alignment_t)
#elif defined(_MSC_VER)
#define ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT __alignof(robot_messages_runtime_default_storage_alignment_t)
#else
#define ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT _Alignof(robot_messages_runtime_default_storage_alignment_t)
#endif
#define ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_CAPACITY \
  (1U + \
   ((ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT - 1U) + ((sizeof(arm_command_t) + (ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT - 1U)) * WL_LATEST_SLOT_COUNT)) + \
   ((ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT - 1U) + ((sizeof(hand_command_t) + (ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_ALIGNMENT - 1U)) * WL_LATEST_SLOT_COUNT)))

typedef union {
  robot_messages_runtime_default_storage_alignment_t alignment;
  uint8_t bytes[ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_CAPACITY];
} robot_messages_runtime_default_storage_t;

typedef struct {
  size_t storage_size;
  size_t storage_alignment;
} robot_messages_runtime_requirements_t;

typedef struct {
  void *data;
  size_t size;
} robot_messages_runtime_storage_t;

typedef struct {
  robot_messages_runtime_t runtime;
  wl_latest_t arm_command_latest;
  wl_latest_t hand_command_latest;
} robot_messages_runtime_instance_t;

typedef int32_t robot_messages_runtime_init_issue_t;
enum {
  ROBOT_MESSAGES_RUNTIME_INIT_OK = 0,
  ROBOT_MESSAGES_RUNTIME_INIT_NULL_ARGUMENT,
  ROBOT_MESSAGES_RUNTIME_INIT_ROLE_ENABLE,
  ROBOT_MESSAGES_RUNTIME_INIT_RETAINED_CAPACITY,
  ROBOT_MESSAGES_RUNTIME_INIT_RPC_CLIENT_CAPACITY,
  ROBOT_MESSAGES_RUNTIME_INIT_RPC_SERVER_CAPACITY,
  ROBOT_MESSAGES_RUNTIME_INIT_RPC_TIMEOUT,
  ROBOT_MESSAGES_RUNTIME_INIT_RPC_CACHE_POLICY,
  ROBOT_MESSAGES_RUNTIME_INIT_LAYOUT_OVERFLOW,
  ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_TOO_SMALL,
  ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_NULL,
  ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_ALIGNMENT,
  ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_OVERLAP,
  ROBOT_MESSAGES_RUNTIME_INIT_COMPONENT
};

typedef struct {
  robot_messages_runtime_init_issue_t issue;
  const char *field;
  size_t required;
  size_t provided;
} robot_messages_runtime_init_diagnostic_t;

const char *robot_messages_runtime_init_issue_str(robot_messages_runtime_init_issue_t issue);

/* Mechanical defaults use one FIFO/RPC slot, generation/operation ID one,
 * bounded encoded maxima, disabled roles, zero timeouts, and reject-new cache.
 * Override policy fields after this call. */
wl_err_t robot_messages_runtime_config_defaults(robot_messages_runtime_config_t *config);

robot_messages_runtime_storage_t robot_messages_runtime_default_storage_descriptor(robot_messages_runtime_default_storage_t *storage);
int robot_messages_runtime_requirements(const robot_messages_runtime_config_t *config, robot_messages_runtime_requirements_t *out_requirements);
/* Checked initialization reports the exact rejected field and capacity values. */
int robot_messages_runtime_init_checked(robot_messages_runtime_instance_t *instance, const robot_messages_runtime_config_t *config, const robot_messages_runtime_storage_t *storage, robot_messages_runtime_init_diagnostic_t *out_diagnostic);
int robot_messages_runtime_init(robot_messages_runtime_instance_t *instance, const robot_messages_runtime_config_t *config, const robot_messages_runtime_storage_t *storage);
/* With non-null ctx/event every RX outcome is consumed. Matching RPC TX
 * terminal events advance the runtime and reclaim the handle. Inspect
 * result.event_consumed before applying a fallback owner action. */
robot_messages_runtime_result_t robot_messages_runtime_dispatch_event(wl_ctx_t *ctx, const wl_event_t *event, robot_messages_runtime_t *runtime, wl_time_ms_t now_ms);

int robot_messages_arm_command_latest_acquire(robot_messages_runtime_t *runtime, robot_messages_arm_command_latest_view_t *out_view);
int robot_messages_arm_command_latest_release(robot_messages_runtime_t *runtime, robot_messages_arm_command_latest_view_t *view);

int robot_messages_hand_command_latest_acquire(robot_messages_runtime_t *runtime, robot_messages_hand_command_latest_view_t *out_view);
int robot_messages_hand_command_latest_release(robot_messages_runtime_t *runtime, robot_messages_hand_command_latest_view_t *view);

/* Build pump hooks that dispatch events with the owner's time sample. RPC
 * profiles also service one queued response per pass and merge their deadline.
 * on_result may be null; result pointers are borrowed only for the callback. */
wl_err_t robot_messages_runtime_pump_init(robot_messages_runtime_pump_t *pump, robot_messages_runtime_t *runtime, robot_messages_runtime_result_fn on_result, void *user_data);
wl_pump_hooks_t robot_messages_runtime_pump_hooks(robot_messages_runtime_pump_t *pump);

/* Default endpoint assembly. Members named private_state are not application
 * API. Zero-initialize once, keep at a stable address, close before reuse.
 * All profile-selected messages must have finite one-frame bounds. */
#define ROBOT_MESSAGES_HAS_DEFAULT_ENDPOINT 1
#define ROBOT_MESSAGES_ENDPOINT_MAX_PAYLOAD 292U
/* Layout is fixed by the local profile; CRC32C bounds also cover smaller CRCs. */
#define ROBOT_MESSAGES_ENDPOINT_RAW_CAPACITY (ROBOT_MESSAGES_ENDPOINT_MAX_PAYLOAD + WL_FRAME_HEADER_SIZE + WL_FRAME_MAX_CRC)
#define ROBOT_MESSAGES_ENDPOINT_UNIT_CAPACITY ROBOT_MESSAGES_ENDPOINT_RAW_CAPACITY
#define ROBOT_MESSAGES_ENDPOINT_CONTROL_CAPACITY (WL_FRAME_HEADER_SIZE + WL_FRAME_MAX_CRC)
#ifndef ROBOT_MESSAGES_ENDPOINT_RX_FIFO_CAPACITY
#define ROBOT_MESSAGES_ENDPOINT_RX_FIFO_CAPACITY 0U
#endif
#define ROBOT_MESSAGES_ENDPOINT_RUNTIME_CAPACITY ROBOT_MESSAGES_RUNTIME_DEFAULT_STORAGE_CAPACITY

typedef struct {
  wl_config_t link;
  wl_environment_t environment;
  /* Expert policy/storage overrides; ordinary applications use defaults. */
  robot_messages_runtime_config_t advanced;



  size_t event_budget;
  robot_messages_runtime_result_fn on_result;
  /* Shared context for on_result and ordinary on_<rpc> handlers. Advanced
   * deferred handlers and per-call completion contexts remain explicit. */
  void *user_data;
} robot_messages_endpoint_config_t;

typedef struct {
  struct {
    wl_endpoint_t owner;
    wl_allocator_t allocator; /* Zero for caller-owned static storage. */
    bool stepping;
    bool closing;
    uint64_t previous_session;
    robot_messages_runtime_instance_t instance;
    union {
      robot_messages_runtime_default_storage_alignment_t alignment;
      uint8_t bytes[ROBOT_MESSAGES_ENDPOINT_RUNTIME_CAPACITY];
    } arena;
    robot_messages_runtime_pump_t pump;
    robot_messages_runtime_result_t result;
    robot_messages_runtime_result_fn on_result;
    void *user_data;
    size_t event_budget;

    uint8_t tx_payload[ROBOT_MESSAGES_ENDPOINT_MAX_PAYLOAD];
    uint8_t tx_unit[ROBOT_MESSAGES_ENDPOINT_UNIT_CAPACITY];
    uint8_t control_unit[ROBOT_MESSAGES_ENDPOINT_CONTROL_CAPACITY];
    uint8_t rx_fallback[ROBOT_MESSAGES_ENDPOINT_UNIT_CAPACITY];

  } private_state;
} robot_messages_endpoint_t;

#if defined(__cplusplus)
#define ROBOT_MESSAGES_ENDPOINT_ALIGNMENT alignof(robot_messages_endpoint_t)
#elif defined(_MSC_VER)
#define ROBOT_MESSAGES_ENDPOINT_ALIGNMENT __alignof(robot_messages_endpoint_t)
#else
#define ROBOT_MESSAGES_ENDPOINT_ALIGNMENT _Alignof(robot_messages_endpoint_t)
#endif

/* Config descriptors can be temporary; callbacks/user_data must outlive use.
 * Ordinary client capability and registered server handlers are assembled by init. */
static inline wl_err_t robot_messages_endpoint_config_defaults(
    robot_messages_endpoint_config_t *config, wl_environment_t environment) {
  int result;
  if (config == NULL) return WL_ERR_INVALID_ARG;
  memset(config, 0, sizeof(*config));
  config->link.max_payload_len = ROBOT_MESSAGES_ENDPOINT_MAX_PAYLOAD;
  config->link.envelope = WL_ENVELOPE_NATIVE_PACKET;
  config->link.integrity = WL_INTEGRITY_CRC32C;
  config->environment = environment;
  config->link.ack_timeout_ms = 100U;
  config->link.max_retries = 4U;
  config->event_budget = 16U;
  result = robot_messages_runtime_config_defaults(&config->advanced);
  if (result != WL_OK) return result;

  return WL_OK;
}

static inline wl_endpoint_t *robot_messages_endpoint_handle(robot_messages_endpoint_t *endpoint) {
  return endpoint != NULL ? &endpoint->private_state.owner : NULL;
}

/* Advanced integration only: do not also dispatch/take events consumed by step. */
static inline robot_messages_runtime_t *robot_messages_endpoint_runtime(robot_messages_endpoint_t *endpoint) {
  return endpoint != NULL && wl_endpoint_link(robot_messages_endpoint_handle(endpoint)) != NULL
      ? &endpoint->private_state.instance.runtime : NULL;
}

static inline void robot_messages_endpoint_record(void *context,
                                      const robot_messages_runtime_result_t *result) {
  robot_messages_endpoint_t *endpoint = (robot_messages_endpoint_t *)context;
  robot_messages_runtime_result_t terminal;
  if (result->domain == ROBOT_MESSAGES_RUNTIME_NON_RX) {
    if (result->event_type != WL_EVT_TX_TIMEOUT && result->event_type != WL_EVT_TX_FAILED)
      return; /* Normal transport completion is not an application failure. */
    terminal = *result;
    terminal.domain = ROBOT_MESSAGES_RUNTIME_CORE_ERROR;
    result = &terminal;
  }
#if ROBOT_MESSAGES_RUNTIME_HAS_MANAGED_RPC
  /* Cached replies retain their reservation and retry ordinary link pressure.
   * Per-call failure/deadline remains observable through completion. */
  if (result->domain == ROBOT_MESSAGES_RUNTIME_CORE_ERROR && result->detail_kind == ROBOT_MESSAGES_RUNTIME_DETAIL_RPC &&
      (result->detail.rpc.core_result == WL_ERR_BUSY ||
       result->detail.rpc.core_result == WL_ERR_WOULD_BLOCK ||
       result->detail.rpc.core_result == WL_ERR_QUEUE_FULL ||
       result->detail.rpc.core_result == WL_ERR_NO_SPACE)) return;
#endif
  /* Retain the first failure even if later events in the same pass succeed. */
  if (robot_messages_runtime_result_ok(&endpoint->private_state.result))
    endpoint->private_state.result = *result;
  if (endpoint->private_state.on_result != NULL)
    endpoint->private_state.on_result(endpoint->private_state.user_data, result);
}

static inline wl_err_t robot_messages_endpoint_init_config(
    robot_messages_endpoint_t *endpoint, const robot_messages_endpoint_config_t *config) {
  wl_storage_t link_storage;
  robot_messages_runtime_storage_t storage;
  wl_pump_hooks_t hooks;
  robot_messages_runtime_config_t runtime_config;
  wl_config_t link_config;
  int result;
  if (endpoint == NULL || config == NULL || config->event_budget == 0U)
    return WL_ERR_INVALID_ARG;
  if (endpoint->private_state.stepping || endpoint->private_state.closing)
    return WL_ERR_REENTRANT;
  if (wl_endpoint_link(robot_messages_endpoint_handle(endpoint)) != NULL)
    return WL_ERR_INVALID_STATE;
  if (config->environment.clock.now_ms == NULL || config->link.session_id != 0U)
    return WL_ERR_INVALID_ARG;
  if (config->link.envelope != WL_ENVELOPE_NATIVE_PACKET) return WL_ERR_NOT_SUPPORTED;
  runtime_config = config->advanced;


  link_config = config->link;
  result = wl_session_next(config->environment.session,
      endpoint->private_state.previous_session, &link_config.session_id);
  if (result != WL_OK) return result;

  memset(&link_storage, 0, sizeof(link_storage));
  link_storage.tx_payload = endpoint->private_state.tx_payload;
  link_storage.tx_payload_size = sizeof(endpoint->private_state.tx_payload);
  link_storage.tx_unit = endpoint->private_state.tx_unit;
  link_storage.tx_unit_size = sizeof(endpoint->private_state.tx_unit);
  link_storage.control_unit = endpoint->private_state.control_unit;
  link_storage.control_unit_size = sizeof(endpoint->private_state.control_unit);
  link_storage.rx_fallback = endpoint->private_state.rx_fallback;
  link_storage.rx_fallback_size = sizeof(endpoint->private_state.rx_fallback);

  storage.data = endpoint->private_state.arena.bytes;
  storage.size = sizeof(endpoint->private_state.arena.bytes);
  result = robot_messages_runtime_init(&endpoint->private_state.instance, &runtime_config, &storage);
  if (result != WL_OK) return result;



  result = robot_messages_runtime_pump_init(&endpoint->private_state.pump,
      &endpoint->private_state.instance.runtime, robot_messages_endpoint_record, endpoint);
  if (result != WL_OK) return result;
  hooks = robot_messages_runtime_pump_hooks(&endpoint->private_state.pump);
  result = wl_endpoint_init(&endpoint->private_state.owner, &link_config,
                            &link_storage, &config->environment.clock, &hooks);
  if (result != WL_OK) return result;
  endpoint->private_state.previous_session = link_config.session_id;

  endpoint->private_state.on_result = config->on_result;
  endpoint->private_state.user_data = config->user_data;
  endpoint->private_state.event_budget = config->event_budget;
  memset(&endpoint->private_state.result, 0, sizeof(endpoint->private_state.result));
  return WL_OK;
}

static inline wl_err_t robot_messages_endpoint_init(robot_messages_endpoint_t *endpoint,
                                        wl_environment_t environment) {
  robot_messages_endpoint_config_t config;
  int result = robot_messages_endpoint_config_defaults(&config, environment);
  return result == WL_OK ? robot_messages_endpoint_init_config(endpoint, &config) : result;
}

/* One bounded owner pass: service transport, dispatch and release events,
 * advance RPC work. NO_DATA/backpressure during transport service is normal.
 * Inspect endpoint_result/last_step for details when this returns an error. */
static inline wl_err_t robot_messages_endpoint_step(robot_messages_endpoint_t *endpoint) {
  int result;
  if (endpoint == NULL) return WL_ERR_INVALID_ARG;
  if (endpoint->private_state.stepping || endpoint->private_state.closing) return WL_ERR_REENTRANT;
  endpoint->private_state.stepping = true;
  memset(&endpoint->private_state.result, 0, sizeof(endpoint->private_state.result));
  result = wl_endpoint_step(&endpoint->private_state.owner,
                             endpoint->private_state.event_budget);
  endpoint->private_state.stepping = false;
  if (result != WL_OK) return result;

  return robot_messages_runtime_result_ok(&endpoint->private_state.result)
      ? WL_OK : WL_ERR_INVALID_STATE;
}

static inline const robot_messages_runtime_result_t *robot_messages_endpoint_result(const robot_messages_endpoint_t *endpoint) {
  return endpoint != NULL ? &endpoint->private_state.result : NULL;
}

static inline wl_err_t robot_messages_endpoint_close(robot_messages_endpoint_t *endpoint) {
  if (endpoint == NULL) return WL_ERR_INVALID_ARG;
  if (endpoint->private_state.stepping || endpoint->private_state.closing) return WL_ERR_REENTRANT;
  endpoint->private_state.closing = true;
  wl_endpoint_close(robot_messages_endpoint_handle(endpoint));

  endpoint->private_state.closing = false;
  return WL_OK;
}

static inline wl_err_t robot_messages_endpoint_driver_step(void *context) {
  return robot_messages_endpoint_step((robot_messages_endpoint_t *)context);
}
static inline wl_err_t robot_messages_endpoint_driver_close(void *context) {
  return robot_messages_endpoint_close((robot_messages_endpoint_t *)context);
}
/* Setup-only platform integration; ordinary code does not drive both objects. */
static inline wl_endpoint_driver_t robot_messages_endpoint_driver(robot_messages_endpoint_t *endpoint) {
  wl_endpoint_driver_t driver;
  driver.endpoint = robot_messages_endpoint_handle(endpoint);
  driver.context = endpoint;
  driver.step = robot_messages_endpoint_driver_step;
  driver.close = robot_messages_endpoint_driver_close;
  driver.readiness_complete = 1U;
  return driver;
}

/* Optional creation: one allocation for the complete endpoint and all protocol
 * storage. *out must be NULL; failure leaves it unchanged. No global allocator
 * or fallback heap. Configure transport/waiting after successful creation. */
static inline wl_err_t robot_messages_endpoint_create(robot_messages_endpoint_t **out,
    const robot_messages_endpoint_config_t *config, const wl_allocator_t *allocator) {
  robot_messages_endpoint_t *endpoint;
  wl_allocator_t storage;
  int error;
  if (out == NULL || *out != NULL || config == NULL || allocator == NULL ||
      allocator->allocate == NULL || allocator->deallocate == NULL) return WL_ERR_INVALID_ARG;
  storage = *allocator;
  endpoint = (robot_messages_endpoint_t *)storage.allocate(storage.context,
      sizeof(*endpoint), ROBOT_MESSAGES_ENDPOINT_ALIGNMENT);
  if (endpoint == NULL) return WL_ERR_NO_MEM;
  if ((uintptr_t)endpoint % ROBOT_MESSAGES_ENDPOINT_ALIGNMENT != 0U) {
    storage.deallocate(storage.context, endpoint, sizeof(*endpoint), ROBOT_MESSAGES_ENDPOINT_ALIGNMENT);
    return WL_ERR_INVALID_ARG;
  }
  memset(endpoint, 0, sizeof(*endpoint));
  error = robot_messages_endpoint_init_config(endpoint, config);
  if (error != WL_OK) {
    (void)robot_messages_endpoint_close(endpoint);
    storage.deallocate(storage.context, endpoint, sizeof(*endpoint), ROBOT_MESSAGES_ENDPOINT_ALIGNMENT);
    return error;
  }
  endpoint->private_state.allocator = storage;
  *out = endpoint;
  return WL_OK;
}

/* Owner safe point, after background executor stop and all caller joins.
 * Close/quiesce and finish callbacks before freeing. Reentrant failure leaves
 * the pointer alive. Static storage must use close, not destroy. NULL is a no-op.
 * Copies of this owning pointer are invalid after successful destruction. */
static inline wl_err_t robot_messages_endpoint_destroy(robot_messages_endpoint_t **owner) {
  robot_messages_endpoint_t *endpoint;
  wl_allocator_t storage;
  int error;
  if (owner == NULL) return WL_ERR_INVALID_ARG;
  endpoint = *owner;
  if (endpoint == NULL) return WL_OK;
  storage = endpoint->private_state.allocator;
  if (storage.deallocate == NULL) return WL_ERR_INVALID_STATE;
  error = robot_messages_endpoint_close(endpoint);
  if (error != WL_OK) return error;
  *owner = NULL;
  storage.deallocate(storage.context, endpoint, sizeof(*endpoint), ROBOT_MESSAGES_ENDPOINT_ALIGNMENT);
  return WL_OK;
}
/* Delivery follows this binding. Use codec sends to override explicitly. */
static inline robot_messages_send_result_t robot_messages_endpoint_send_arm_command(robot_messages_endpoint_t *endpoint, const arm_command_t *message) {
  wl_time_ms_t now_ms = 0U;
  return robot_messages_arm_command_send(wl_endpoint_link(robot_messages_endpoint_handle(endpoint)), message, WL_DELIVERY_UNRELIABLE, now_ms);
}

/* Copy an owned value and release its lease internally. NO_DATA leaves out unchanged. */
static inline wl_err_t robot_messages_endpoint_read_arm_command(robot_messages_endpoint_t *endpoint, arm_command_t *out) {
  robot_messages_arm_command_latest_view_t view;
  robot_messages_runtime_t *runtime = robot_messages_endpoint_runtime(endpoint);
  int result;
  if (out == NULL) return WL_ERR_INVALID_ARG;
  if (runtime == NULL) return WL_ERR_NOT_INITIALIZED;
  result = robot_messages_arm_command_latest_acquire(runtime, &view);
  if (result != WL_OK) return result;
  *out = *view.value;
  return robot_messages_arm_command_latest_release(runtime, &view);
}

/* Delivery follows this binding. Use codec sends to override explicitly. */
static inline robot_messages_send_result_t robot_messages_endpoint_send_hand_command(robot_messages_endpoint_t *endpoint, const hand_command_t *message) {
  wl_time_ms_t now_ms = 0U;
  return robot_messages_hand_command_send(wl_endpoint_link(robot_messages_endpoint_handle(endpoint)), message, WL_DELIVERY_UNRELIABLE, now_ms);
}

/* Copy an owned value and release its lease internally. NO_DATA leaves out unchanged. */
static inline wl_err_t robot_messages_endpoint_read_hand_command(robot_messages_endpoint_t *endpoint, hand_command_t *out) {
  robot_messages_hand_command_latest_view_t view;
  robot_messages_runtime_t *runtime = robot_messages_endpoint_runtime(endpoint);
  int result;
  if (out == NULL) return WL_ERR_INVALID_ARG;
  if (runtime == NULL) return WL_ERR_NOT_INITIALIZED;
  result = robot_messages_hand_command_latest_acquire(runtime, &view);
  if (result != WL_OK) return result;
  *out = *view.value;
  return robot_messages_hand_command_latest_release(runtime, &view);
}

/* Delivery follows this binding. Use codec sends to override explicitly. */
static inline robot_messages_send_result_t robot_messages_endpoint_send_arm_state(robot_messages_endpoint_t *endpoint, const arm_state_t *message) {
  wl_time_ms_t now_ms = 0U;
  return robot_messages_arm_state_send(wl_endpoint_link(robot_messages_endpoint_handle(endpoint)), message, WL_DELIVERY_UNRELIABLE, now_ms);
}

/* Delivery follows this binding. Use codec sends to override explicitly. */
static inline robot_messages_send_result_t robot_messages_endpoint_send_hand_state(robot_messages_endpoint_t *endpoint, const hand_state_t *message) {
  wl_time_ms_t now_ms = 0U;
  return robot_messages_hand_state_send(wl_endpoint_link(robot_messages_endpoint_handle(endpoint)), message, WL_DELIVERY_UNRELIABLE, now_ms);
}

/* Delivery follows this binding. Use codec sends to override explicitly. */
static inline robot_messages_send_result_t robot_messages_endpoint_send_fault_snapshot(robot_messages_endpoint_t *endpoint, const fault_snapshot_t *message) {
  wl_time_ms_t now_ms = 0U;
  return robot_messages_fault_snapshot_send(wl_endpoint_link(robot_messages_endpoint_handle(endpoint)), message, WL_DELIVERY_UNRELIABLE, now_ms);
}

/* Delivery follows this binding. Use codec sends to override explicitly. */
static inline robot_messages_send_result_t robot_messages_endpoint_send_diagnostics(robot_messages_endpoint_t *endpoint, const diagnostics_t *message) {
  wl_time_ms_t now_ms = 0U;
  return robot_messages_diagnostics_send(wl_endpoint_link(robot_messages_endpoint_handle(endpoint)), message, WL_DELIVERY_UNRELIABLE, now_ms);
}

#ifdef __cplusplus
}
#endif

#endif
