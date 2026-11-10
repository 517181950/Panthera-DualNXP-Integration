#include "robot_messages_runtime.h"

#include <string.h>

static robot_messages_runtime_result_t robot_messages_runtime_result(const wl_event_t *event) {
  robot_messages_runtime_result_t result = {0};
  result.domain = ROBOT_MESSAGES_RUNTIME_INVALID_ARGUMENT;
  if (event != NULL) {
    result.message_id = event->message_id;
    result.event_type = event->type;
  }
  return result;
}

const char *robot_messages_runtime_result_str(const robot_messages_runtime_result_t *result) {
  if (result == NULL) return "null result";
  switch (result->domain) {
    case ROBOT_MESSAGES_RUNTIME_OK: return "ok";
    case ROBOT_MESSAGES_RUNTIME_NON_RX: return "non-rx event";
    case ROBOT_MESSAGES_RUNTIME_UNKNOWN_MESSAGE: return "unknown message";
    case ROBOT_MESSAGES_RUNTIME_MISSING_ROUTE: return "missing route";
    case ROBOT_MESSAGES_RUNTIME_MISSING_SCRATCH: return "missing scratch";
    case ROBOT_MESSAGES_RUNTIME_DELIVERY_MISMATCH: return "delivery mismatch";
    case ROBOT_MESSAGES_RUNTIME_CODEC_ERROR: return "codec error";
    case ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR: return "storage error";
    case ROBOT_MESSAGES_RUNTIME_RPC_ERROR: return "rpc error";
    case ROBOT_MESSAGES_RUNTIME_CORE_ERROR: return "core error";
    case ROBOT_MESSAGES_RUNTIME_APPLICATION_ERROR: return "application error";
    case ROBOT_MESSAGES_RUNTIME_INVALID_ARGUMENT: return "invalid argument";
    default: return "unknown runtime result";
  }
}

wl_err_t robot_messages_runtime_config_defaults(robot_messages_runtime_config_t *config) {
  if (config == NULL) return WL_ERR_INVALID_ARG;
  memset(config, 0, sizeof(*config));
  config->arm_command_latest_initial_generation = 1U;
  config->hand_command_latest_initial_generation = 1U;
  return WL_OK;
}

robot_messages_runtime_storage_t robot_messages_runtime_default_storage_descriptor(robot_messages_runtime_default_storage_t *storage) {
  robot_messages_runtime_storage_t descriptor = {0};
  if (storage != NULL) {
    descriptor.data = storage->bytes;
    descriptor.size = sizeof(storage->bytes);
  }
  return descriptor;
}

const char *robot_messages_runtime_init_issue_str(robot_messages_runtime_init_issue_t issue) {
  switch (issue) {
    case ROBOT_MESSAGES_RUNTIME_INIT_OK: return "ok";
    case ROBOT_MESSAGES_RUNTIME_INIT_NULL_ARGUMENT: return "null argument";
    case ROBOT_MESSAGES_RUNTIME_INIT_ROLE_ENABLE: return "role enable must be zero or one";
    case ROBOT_MESSAGES_RUNTIME_INIT_RETAINED_CAPACITY: return "retained capacity is zero";
    case ROBOT_MESSAGES_RUNTIME_INIT_RPC_CLIENT_CAPACITY: return "RPC client capacity is zero";
    case ROBOT_MESSAGES_RUNTIME_INIT_RPC_SERVER_CAPACITY: return "RPC server capacity is zero";
    case ROBOT_MESSAGES_RUNTIME_INIT_RPC_TIMEOUT: return "RPC timeout exceeds wrap-safe range";
    case ROBOT_MESSAGES_RUNTIME_INIT_RPC_CACHE_POLICY: return "unknown RPC cache policy";
    case ROBOT_MESSAGES_RUNTIME_INIT_LAYOUT_OVERFLOW: return "runtime layout size overflow";
    case ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_TOO_SMALL: return "runtime storage is too small";
    case ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_NULL: return "runtime storage data is null";
    case ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_ALIGNMENT: return "runtime storage is misaligned";
    case ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_OVERLAP: return "runtime storage overlaps the instance";
    case ROBOT_MESSAGES_RUNTIME_INIT_COMPONENT: return "runtime component initialization failed";
    default: return "unknown runtime initialization issue";
  }
}

static int robot_messages_runtime_init_failure(robot_messages_runtime_init_diagnostic_t *diagnostic, robot_messages_runtime_init_issue_t issue, const char *field, size_t required, size_t provided, int result) {
  if (diagnostic != NULL) {
    diagnostic->issue = issue;
    diagnostic->field = field;
    diagnostic->required = required;
    diagnostic->provided = provided;
  }
  return result;
}

typedef struct {
  uint8_t *base;
  size_t size;
  size_t offset;
} robot_messages_runtime_storage_cursor_t;

typedef struct {
  void *arm_command_latest_storage;
  void *hand_command_latest_storage;
} robot_messages_runtime_layout_t;

static inline int robot_messages_runtime_storage_region(robot_messages_runtime_storage_cursor_t *cursor, size_t alignment, size_t count, size_t element_size, void **out_data, size_t *out_size) {
  size_t aligned;
  size_t region_size;
  if (cursor == NULL || alignment == 0U || (alignment & (alignment - 1U)) != 0U) return WL_ERR_INVALID_ARG;
  if (out_data != NULL) *out_data = NULL;
  if (out_size != NULL) *out_size = 0U;
  if (count != 0U && element_size > SIZE_MAX / count) return WL_ERR_INVALID_ARG;
  region_size = count * element_size;
  if (cursor->offset > SIZE_MAX - (alignment - 1U)) return WL_ERR_INVALID_ARG;
  aligned = (cursor->offset + (alignment - 1U)) & ~(alignment - 1U);
  if (region_size > SIZE_MAX - aligned) return WL_ERR_INVALID_ARG;
  if (aligned + region_size > cursor->size) return WL_ERR_BUF_TOO_SMALL;
  if (out_data != NULL && cursor->base != NULL) *out_data = cursor->base + aligned;
  if (out_size != NULL) *out_size = region_size;
  cursor->offset = aligned + region_size;
  return WL_OK;
}

static int robot_messages_runtime_layout(const robot_messages_runtime_config_t *config, uint8_t *base, size_t size, robot_messages_runtime_layout_t *out_layout, robot_messages_runtime_requirements_t *out_requirements) {
  robot_messages_runtime_storage_cursor_t cursor = {base, size, 0U};
  size_t alignment = 1U;
  int result;
  if (out_layout != NULL) memset(out_layout, 0, sizeof(*out_layout));
  if (out_requirements != NULL) memset(out_requirements, 0, sizeof(*out_requirements));
  if (config == NULL) return WL_ERR_INVALID_ARG;
  {
    const wl_latest_config_t route_config = {sizeof(arm_command_t), _Alignof(arm_command_t), config->arm_command_latest_initial_generation};
    wl_latest_requirements_t route_requirements;
    result = wl_latest_requirements(&route_config, &route_requirements);
    if (result != WL_OK) return result;
    if (alignment < _Alignof(arm_command_t)) alignment = _Alignof(arm_command_t);
    result = robot_messages_runtime_storage_region(&cursor, _Alignof(arm_command_t), 1U, route_requirements.storage_size, out_layout == NULL ? NULL : &out_layout->arm_command_latest_storage, NULL);
    if (result != WL_OK) return result;
  }
  {
    const wl_latest_config_t route_config = {sizeof(hand_command_t), _Alignof(hand_command_t), config->hand_command_latest_initial_generation};
    wl_latest_requirements_t route_requirements;
    result = wl_latest_requirements(&route_config, &route_requirements);
    if (result != WL_OK) return result;
    if (alignment < _Alignof(hand_command_t)) alignment = _Alignof(hand_command_t);
    result = robot_messages_runtime_storage_region(&cursor, _Alignof(hand_command_t), 1U, route_requirements.storage_size, out_layout == NULL ? NULL : &out_layout->hand_command_latest_storage, NULL);
    if (result != WL_OK) return result;
  }
  if (out_requirements != NULL) {
    out_requirements->storage_size = cursor.offset;
    out_requirements->storage_alignment = alignment;
  }
  return WL_OK;
}

int robot_messages_runtime_requirements(const robot_messages_runtime_config_t *config, robot_messages_runtime_requirements_t *out_requirements) {
  robot_messages_runtime_config_t config_copy;
  if (config == NULL || out_requirements == NULL) return WL_ERR_INVALID_ARG;
  config_copy = *config;
  *out_requirements = (robot_messages_runtime_requirements_t){0};
  return robot_messages_runtime_layout(&config_copy, NULL, SIZE_MAX, NULL, out_requirements);
}

static int robot_messages_runtime_init_validate(const robot_messages_runtime_instance_t *instance, const robot_messages_runtime_config_t *config, const robot_messages_runtime_storage_t *storage, robot_messages_runtime_requirements_t *requirements, robot_messages_runtime_init_diagnostic_t *diagnostic) {
  uintptr_t instance_address;
  uintptr_t storage_address;
  int result;
  if (diagnostic != NULL) memset(diagnostic, 0, sizeof(*diagnostic));
  if (instance == NULL || config == NULL || storage == NULL || requirements == NULL || diagnostic == NULL)
    return robot_messages_runtime_init_failure(diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_NULL_ARGUMENT, instance == NULL ? "instance" : config == NULL ? "config" : storage == NULL ? "storage" : requirements == NULL ? "requirements" : "diagnostic", 1U, 0U, WL_ERR_INVALID_ARG);
  result = robot_messages_runtime_requirements(config, requirements);
  if (result != WL_OK) return robot_messages_runtime_init_failure(diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_LAYOUT_OVERFLOW, "config", 0U, 0U, result);
  if (storage->size < requirements->storage_size) return robot_messages_runtime_init_failure(diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_TOO_SMALL, "storage.size", requirements->storage_size, storage->size, WL_ERR_BUF_TOO_SMALL);
  if (requirements->storage_size != 0U) {
    if (storage->data == NULL) return robot_messages_runtime_init_failure(diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_NULL, "storage.data", requirements->storage_size, 0U, WL_ERR_INVALID_ARG);
    if (((uintptr_t)storage->data & (requirements->storage_alignment - 1U)) != 0U) return robot_messages_runtime_init_failure(diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_ALIGNMENT, "storage.data", requirements->storage_alignment, (size_t)((uintptr_t)storage->data & (requirements->storage_alignment - 1U)), WL_ERR_INVALID_ARG);
    instance_address = (uintptr_t)(const void *)instance;
    storage_address = (uintptr_t)storage->data;
    if ((storage_address <= instance_address && instance_address - storage_address < requirements->storage_size) || (instance_address < storage_address && storage_address - instance_address < sizeof(*instance))) return robot_messages_runtime_init_failure(diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_STORAGE_OVERLAP, "storage.data", requirements->storage_size, storage->size, WL_ERR_INVALID_ARG);
  }
  return WL_OK;
}

int robot_messages_runtime_init(robot_messages_runtime_instance_t *instance, const robot_messages_runtime_config_t *config, const robot_messages_runtime_storage_t *storage) {
  robot_messages_runtime_config_t config_copy;
  robot_messages_runtime_storage_t storage_copy;
  robot_messages_runtime_requirements_t requirements;
  robot_messages_runtime_layout_t layout;
  uintptr_t instance_address;
  uintptr_t storage_address;
  int result;
  if (instance == NULL || config == NULL || storage == NULL) return WL_ERR_INVALID_ARG;
  config_copy = *config;
  storage_copy = *storage;
  config = &config_copy;
  storage = &storage_copy;
  result = robot_messages_runtime_requirements(config, &requirements);
  if (result != WL_OK) return result;
  if (storage->size < requirements.storage_size) return WL_ERR_BUF_TOO_SMALL;
  if (requirements.storage_size != 0U) {
    if (storage->data == NULL || ((uintptr_t)storage->data & (requirements.storage_alignment - 1U)) != 0U) return WL_ERR_INVALID_ARG;
    instance_address = (uintptr_t)(void *)instance;
    storage_address = (uintptr_t)storage->data;
    if ((storage_address <= instance_address && instance_address - storage_address < requirements.storage_size) || (instance_address < storage_address && storage_address - instance_address < sizeof(*instance))) return WL_ERR_INVALID_ARG;
  }
  result = robot_messages_runtime_layout(config, (uint8_t *)storage->data, storage->size, &layout, NULL);
  if (result != WL_OK) return result;
  memset(instance, 0, sizeof(*instance));
  {
    const wl_latest_config_t route_config = {sizeof(arm_command_t), _Alignof(arm_command_t), config->arm_command_latest_initial_generation};
    wl_latest_requirements_t route_requirements;
    wl_latest_storage_t route_storage;
    result = wl_latest_requirements(&route_config, &route_requirements);
    if (result != WL_OK) goto init_failed;
    route_storage.data = layout.arm_command_latest_storage;
    route_storage.size = route_requirements.storage_size;
    result = wl_latest_init(&instance->arm_command_latest, &route_config, &route_storage);
    if (result != WL_OK) goto init_failed;
    instance->runtime.arm_command_latest = &instance->arm_command_latest;
  }
  {
    const wl_latest_config_t route_config = {sizeof(hand_command_t), _Alignof(hand_command_t), config->hand_command_latest_initial_generation};
    wl_latest_requirements_t route_requirements;
    wl_latest_storage_t route_storage;
    result = wl_latest_requirements(&route_config, &route_requirements);
    if (result != WL_OK) goto init_failed;
    route_storage.data = layout.hand_command_latest_storage;
    route_storage.size = route_requirements.storage_size;
    result = wl_latest_init(&instance->hand_command_latest, &route_config, &route_storage);
    if (result != WL_OK) goto init_failed;
    instance->runtime.hand_command_latest = &instance->hand_command_latest;
  }
  return WL_OK;

init_failed:
  memset(instance, 0, sizeof(*instance));
  return result;
}

int robot_messages_runtime_init_checked(robot_messages_runtime_instance_t *instance, const robot_messages_runtime_config_t *config, const robot_messages_runtime_storage_t *storage, robot_messages_runtime_init_diagnostic_t *out_diagnostic) {
  robot_messages_runtime_requirements_t requirements;
  int result = robot_messages_runtime_init_validate(instance, config, storage, &requirements, out_diagnostic);
  if (result != WL_OK) return result;
  result = robot_messages_runtime_init(instance, config, storage);
  if (result != WL_OK) return robot_messages_runtime_init_failure(out_diagnostic, ROBOT_MESSAGES_RUNTIME_INIT_COMPONENT, "component", 0U, 0U, result);
  return WL_OK;
}

robot_messages_runtime_result_t robot_messages_runtime_dispatch_event(wl_ctx_t *ctx, const wl_event_t *event, robot_messages_runtime_t *runtime, wl_time_ms_t now_ms) {
  robot_messages_runtime_result_t result = robot_messages_runtime_result(event);
  if (event == NULL) return result;
  (void)now_ms;
  if (event->type != WL_EVT_UNRELIABLE_RX && event->type != WL_EVT_RELIABLE_RX) {
    result.domain = ROBOT_MESSAGES_RUNTIME_NON_RX;
    return result;
  }
  if (ctx == NULL) return result;
  if (runtime == NULL) goto release_event;

  switch (event->message_id) {
    case ARM_COMMAND_MESSAGE_ID: {
      wl_latest_write_claim_t claim = {0};
      result.detail_kind = ROBOT_MESSAGES_RUNTIME_DETAIL_RETAINED;
      if (event->type != WL_EVT_UNRELIABLE_RX) {
        result.domain = ROBOT_MESSAGES_RUNTIME_DELIVERY_MISMATCH;
        break;
      }
      if (runtime->arm_command_latest == NULL) {
        result.domain = ROBOT_MESSAGES_RUNTIME_MISSING_ROUTE;
        break;
      }
      result.detail.retained.storage_result = wl_latest_write_claim(runtime->arm_command_latest, &claim);
      if (result.detail.retained.storage_result != WL_OK) {
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      if (claim.value_size < sizeof(arm_command_t)) {
        result.detail.retained.storage_result = WL_ERR_BUF_TOO_SMALL;
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->arm_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      if (((uintptr_t)claim.value % _Alignof(arm_command_t)) != 0U) {
        result.detail.retained.storage_result = WL_ERR_INVALID_ARG;
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->arm_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      result.detail.retained.codec_status = arm_command_decode(event->payload, event->payload_len, (arm_command_t *)claim.value);
      if (result.detail.retained.codec_status != WL_CODEC_OK) {
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->arm_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_CODEC_ERROR;
        break;
      }
      result.detail.retained.storage_result = wl_latest_write_publish(runtime->arm_command_latest, &claim);
      if (result.detail.retained.storage_result != WL_OK) {
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->arm_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      result.domain = ROBOT_MESSAGES_RUNTIME_OK;
      break;
    }
    case HAND_COMMAND_MESSAGE_ID: {
      wl_latest_write_claim_t claim = {0};
      result.detail_kind = ROBOT_MESSAGES_RUNTIME_DETAIL_RETAINED;
      if (event->type != WL_EVT_UNRELIABLE_RX) {
        result.domain = ROBOT_MESSAGES_RUNTIME_DELIVERY_MISMATCH;
        break;
      }
      if (runtime->hand_command_latest == NULL) {
        result.domain = ROBOT_MESSAGES_RUNTIME_MISSING_ROUTE;
        break;
      }
      result.detail.retained.storage_result = wl_latest_write_claim(runtime->hand_command_latest, &claim);
      if (result.detail.retained.storage_result != WL_OK) {
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      if (claim.value_size < sizeof(hand_command_t)) {
        result.detail.retained.storage_result = WL_ERR_BUF_TOO_SMALL;
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->hand_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      if (((uintptr_t)claim.value % _Alignof(hand_command_t)) != 0U) {
        result.detail.retained.storage_result = WL_ERR_INVALID_ARG;
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->hand_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      result.detail.retained.codec_status = hand_command_decode(event->payload, event->payload_len, (hand_command_t *)claim.value);
      if (result.detail.retained.codec_status != WL_CODEC_OK) {
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->hand_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_CODEC_ERROR;
        break;
      }
      result.detail.retained.storage_result = wl_latest_write_publish(runtime->hand_command_latest, &claim);
      if (result.detail.retained.storage_result != WL_OK) {
        result.detail.retained.abort_result = wl_latest_write_abort(runtime->hand_command_latest, &claim);
        result.domain = ROBOT_MESSAGES_RUNTIME_STORAGE_ERROR;
        break;
      }
      result.domain = ROBOT_MESSAGES_RUNTIME_OK;
      break;
    }
    default:
      result.domain = ROBOT_MESSAGES_RUNTIME_UNKNOWN_MESSAGE;
      break;
  }

release_event:
  wl_event_release(ctx, event);
  result.event_consumed = 1U;
  return result;
}

int robot_messages_arm_command_latest_acquire(robot_messages_runtime_t *runtime, robot_messages_arm_command_latest_view_t *out_view) {
  wl_latest_view_t lease = {0};
  int result;
  if (out_view != NULL) memset(out_view, 0, sizeof(*out_view));
  if (runtime == NULL || out_view == NULL) return WL_ERR_INVALID_ARG;
  if (runtime->arm_command_latest == NULL) return WL_ERR_NOT_INITIALIZED;
  result = wl_latest_read_acquire(runtime->arm_command_latest, &lease);
  if (result != WL_OK) return result;
  if (lease.value == NULL || lease.value_size < sizeof(arm_command_t) || ((uintptr_t)lease.value % _Alignof(arm_command_t)) != 0U) {
    int failure = lease.value_size < sizeof(arm_command_t) ? WL_ERR_BUF_TOO_SMALL : WL_ERR_INVALID_STATE;
    int release_result = wl_latest_read_release(runtime->arm_command_latest, &lease);
    if (release_result != WL_OK) return release_result;
    return failure;
  }
  out_view->value = (const arm_command_t *)lease.value;
  out_view->generation = lease.generation;
  out_view->lease = lease;
  return WL_OK;
}

int robot_messages_arm_command_latest_release(robot_messages_runtime_t *runtime, robot_messages_arm_command_latest_view_t *view) {
  int result;
  if (runtime == NULL || view == NULL) return WL_ERR_INVALID_ARG;
  if (runtime->arm_command_latest == NULL) return WL_ERR_NOT_INITIALIZED;
  if ((const void *)view->value != view->lease.value || view->generation != view->lease.generation) return WL_ERR_INVALID_STATE;
  result = wl_latest_read_release(runtime->arm_command_latest, &view->lease);
  if (result == WL_OK) memset(view, 0, sizeof(*view));
  return result;
}

int robot_messages_hand_command_latest_acquire(robot_messages_runtime_t *runtime, robot_messages_hand_command_latest_view_t *out_view) {
  wl_latest_view_t lease = {0};
  int result;
  if (out_view != NULL) memset(out_view, 0, sizeof(*out_view));
  if (runtime == NULL || out_view == NULL) return WL_ERR_INVALID_ARG;
  if (runtime->hand_command_latest == NULL) return WL_ERR_NOT_INITIALIZED;
  result = wl_latest_read_acquire(runtime->hand_command_latest, &lease);
  if (result != WL_OK) return result;
  if (lease.value == NULL || lease.value_size < sizeof(hand_command_t) || ((uintptr_t)lease.value % _Alignof(hand_command_t)) != 0U) {
    int failure = lease.value_size < sizeof(hand_command_t) ? WL_ERR_BUF_TOO_SMALL : WL_ERR_INVALID_STATE;
    int release_result = wl_latest_read_release(runtime->hand_command_latest, &lease);
    if (release_result != WL_OK) return release_result;
    return failure;
  }
  out_view->value = (const hand_command_t *)lease.value;
  out_view->generation = lease.generation;
  out_view->lease = lease;
  return WL_OK;
}

int robot_messages_hand_command_latest_release(robot_messages_runtime_t *runtime, robot_messages_hand_command_latest_view_t *view) {
  int result;
  if (runtime == NULL || view == NULL) return WL_ERR_INVALID_ARG;
  if (runtime->hand_command_latest == NULL) return WL_ERR_NOT_INITIALIZED;
  if ((const void *)view->value != view->lease.value || view->generation != view->lease.generation) return WL_ERR_INVALID_STATE;
  result = wl_latest_read_release(runtime->hand_command_latest, &view->lease);
  if (result == WL_OK) memset(view, 0, sizeof(*view));
  return result;
}

static wl_pump_event_disposition_t robot_messages_runtime_pump_event(void *user_data, wl_ctx_t *ctx, const wl_event_t *event, wl_time_ms_t now_ms) {
  robot_messages_runtime_pump_t *pump = (robot_messages_runtime_pump_t *)user_data;
  robot_messages_runtime_result_t result;
  if (pump == NULL || pump->runtime == NULL) return WL_PUMP_EVENT_UNHANDLED;
  result = robot_messages_runtime_dispatch_event(ctx, event, pump->runtime, now_ms);
  if (pump->on_result != NULL) pump->on_result(pump->user_data, &result);
  return result.event_consumed != 0U ? WL_PUMP_EVENT_CONSUMED : WL_PUMP_EVENT_UNHANDLED;
}

wl_err_t robot_messages_runtime_pump_init(robot_messages_runtime_pump_t *pump, robot_messages_runtime_t *runtime, robot_messages_runtime_result_fn on_result, void *user_data) {
  if (pump == NULL || runtime == NULL) return WL_ERR_INVALID_ARG;
  memset(pump, 0, sizeof(*pump));
  pump->runtime = runtime;
  pump->user_data = user_data;
  pump->on_result = on_result;
  return WL_OK;
}

wl_pump_hooks_t robot_messages_runtime_pump_hooks(robot_messages_runtime_pump_t *pump) {
  wl_pump_hooks_t hooks = {0};
  if (pump == NULL) return hooks;
  hooks.application_user_data = pump;
  hooks.on_event = robot_messages_runtime_pump_event;
  return hooks;
}
