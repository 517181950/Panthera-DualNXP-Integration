#ifndef WIRELINK_GENERATED_ROBOT_MESSAGES_H_VALUES
#define WIRELINK_GENERATED_ROBOT_MESSAGES_H_VALUES

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <wirelink/codec.h>
#include <float.h>

#if defined(__cplusplus)
static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && FLT_MIN_EXP == -125, "WLC float32 requires IEEE-754 binary32");
#else
_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && FLT_MIN_EXP == -125, "WLC float32 requires IEEE-754 binary32");
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Self-owning business values. Assignment copies all data; no destroy is needed.
 * String lengths are bytes (embedded NUL is allowed); data[length] is a
 * convenience terminator after clear/decode/from_view, not part of the wire.
 * Views borrow their source. Conversion input/output must not overlap.
 * Failed conversions/decodes leave output unchanged. */
#define STREAM_HEADER_HAS_VALUE 1
typedef struct {
  bool has_schema_revision;
  uint16_t schema_revision;
  bool has_source_node;
  uint16_t source_node;
  bool has_target_node;
  uint16_t target_node;
  bool has_source_boot_id;
  uint64_t source_boot_id;
  bool has_target_boot_id;
  uint64_t target_boot_id;
  bool has_sequence;
  uint32_t sequence;
  bool has_tx_monotonic_us;
  uint64_t tx_monotonic_us;
} stream_header_value_t;
#define STREAM_HEADER_VALUE_SIZE (sizeof(stream_header_value_t))
void stream_header_value_clear(stream_header_value_t *value);
size_t stream_header_value_encoded_size(const stream_header_value_t *value);
wl_codec_status_t stream_header_value_encode(const stream_header_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t stream_header_value_decode(const uint8_t *input, size_t length, stream_header_value_t *out);

#define ARM_COMMAND_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_control_lease;
  uint64_t control_lease;
  bool has_policy_revision;
  uint32_t policy_revision;
  bool has_valid_for_us;
  uint32_t valid_for_us;
  bool has_joint_mask;
  uint32_t joint_mask;
  bool has_mit;
  float mit[30];
} arm_command_value_t;
#define ARM_COMMAND_VALUE_SIZE (sizeof(arm_command_value_t))
void arm_command_value_clear(arm_command_value_t *value);
size_t arm_command_value_encoded_size(const arm_command_value_t *value);
wl_codec_status_t arm_command_value_encode(const arm_command_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t arm_command_value_decode(const uint8_t *input, size_t length, arm_command_value_t *out);

#define HAND_COMMAND_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_control_lease;
  uint64_t control_lease;
  bool has_policy_revision;
  uint32_t policy_revision;
  bool has_valid_for_us;
  uint32_t valid_for_us;
  bool has_target_q_rad;
  float target_q_rad[6];
  bool has_duration_ms;
  uint16_t duration_ms;
} hand_command_value_t;
#define HAND_COMMAND_VALUE_SIZE (sizeof(hand_command_value_t))
void hand_command_value_clear(hand_command_value_t *value);
size_t hand_command_value_encoded_size(const hand_command_value_t *value);
wl_codec_status_t hand_command_value_encode(const hand_command_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t hand_command_value_decode(const uint8_t *input, size_t length, hand_command_value_t *out);

#define ARM_STATE_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_last_applied_command_sequence;
  uint32_t last_applied_command_sequence;
  bool has_command_age_us;
  uint32_t command_age_us;
  bool has_policy_revision;
  uint32_t policy_revision;
  bool has_control_owner;
  uint8_t control_owner;
  bool has_run_state;
  uint8_t run_state;
  bool has_compensation_mode;
  uint8_t compensation_mode;
  bool has_online_mask;
  uint32_t online_mask;
  bool has_valid_q_mask;
  uint32_t valid_q_mask;
  bool has_valid_dq_mask;
  uint32_t valid_dq_mask;
  bool has_valid_tau_mask;
  uint32_t valid_tau_mask;
  bool has_feedback_age_us;
  uint32_t feedback_age_us[6];
  bool has_q_rad;
  float q_rad[6];
  bool has_dq_rad_s;
  float dq_rad_s[6];
  bool has_tau_measured_nm;
  float tau_measured_nm[6];
  bool has_fault_code_raw;
  uint32_t fault_code_raw[6];
  bool has_motor_mode_raw;
  uint32_t motor_mode_raw[6];
  bool has_motor_fault_valid_mask;
  uint32_t motor_fault_valid_mask;
  bool has_last_state_probe_sequence;
  uint32_t last_state_probe_sequence;
} arm_state_value_t;
#define ARM_STATE_VALUE_SIZE (sizeof(arm_state_value_t))
void arm_state_value_clear(arm_state_value_t *value);
size_t arm_state_value_encoded_size(const arm_state_value_t *value);
wl_codec_status_t arm_state_value_encode(const arm_state_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t arm_state_value_decode(const uint8_t *input, size_t length, arm_state_value_t *out);

#define HAND_STATE_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_last_applied_command_sequence;
  uint32_t last_applied_command_sequence;
  bool has_command_age_us;
  uint32_t command_age_us;
  bool has_online;
  bool online;
  bool has_valid_fields;
  uint32_t valid_fields;
  bool has_field_age_us;
  uint32_t field_age_us[5];
  bool has_motor_position_percent;
  float motor_position_percent[6];
  bool has_joint_q_rad;
  float joint_q_rad[11];
  bool has_motor_dq_rad_s;
  float motor_dq_rad_s[6];
  bool has_motor_current_ma;
  float motor_current_ma[6];
  bool has_fault_code_raw;
  uint32_t fault_code_raw[6];
  bool has_mode_raw;
  uint16_t mode_raw;
  bool has_fault_valid_mask;
  uint32_t fault_valid_mask;
  bool has_tactile_capable;
  bool tactile_capable;
  bool has_tactile_valid;
  bool tactile_valid;
} hand_state_value_t;
#define HAND_STATE_VALUE_SIZE (sizeof(hand_state_value_t))
void hand_state_value_clear(hand_state_value_t *value);
size_t hand_state_value_encoded_size(const hand_state_value_t *value);
wl_codec_status_t hand_state_value_encode(const hand_state_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t hand_state_value_decode(const uint8_t *input, size_t length, hand_state_value_t *out);

#define FAULT_SNAPSHOT_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_policy_revision;
  uint32_t policy_revision;
  bool has_mcu_fault_active;
  uint32_t mcu_fault_active;
  bool has_mcu_fault_latched;
  uint32_t mcu_fault_latched;
  bool has_mcu_fault_valid_mask;
  uint32_t mcu_fault_valid_mask;
  bool has_local_action;
  uint8_t local_action;
  bool has_arm_fault_raw;
  uint32_t arm_fault_raw[6];
  bool has_arm_fault_valid_mask;
  uint32_t arm_fault_valid_mask;
  bool has_hand_fault_raw;
  uint32_t hand_fault_raw[6];
  bool has_hand_fault_valid_mask;
  uint32_t hand_fault_valid_mask;
  bool has_latest_event_id;
  uint64_t latest_event_id;
  bool has_oldest_retained_event_id;
  uint64_t oldest_retained_event_id;
  bool has_event_overflow_count;
  uint32_t event_overflow_count;
} fault_snapshot_value_t;
#define FAULT_SNAPSHOT_VALUE_SIZE (sizeof(fault_snapshot_value_t))
void fault_snapshot_value_clear(fault_snapshot_value_t *value);
size_t fault_snapshot_value_encoded_size(const fault_snapshot_value_t *value);
wl_codec_status_t fault_snapshot_value_encode(const fault_snapshot_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t fault_snapshot_value_decode(const uint8_t *input, size_t length, fault_snapshot_value_t *out);

#define DIAGNOSTICS_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_can_tx_completed;
  uint32_t can_tx_completed;
  bool has_can_tx_failed;
  uint32_t can_tx_failed;
  bool has_can_rx_valid;
  uint32_t can_rx_valid;
  bool has_can_rx_unknown;
  uint32_t can_rx_unknown;
  bool has_can_rx_ambiguous;
  uint32_t can_rx_ambiguous;
  bool has_can_rx_malformed;
  uint32_t can_rx_malformed;
  bool has_can_bus_state;
  uint32_t can_bus_state;
  bool has_can_tx_error_count;
  uint32_t can_tx_error_count;
  bool has_can_rx_error_count;
  uint32_t can_rx_error_count;
  bool has_udp_rx_valid;
  uint32_t udp_rx_valid;
  bool has_udp_rx_bad;
  uint32_t udp_rx_bad;
  bool has_udp_rx_old;
  uint32_t udp_rx_old;
  bool has_udp_tx_drop;
  uint32_t udp_tx_drop;
  bool has_control_period_mean_us;
  uint32_t control_period_mean_us;
  bool has_control_period_max_us;
  uint32_t control_period_max_us;
  bool has_control_jitter_abs_max_us;
  uint32_t control_jitter_abs_max_us;
  bool has_command_coalesced;
  uint32_t command_coalesced;
  bool has_counter_valid_mask;
  uint32_t counter_valid_mask;
  bool has_motor_feedback_count;
  uint32_t motor_feedback_count[6];
  bool has_hand_field_feedback_count;
  uint32_t hand_field_feedback_count[5];
  bool has_state_probe_received;
  uint32_t state_probe_received;
  bool has_last_state_probe_sequence;
  uint32_t last_state_probe_sequence;
  bool has_hand_state_query_mode;
  uint8_t hand_state_query_mode;
  bool has_hand_mode_guard_phase;
  uint8_t hand_mode_guard_phase;
  bool has_hand_mode_guard_status;
  int32_t hand_mode_guard_status;
  bool has_hand_mode_guard_elapsed_us;
  uint32_t hand_mode_guard_elapsed_us;
  bool has_hand_mode_guard_token;
  uint32_t hand_mode_guard_token;
  bool has_hand_control_tx;
  uint32_t hand_control_tx;
  bool has_hand_control_ack;
  uint32_t hand_control_ack;
  bool has_hand_query_timeouts;
  uint32_t hand_query_timeouts;
  bool has_motor_control_tx;
  uint32_t motor_control_tx[6];
  bool has_hand_full_query_tx;
  uint32_t hand_full_query_tx;
  bool has_hand_control_negative_ack;
  uint32_t hand_control_negative_ack;
  bool has_hand_control_ack_latest_status;
  uint32_t hand_control_ack_latest_status;
} diagnostics_value_t;
#define DIAGNOSTICS_VALUE_SIZE (sizeof(diagnostics_value_t))
void diagnostics_value_clear(diagnostics_value_t *value);
size_t diagnostics_value_encoded_size(const diagnostics_value_t *value);
wl_codec_status_t diagnostics_value_encode(const diagnostics_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t diagnostics_value_decode(const uint8_t *input, size_t length, diagnostics_value_t *out);

#define FAULT_EVENT_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_event_id;
  uint64_t event_id;
  bool has_device_class;
  uint8_t device_class;
  bool has_device_id;
  uint16_t device_id;
  bool has_device_channel;
  uint8_t device_channel;
  bool has_raw_code;
  uint32_t raw_code;
  bool has_normalized_code;
  uint32_t normalized_code;
  bool has_severity;
  uint8_t severity;
  bool has_active;
  bool active;
  bool has_local_action;
  uint8_t local_action;
  bool has_policy_revision;
  uint32_t policy_revision;
  bool has_first_seen_us;
  uint64_t first_seen_us;
  bool has_last_seen_us;
  uint64_t last_seen_us;
  bool has_occurrences;
  uint32_t occurrences;
} fault_event_value_t;
#define FAULT_EVENT_VALUE_SIZE (sizeof(fault_event_value_t))
void fault_event_value_clear(fault_event_value_t *value);
size_t fault_event_value_encoded_size(const fault_event_value_t *value);
wl_codec_status_t fault_event_value_encode(const fault_event_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t fault_event_value_decode(const uint8_t *input, size_t length, fault_event_value_t *out);

#define OPERATION_REQUEST_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_operation_id;
  uint32_t operation_id;
  bool has_operation;
  uint16_t operation;
  bool has_control_lease;
  uint64_t control_lease;
  bool has_expected_policy_revision;
  uint32_t expected_policy_revision;
  bool has_target_mask;
  uint32_t target_mask;
  bool has_fault_event_id;
  uint64_t fault_event_id;
} operation_request_value_t;
#define OPERATION_REQUEST_VALUE_SIZE (sizeof(operation_request_value_t))
void operation_request_value_clear(operation_request_value_t *value);
size_t operation_request_value_encoded_size(const operation_request_value_t *value);
wl_codec_status_t operation_request_value_encode(const operation_request_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t operation_request_value_decode(const uint8_t *input, size_t length, operation_request_value_t *out);

#define OPERATION_RESULT_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_operation_id;
  uint32_t operation_id;
  bool has_status;
  int32_t status;
  bool has_reason;
  uint16_t reason;
  bool has_observed_run_state;
  uint8_t observed_run_state;
  bool has_applied_policy_revision;
  uint32_t applied_policy_revision;
  bool has_control_lease;
  uint64_t control_lease;
} operation_result_value_t;
#define OPERATION_RESULT_VALUE_SIZE (sizeof(operation_result_value_t))
void operation_result_value_clear(operation_result_value_t *value);
size_t operation_result_value_encoded_size(const operation_result_value_t *value);
wl_codec_status_t operation_result_value_encode(const operation_result_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t operation_result_value_decode(const uint8_t *input, size_t length, operation_result_value_t *out);

#define CONFIGURE_REQUEST_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_operation_id;
  uint32_t operation_id;
  bool has_expected_policy_revision;
  uint32_t expected_policy_revision;
  bool has_new_policy_revision;
  uint32_t new_policy_revision;
  bool has_gravity_owner;
  uint8_t gravity_owner;
  bool has_friction_owner;
  uint8_t friction_owner;
  bool has_disconnect_action;
  uint8_t disconnect_action;
  bool has_teleop_mode;
  uint8_t teleop_mode;
  bool has_arm_command_timeout_us;
  uint32_t arm_command_timeout_us;
  bool has_hand_command_timeout_us;
  uint32_t hand_command_timeout_us;
  bool has_arm_feedback_timeout_us;
  uint32_t arm_feedback_timeout_us;
  bool has_hand_feedback_timeout_us;
  uint32_t hand_feedback_timeout_us;
  bool has_hand_mode;
  uint8_t hand_mode;
  bool has_hand_soft_current_ma;
  uint32_t hand_soft_current_ma[6];
  bool has_hand_current_semantics;
  uint8_t hand_current_semantics;
  bool has_local_fallback_enabled;
  bool local_fallback_enabled;
  bool has_model_revision;
  uint32_t model_revision;
  bool has_host_managed_fault_mask;
  uint32_t host_managed_fault_mask;
  bool has_hard_protection_profile_id;
  uint32_t hard_protection_profile_id;
  bool has_max_command_valid_for_us;
  uint32_t max_command_valid_for_us;
  bool has_force_feedback_gain;
  float force_feedback_gain[6];
  bool has_arm_state_period_us;
  uint32_t arm_state_period_us;
  bool has_hand_state_period_us;
  uint32_t hand_state_period_us;
  bool has_hand_control_feedforward_ma;
  uint32_t hand_control_feedforward_ma;
  bool has_hand_position_current_limit_ma;
  uint32_t hand_position_current_limit_ma[6];
} configure_request_value_t;
#define CONFIGURE_REQUEST_VALUE_SIZE (sizeof(configure_request_value_t))
void configure_request_value_clear(configure_request_value_t *value);
size_t configure_request_value_encoded_size(const configure_request_value_t *value);
wl_codec_status_t configure_request_value_encode(const configure_request_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t configure_request_value_decode(const uint8_t *input, size_t length, configure_request_value_t *out);

#define CONFIGURE_RESULT_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_operation_id;
  uint32_t operation_id;
  bool has_status;
  int32_t status;
  bool has_reason;
  uint16_t reason;
  bool has_applied_policy_revision;
  uint32_t applied_policy_revision;
  bool has_hard_protection_mask;
  uint32_t hard_protection_mask;
  bool has_capabilities;
  uint32_t capabilities;
} configure_result_value_t;
#define CONFIGURE_RESULT_VALUE_SIZE (sizeof(configure_result_value_t))
void configure_result_value_clear(configure_result_value_t *value);
size_t configure_result_value_encoded_size(const configure_result_value_t *value);
wl_codec_status_t configure_result_value_encode(const configure_result_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t configure_result_value_decode(const uint8_t *input, size_t length, configure_result_value_t *out);

#define PEER_ARM_STATE_HAS_VALUE 1
typedef struct {
  bool has_header;
  stream_header_value_t header;
  bool has_pairing_session;
  uint64_t pairing_session;
  bool has_policy_revision;
  uint32_t policy_revision;
  bool has_role;
  uint8_t role;
  bool has_teleop_mode;
  uint8_t teleop_mode;
  bool has_valid_q_mask;
  uint32_t valid_q_mask;
  bool has_valid_dq_mask;
  uint32_t valid_dq_mask;
  bool has_valid_residual_tau_mask;
  uint32_t valid_residual_tau_mask;
  bool has_feedback_age_us;
  uint32_t feedback_age_us[6];
  bool has_q_rad;
  float q_rad[6];
  bool has_dq_rad_s;
  float dq_rad_s[6];
  bool has_residual_tau_nm;
  float residual_tau_nm[6];
  bool has_critical_fault_mask;
  uint32_t critical_fault_mask;
} peer_arm_state_value_t;
#define PEER_ARM_STATE_VALUE_SIZE (sizeof(peer_arm_state_value_t))
void peer_arm_state_value_clear(peer_arm_state_value_t *value);
size_t peer_arm_state_value_encoded_size(const peer_arm_state_value_t *value);
wl_codec_status_t peer_arm_state_value_encode(const peer_arm_state_value_t *value, uint8_t *out, size_t capacity, size_t *length);
wl_codec_status_t peer_arm_state_value_decode(const uint8_t *input, size_t length, peer_arm_state_value_t *out);

#define STREAM_HEADER_MESSAGE_ID 1U
#define STREAM_HEADER_HAS_MAX_ENCODED_SIZE 1
#define STREAM_HEADER_MAX_ENCODED_SIZE UINT64_C(44)
#define ARM_COMMAND_MESSAGE_ID 257U
#define ARM_COMMAND_HAS_MAX_ENCODED_SIZE 1
#define ARM_COMMAND_MAX_ENCODED_SIZE UINT64_C(192)
#define HAND_COMMAND_MESSAGE_ID 258U
#define HAND_COMMAND_HAS_MAX_ENCODED_SIZE 1
#define HAND_COMMAND_MAX_ENCODED_SIZE UINT64_C(95)
#define ARM_STATE_MESSAGE_ID 513U
#define ARM_STATE_HAS_MAX_ENCODED_SIZE 1
#define ARM_STATE_MAX_ENCODED_SIZE UINT64_C(260)
#define HAND_STATE_MESSAGE_ID 514U
#define HAND_STATE_HAS_MAX_ENCODED_SIZE 1
#define HAND_STATE_MAX_ENCODED_SIZE UINT64_C(248)
#define FAULT_SNAPSHOT_MESSAGE_ID 515U
#define FAULT_SNAPSHOT_HAS_MAX_ENCODED_SIZE 1
#define FAULT_SNAPSHOT_MAX_ENCODED_SIZE UINT64_C(154)
#define DIAGNOSTICS_MESSAGE_ID 516U
#define DIAGNOSTICS_HAS_MAX_ENCODED_SIZE 1
#define DIAGNOSTICS_MAX_ENCODED_SIZE UINT64_C(292)
#define FAULT_EVENT_MESSAGE_ID 769U
#define FAULT_EVENT_HAS_MAX_ENCODED_SIZE 1
#define FAULT_EVENT_MAX_ENCODED_SIZE UINT64_C(111)
#define OPERATION_REQUEST_MESSAGE_ID 1025U
#define OPERATION_REQUEST_HAS_MAX_ENCODED_SIZE 1
#define OPERATION_REQUEST_MAX_ENCODED_SIZE UINT64_C(84)
#define OPERATION_RESULT_MESSAGE_ID 1026U
#define OPERATION_RESULT_HAS_MAX_ENCODED_SIZE 1
#define OPERATION_RESULT_MAX_ENCODED_SIZE UINT64_C(79)
#define CONFIGURE_REQUEST_MESSAGE_ID 1027U
#define CONFIGURE_REQUEST_HAS_MAX_ENCODED_SIZE 1
#define CONFIGURE_REQUEST_MAX_ENCODED_SIZE UINT64_C(225)
#define CONFIGURE_RESULT_MESSAGE_ID 1028U
#define CONFIGURE_RESULT_HAS_MAX_ENCODED_SIZE 1
#define CONFIGURE_RESULT_MAX_ENCODED_SIZE UINT64_C(77)
#define PEER_ARM_STATE_MESSAGE_ID 1281U
#define PEER_ARM_STATE_HAS_MAX_ENCODED_SIZE 1
#define PEER_ARM_STATE_MAX_ENCODED_SIZE UINT64_C(190)
#ifdef __cplusplus
}
#endif

#endif
