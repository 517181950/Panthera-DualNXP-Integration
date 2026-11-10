#ifndef WIRELINK_GENERATED_ROBOT_MESSAGES_H
#define WIRELINK_GENERATED_ROBOT_MESSAGES_H

/* Advanced borrowed codec and explicit value/view conversions. */
#include "robot_messages_values.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct stream_header stream_header_t;
typedef struct arm_command arm_command_t;
typedef struct hand_command hand_command_t;
typedef struct arm_state arm_state_t;
typedef struct hand_state hand_state_t;
typedef struct fault_snapshot fault_snapshot_t;
typedef struct diagnostics diagnostics_t;
typedef struct fault_event fault_event_t;
typedef struct operation_request operation_request_t;
typedef struct operation_result operation_result_t;
typedef struct configure_request configure_request_t;
typedef struct configure_result configure_result_t;
typedef struct peer_arm_state peer_arm_state_t;

struct stream_header {
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
};

struct arm_command {
  bool has_header;
  stream_header_t header;
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
};

struct hand_command {
  bool has_header;
  stream_header_t header;
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
};

struct arm_state {
  bool has_header;
  stream_header_t header;
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
};

struct hand_state {
  bool has_header;
  stream_header_t header;
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
};

struct fault_snapshot {
  bool has_header;
  stream_header_t header;
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
};

struct diagnostics {
  bool has_header;
  stream_header_t header;
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
};

struct fault_event {
  bool has_header;
  stream_header_t header;
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
};

struct operation_request {
  bool has_header;
  stream_header_t header;
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
};

struct operation_result {
  bool has_header;
  stream_header_t header;
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
};

struct configure_request {
  bool has_header;
  stream_header_t header;
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
};

struct configure_result {
  bool has_header;
  stream_header_t header;
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
};

struct peer_arm_state {
  bool has_header;
  stream_header_t header;
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
};

void stream_header_clear(stream_header_t *value);
size_t stream_header_encoded_size(const stream_header_t *value);
wl_codec_status_t stream_header_encode(const stream_header_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t stream_header_decode(const uint8_t *input, size_t input_length, stream_header_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t stream_header_value_from_view(const stream_header_t *view, stream_header_value_t *out);
wl_codec_status_t stream_header_value_to_view(const stream_header_value_t *value, stream_header_t *out);

void arm_command_clear(arm_command_t *value);
size_t arm_command_encoded_size(const arm_command_t *value);
wl_codec_status_t arm_command_encode(const arm_command_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t arm_command_decode(const uint8_t *input, size_t input_length, arm_command_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t arm_command_value_from_view(const arm_command_t *view, arm_command_value_t *out);
wl_codec_status_t arm_command_value_to_view(const arm_command_value_t *value, arm_command_t *out);

void hand_command_clear(hand_command_t *value);
size_t hand_command_encoded_size(const hand_command_t *value);
wl_codec_status_t hand_command_encode(const hand_command_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t hand_command_decode(const uint8_t *input, size_t input_length, hand_command_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t hand_command_value_from_view(const hand_command_t *view, hand_command_value_t *out);
wl_codec_status_t hand_command_value_to_view(const hand_command_value_t *value, hand_command_t *out);

void arm_state_clear(arm_state_t *value);
size_t arm_state_encoded_size(const arm_state_t *value);
wl_codec_status_t arm_state_encode(const arm_state_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t arm_state_decode(const uint8_t *input, size_t input_length, arm_state_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t arm_state_value_from_view(const arm_state_t *view, arm_state_value_t *out);
wl_codec_status_t arm_state_value_to_view(const arm_state_value_t *value, arm_state_t *out);

void hand_state_clear(hand_state_t *value);
size_t hand_state_encoded_size(const hand_state_t *value);
wl_codec_status_t hand_state_encode(const hand_state_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t hand_state_decode(const uint8_t *input, size_t input_length, hand_state_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t hand_state_value_from_view(const hand_state_t *view, hand_state_value_t *out);
wl_codec_status_t hand_state_value_to_view(const hand_state_value_t *value, hand_state_t *out);

void fault_snapshot_clear(fault_snapshot_t *value);
size_t fault_snapshot_encoded_size(const fault_snapshot_t *value);
wl_codec_status_t fault_snapshot_encode(const fault_snapshot_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t fault_snapshot_decode(const uint8_t *input, size_t input_length, fault_snapshot_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t fault_snapshot_value_from_view(const fault_snapshot_t *view, fault_snapshot_value_t *out);
wl_codec_status_t fault_snapshot_value_to_view(const fault_snapshot_value_t *value, fault_snapshot_t *out);

void diagnostics_clear(diagnostics_t *value);
size_t diagnostics_encoded_size(const diagnostics_t *value);
wl_codec_status_t diagnostics_encode(const diagnostics_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t diagnostics_decode(const uint8_t *input, size_t input_length, diagnostics_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t diagnostics_value_from_view(const diagnostics_t *view, diagnostics_value_t *out);
wl_codec_status_t diagnostics_value_to_view(const diagnostics_value_t *value, diagnostics_t *out);

void fault_event_clear(fault_event_t *value);
size_t fault_event_encoded_size(const fault_event_t *value);
wl_codec_status_t fault_event_encode(const fault_event_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t fault_event_decode(const uint8_t *input, size_t input_length, fault_event_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t fault_event_value_from_view(const fault_event_t *view, fault_event_value_t *out);
wl_codec_status_t fault_event_value_to_view(const fault_event_value_t *value, fault_event_t *out);

void operation_request_clear(operation_request_t *value);
size_t operation_request_encoded_size(const operation_request_t *value);
wl_codec_status_t operation_request_encode(const operation_request_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t operation_request_decode(const uint8_t *input, size_t input_length, operation_request_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t operation_request_value_from_view(const operation_request_t *view, operation_request_value_t *out);
wl_codec_status_t operation_request_value_to_view(const operation_request_value_t *value, operation_request_t *out);

void operation_result_clear(operation_result_t *value);
size_t operation_result_encoded_size(const operation_result_t *value);
wl_codec_status_t operation_result_encode(const operation_result_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t operation_result_decode(const uint8_t *input, size_t input_length, operation_result_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t operation_result_value_from_view(const operation_result_t *view, operation_result_value_t *out);
wl_codec_status_t operation_result_value_to_view(const operation_result_value_t *value, operation_result_t *out);

void configure_request_clear(configure_request_t *value);
size_t configure_request_encoded_size(const configure_request_t *value);
wl_codec_status_t configure_request_encode(const configure_request_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t configure_request_decode(const uint8_t *input, size_t input_length, configure_request_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t configure_request_value_from_view(const configure_request_t *view, configure_request_value_t *out);
wl_codec_status_t configure_request_value_to_view(const configure_request_value_t *value, configure_request_t *out);

void configure_result_clear(configure_result_t *value);
size_t configure_result_encoded_size(const configure_result_t *value);
wl_codec_status_t configure_result_encode(const configure_result_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t configure_result_decode(const uint8_t *input, size_t input_length, configure_result_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t configure_result_value_from_view(const configure_result_t *view, configure_result_value_t *out);
wl_codec_status_t configure_result_value_to_view(const configure_result_value_t *value, configure_result_t *out);

void peer_arm_state_clear(peer_arm_state_t *value);
size_t peer_arm_state_encoded_size(const peer_arm_state_t *value);
wl_codec_status_t peer_arm_state_encode(const peer_arm_state_t *value, uint8_t *out, size_t out_capacity, size_t *out_length);
wl_codec_status_t peer_arm_state_decode(const uint8_t *input, size_t input_length, peer_arm_state_t *out);

/* Advanced conversions: views borrow value/input storage. Inputs and outputs
 * must not overlap; failures leave output unchanged. */
wl_codec_status_t peer_arm_state_value_from_view(const peer_arm_state_t *view, peer_arm_state_value_t *out);
wl_codec_status_t peer_arm_state_value_to_view(const peer_arm_state_value_t *value, peer_arm_state_t *out);

#ifdef __cplusplus
}
#endif

#endif
