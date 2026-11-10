// Active schema revision6; actuator and diagnostic messages use generated codecs.
// WLC 0.8.0; exact array counts and field IDs are part of wire identity.
// mit order: joint0[q,dq,kp,kd,tau_ff], ..., joint5[q,dq,kp,kd,tau_ff].
// Named modes, units, freshness and fault policy: see communication-review.txt.
version 1;

message StreamHeader @id(1) {
  required uint16 schema_revision @id(1);
  required uint16 source_node @id(2);
  required uint16 target_node @id(3);
  required fixed64 source_boot_id @id(4);
  required fixed64 target_boot_id @id(5);
  required fixed32 sequence @id(6);
  required fixed64 tx_monotonic_us @id(7);
}

message ArmCommand @id(257) {
  required StreamHeader header @id(1);
  required fixed64 control_lease @id(2);
  required fixed32 policy_revision @id(3);
  required fixed32 valid_for_us @id(4);
  required fixed32 joint_mask @id(5);
  required packed float32 mit[30] @id(6);
}

message HandCommand @id(258) {
  required StreamHeader header @id(1);
  required fixed64 control_lease @id(2);
  required fixed32 policy_revision @id(3);
  required fixed32 valid_for_us @id(4);
  required packed float32 target_q_rad[6] @id(5);
  required uint16 duration_ms @id(6);
}

message ArmState @id(513) {
  required StreamHeader header @id(1);
  required fixed32 last_applied_command_sequence @id(2);
  required fixed32 command_age_us @id(3);
  required fixed32 policy_revision @id(4);
  required uint8 control_owner @id(5);
  required uint8 run_state @id(6);
  required uint8 compensation_mode @id(7);
  required fixed32 online_mask @id(8);
  required fixed32 valid_q_mask @id(9);
  required fixed32 valid_dq_mask @id(10);
  required fixed32 valid_tau_mask @id(11);
  required packed fixed32 feedback_age_us[6] @id(12);
  required packed float32 q_rad[6] @id(13);
  required packed float32 dq_rad_s[6] @id(14);
  required packed float32 tau_measured_nm[6] @id(15);
  required packed fixed32 fault_code_raw[6] @id(16);
  required packed fixed32 motor_mode_raw[6] @id(17);
  required fixed32 motor_fault_valid_mask @id(18);
  required fixed32 last_state_probe_sequence @id(19);
}

message HandState @id(514) {
  required StreamHeader header @id(1);
  required fixed32 last_applied_command_sequence @id(2);
  required fixed32 command_age_us @id(3);
  required bool online @id(4);
  required fixed32 valid_fields @id(5);
  required packed fixed32 field_age_us[5] @id(6);
  required packed float32 motor_position_percent[6] @id(7);
  required packed float32 joint_q_rad[11] @id(8);
  required packed float32 motor_dq_rad_s[6] @id(9);
  required packed float32 motor_current_ma[6] @id(10);
  required packed fixed32 fault_code_raw[6] @id(11);
  required uint16 mode_raw @id(12);
  required fixed32 fault_valid_mask @id(13);
  required bool tactile_capable @id(14);
  required bool tactile_valid @id(15);
}

message FaultSnapshot @id(515) {
  required StreamHeader header @id(1);
  required fixed32 policy_revision @id(2);
  required fixed32 mcu_fault_active @id(3);
  required fixed32 mcu_fault_latched @id(4);
  required fixed32 mcu_fault_valid_mask @id(5);
  required uint8 local_action @id(6);
  required packed fixed32 arm_fault_raw[6] @id(7);
  required fixed32 arm_fault_valid_mask @id(8);
  required packed fixed32 hand_fault_raw[6] @id(9);
  required fixed32 hand_fault_valid_mask @id(10);
  required fixed64 latest_event_id @id(11);
  required fixed64 oldest_retained_event_id @id(12);
  required fixed32 event_overflow_count @id(13);
}

message Diagnostics @id(516) {
  required StreamHeader header @id(1);
  required fixed32 can_tx_completed @id(2);
  required fixed32 can_tx_failed @id(3);
  required fixed32 can_rx_valid @id(4);
  required fixed32 can_rx_unknown @id(5);
  required fixed32 can_rx_ambiguous @id(6);
  required fixed32 can_rx_malformed @id(7);
  required fixed32 can_bus_state @id(8);
  required fixed32 can_tx_error_count @id(9);
  required fixed32 can_rx_error_count @id(10);
  required fixed32 udp_rx_valid @id(11);
  required fixed32 udp_rx_bad @id(12);
  required fixed32 udp_rx_old @id(13);
  required fixed32 udp_tx_drop @id(14);
  required fixed32 control_period_mean_us @id(15);
  required fixed32 control_period_max_us @id(16);
  required fixed32 control_jitter_abs_max_us @id(17);
  required fixed32 command_coalesced @id(18);
  required fixed32 counter_valid_mask @id(19);
  required packed fixed32 motor_feedback_count[6] @id(20);
  required packed fixed32 hand_field_feedback_count[5] @id(21);
  required fixed32 state_probe_received @id(22);
  required fixed32 last_state_probe_sequence @id(23);
  required uint8 hand_state_query_mode @id(24);
  required uint8 hand_mode_guard_phase @id(25);
  required int32 hand_mode_guard_status @id(26);
  required fixed32 hand_mode_guard_elapsed_us @id(27);
  required fixed32 hand_mode_guard_token @id(28);
  required fixed32 hand_control_tx @id(29);
  required fixed32 hand_control_ack @id(30);
  required fixed32 hand_query_timeouts @id(31);
  required packed fixed32 motor_control_tx[6] @id(32);
  required fixed32 hand_full_query_tx @id(33);
  required fixed32 hand_control_negative_ack @id(34);
  required fixed32 hand_control_ack_latest_status @id(35);
}

message FaultEvent @id(769) {
  required StreamHeader header @id(1);
  required fixed64 event_id @id(2);
  required uint8 device_class @id(3);
  required uint16 device_id @id(4);
  required uint8 device_channel @id(5);
  required fixed32 raw_code @id(6);
  required fixed32 normalized_code @id(7);
  required uint8 severity @id(8);
  required bool active @id(9);
  required uint8 local_action @id(10);
  required fixed32 policy_revision @id(11);
  required fixed64 first_seen_us @id(12);
  required fixed64 last_seen_us @id(13);
  required fixed32 occurrences @id(14);
}

message OperationRequest @id(1025) {
  required StreamHeader header @id(1);
  required uint32 operation_id @id(2);
  required uint16 operation @id(3);
  required fixed64 control_lease @id(4);
  required fixed32 expected_policy_revision @id(5);
  required fixed32 target_mask @id(6);
  required fixed64 fault_event_id @id(7);
}

message OperationResult @id(1026) {
  required StreamHeader header @id(1);
  required uint32 operation_id @id(2);
  required int32 status @id(3);
  required uint16 reason @id(4);
  required uint8 observed_run_state @id(5);
  required fixed32 applied_policy_revision @id(6);
  required fixed64 control_lease @id(7);
}

message ConfigureRequest @id(1027) {
  required StreamHeader header @id(1);
  required uint32 operation_id @id(2);
  required fixed32 expected_policy_revision @id(3);
  required fixed32 new_policy_revision @id(4);
  required uint8 gravity_owner @id(5);
  required uint8 friction_owner @id(6);
  required uint8 disconnect_action @id(7);
  required uint8 teleop_mode @id(8);
  required fixed32 arm_command_timeout_us @id(9);
  required fixed32 hand_command_timeout_us @id(10);
  required fixed32 arm_feedback_timeout_us @id(11);
  required fixed32 hand_feedback_timeout_us @id(12);
  required uint8 hand_mode @id(13);
  required packed fixed32 hand_soft_current_ma[6] @id(14);
  required uint8 hand_current_semantics @id(15);
  required bool local_fallback_enabled @id(16);
  required fixed32 model_revision @id(17);
  required fixed32 host_managed_fault_mask @id(18);
  required fixed32 hard_protection_profile_id @id(19);
  required fixed32 max_command_valid_for_us @id(20);
  required packed float32 force_feedback_gain[6] @id(23);
  required fixed32 arm_state_period_us @id(25);
  required fixed32 hand_state_period_us @id(26);
  required fixed32 hand_control_feedforward_ma @id(27);
  required packed fixed32 hand_position_current_limit_ma[6] @id(28);
}

message ConfigureResult @id(1028) {
  required StreamHeader header @id(1);
  required uint32 operation_id @id(2);
  required int32 status @id(3);
  required uint16 reason @id(4);
  required fixed32 applied_policy_revision @id(5);
  required fixed32 hard_protection_mask @id(6);
  required fixed32 capabilities @id(7);
}

message PeerArmState @id(1281) {
  required StreamHeader header @id(1);
  required fixed64 pairing_session @id(2);
  required fixed32 policy_revision @id(3);
  required uint8 role @id(4);
  required uint8 teleop_mode @id(5);
  required fixed32 valid_q_mask @id(6);
  required fixed32 valid_dq_mask @id(7);
  required fixed32 valid_residual_tau_mask @id(8);
  required packed fixed32 feedback_age_us[6] @id(9);
  required packed float32 q_rad[6] @id(10);
  required packed float32 dq_rad_s[6] @id(11);
  required packed float32 residual_tau_nm[6] @id(12);
  required fixed32 critical_fault_mask @id(13);
}
