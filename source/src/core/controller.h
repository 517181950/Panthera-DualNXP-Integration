#pragma once
#include <zephyr/kernel.h>
#include "generated/value_init.h"
struct robot_state {
 atomic_t hand_tx_disabled;
 arm_command_value_t arm; hand_command_value_t hand; configure_request_value_t config;
 arm_state_value_t arm_feedback; hand_state_value_t hand_feedback;
 uint64_t boot,peer_boot,lease; uint32_t revision,arm_seq,hand_seq,peer_seq,state_probe_seq,state_probe_count;
 uint32_t rx,bad,old,tx_drop,cycles,period_max,jitter_max,arm_received,hand_received;
 uint8_t motor_probe_base;bool hand_reply_driven;bool hand_full_state;bool hand_read_all_once;bool zero_mit_load;bool hand_current_fast,hand_velocity_fast;bool hand_angle_fast;bool hand_isolation;bool hand_guard_keep_disabled;uint8_t hand_guard_phase;int32_t hand_guard_status;uint32_t hand_guard_elapsed_us,hand_guard_token;
 uint32_t prior_arm_state_us,prior_hand_state_us,hand_poll_period_us,prior_hand_poll_us,motor_poll_period_us,prior_motor_poll_us;bool state_stream_active,prior_trace_enabled;
 uint64_t arm_at,hand_at,period_sum,state_probe_at; uint8_t active,role,teleop,hand_mode_request,motor_disable_request;
};
extern struct robot_state robot;
extern struct k_spinlock robot_lock;
uint64_t robot_us(void);
void robot_init(void);
bool robot_header_ok(const stream_header_value_t *h,bool control);
void robot_header(stream_header_value_t *h,uint32_t sequence);
int robot_arm(const arm_command_value_t *v);
int robot_hand(const hand_command_value_t *v);
int robot_configure(const configure_request_value_t *v);
int robot_operation(const operation_request_value_t *v);
/* Integrator supplies model/drive hooks; return false when no model exists. */
bool robot_gravity(const float q[6],float tau[6]);
bool robot_friction(const float dq[6],float tau[6]);
void robot_output(const arm_command_value_t *a,const hand_command_value_t *h);
