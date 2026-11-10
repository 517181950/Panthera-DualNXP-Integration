#pragma once
#include "can_codec.h"
#include <stdbool.h>
enum { ROBOT_PARSE_OK=0, ROBOT_PARSE_OTHER=1, ROBOT_PARSE_MALFORMED=-1, ROBOT_PARSE_UNSUPPORTED=-2 };
struct ht_feedback {
 int joint;
 uint8_t present,valid;
 float position_turns,velocity_turns_s,torque_native;
 uint32_t mode_raw,fault_raw;
};
enum { HT_FEEDBACK_Q=1,HT_FEEDBACK_DQ=2,HT_FEEDBACK_TAU=4,HT_FEEDBACK_MODE=8,HT_FEEDBACK_FAULT=16 };
struct hand_feedback {
 uint8_t cmd,index;
 uint32_t fields;
 float position_percent[6],current_ma[6],q_rad[11],dq_rad_s[6];
 uint32_t faults[6];
 uint16_t mode_raw,ack_status;
 bool ack;
};
int ht_decode_feedback(const struct robot_can_frame*f,struct ht_feedback*out);
int hand_decode_feedback(const struct robot_can_frame*f,struct hand_feedback*out);
