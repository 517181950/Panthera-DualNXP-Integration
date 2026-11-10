#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* PHD1 v3: explicit little-endian integers and IEEE754 binary32; CRC32/IEEE.
 * State plus bounded physical-SW2 homing events; host queries stay read-only. */
#define PANTHERA_PEER_SIZE 492
#define PANTHERA_PEER_REPLY_SIZE (8 + 2 * PANTHERA_PEER_SIZE)
struct panthera_peer_state {
    uint16_t node, target;
    uint64_t boot;
    uint32_t sequence;
    uint64_t time_us;
    uint32_t active, actuators, q_mask, dq_mask, tau_mask, fault_mask;
    uint32_t arm_age_us[6];
    float q[6], dq[6], tau[6];
    uint32_t fault[6], mode[6];
    uint32_t hand_fields, hand_mode, hand_age_us[5];
    float hand_q[11];
    uint32_t tx, rx, tx_error, bad, old, gaps, missed, reboots;
    uint64_t peer_boot;
    uint32_t peer_sequence, peer_age_us, arm_command_sequence, hand_command_sequence;
    uint32_t motor_control_tx[6], motor_feedback_count[6], hand_field_feedback_count[5];
    uint32_t hand_control_tx, hand_control_ack, hand_ack_status, can_state, can_tx_error, can_rx_error;
    uint32_t home_state,home_generation,home_error,home_source;
    float home_zero[6];
    uint32_t home_button_seq;
    uint64_t home_button_target_boot;
    uint32_t home_button_ref_seq,home_button_age_us,home_button_ack,home_button_status;
};
void panthera_peer_encode(const struct panthera_peer_state *s, uint8_t out[PANTHERA_PEER_SIZE]);
bool panthera_peer_decode(const uint8_t *data, size_t n, struct panthera_peer_state *s);
bool panthera_peer_newer(uint32_t sequence, uint32_t previous);
