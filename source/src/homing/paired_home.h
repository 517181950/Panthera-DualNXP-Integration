#pragma once
#include <stdbool.h>
#include <stdint.h>
struct home_button_wire {uint32_t seq;uint64_t target_boot;uint32_t target_sequence,age_us,ack,status;};
struct home_pair {
    uint64_t peer_boot,peer_at,sent_at,target_boot;
    uint32_t peer_sequence,seen,sequence,target_sequence,ack,status;
    bool peer_ready,received_committed;
};
/* No command is queued across reboot/link loss. Reference to receiver's own
 * live sequence bounds age without assuming synchronized MCU clocks. */
bool home_pair_press(struct home_pair *p,bool local_ready,uint64_t now);
bool home_pair_receive(struct home_pair *p,uint64_t local_boot,uint32_t local_sequence,
    uint64_t remote_boot,uint32_t remote_sequence,uint64_t remote_peer_boot,uint32_t remote_peer_age,
    bool remote_ready,bool local_ready,const struct home_button_wire *w,uint64_t now);
void home_pair_wire(struct home_pair *p,struct home_button_wire *w,uint64_t now);
