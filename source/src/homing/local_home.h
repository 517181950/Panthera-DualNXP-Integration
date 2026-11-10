#pragma once
#include "homing.h"
#include "paired_home.h"
#include "core/controller.h"
extern struct home_context local_home;
extern uint64_t local_home_updated;
void local_home_init(void);
void local_home_tick(uint64_t now);
/* Call with robot_lock held. */
void local_home_input(struct home_input *in);
bool local_home_activate(void);
bool local_home_soft_stop(unsigned reason);

void local_home_peer_receive(uint32_t local_sequence,uint64_t remote_boot,uint32_t remote_sequence,uint64_t remote_peer_boot,uint32_t remote_peer_age,bool remote_ready,uint32_t remote_home_source,uint32_t remote_home_state,const struct home_button_wire *w);
/* Caller holds robot_lock. */
void local_home_button_state(struct home_button_wire *w,uint64_t now);
