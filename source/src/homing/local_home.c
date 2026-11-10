#include "local_home.h"
#include <zephyr/drivers/gpio.h>
#include "devices/can_trace.h"
#include <string.h>
#include <math.h>
struct home_context local_home;
uint64_t local_home_updated;
/* Debugger writes this AFTER reset-and-halt, before startup. Not persisted.
 * Consumed once: next ordinary board boot retains requested automatic homing. */
__attribute__((section(".noinit"))) volatile uint32_t local_home_boot_inhibit[2];
static const struct gpio_dt_spec sw2=GPIO_DT_SPEC_GET(DT_ALIAS(sw0),gpios);
static bool ready,button_ready,pressed,raw_previous;
static uint64_t changed;
static struct home_pair pair;
static bool remote_button_pending,paired_peer_fault;
void local_home_input(struct home_input *in) {
    const arm_state_value_t *s=&robot.arm_feedback;
    memcpy(in->q,s->q_rad,sizeof(in->q));memcpy(in->dq,s->dq_rad_s,sizeof(in->dq));
    memcpy(in->age,s->feedback_age_us,sizeof(in->age));memcpy(in->fault,s->fault_code_raw,sizeof(in->fault));
    in->q_mask=s->valid_q_mask;in->dq_mask=s->valid_dq_mask;in->fault_mask=s->motor_fault_valid_mask;
}
void local_home_init(void) {
    if(!IS_ENABLED(CONFIG_ROBOT_LOCAL_HOME)) return;
    /* Standalone homing uses the same quiet CAN diagnostics as host teleop. */
    robot_can_trace_enable(false);
    robot.hand_angle_fast=true;robot.hand_poll_period_us=20000;
    robot.config.hand_state_period_us=20000;
    button_ready=gpio_is_ready_dt(&sw2) && !gpio_pin_configure_dt(&sw2,GPIO_INPUT);
    printk("Local home: SW2 GPIO0_23 %s; boot=%d\n",button_ready?"ready":"unavailable",IS_ENABLED(CONFIG_ROBOT_BOOT_HOME));
    k_spinlock_key_t key=k_spin_lock(&robot_lock);
    bool maintenance=local_home_boot_inhibit[0]==0x50484d31u && local_home_boot_inhibit[1]==0xafb7b2ceu;
    local_home_boot_inhibit[0]=local_home_boot_inhibit[1]=0;
    if(maintenance) home_host_stop(&local_home);
    else if(IS_ENABLED(CONFIG_ROBOT_BOOT_HOME)) home_request(&local_home,robot_us(),1);
    ready=true;k_spin_unlock(&robot_lock,key);
}
bool local_home_activate(void) {
    struct home_input in;local_home_input(&in);
    if(local_home.phase==HOME_IDLE) return true;
    if(!robot.arm_received || robot.arm.joint_mask!=63 || !home_takeover_ok(&local_home,&in,robot.arm.mit,robot_us()-robot.arm_at)) return false;
    local_home.phase=HOME_IDLE;local_home.error=HOME_OK;local_home.source=0;local_home.output=false;return true;
}
bool local_home_soft_stop(unsigned reason) {
    struct home_input in;local_home_input(&in);uint64_t now=robot_us();
    bool held=home_soft_stop(&local_home,&in,now,reason);
    local_home_updated=now;
    robot.lease=0;robot.active=0;robot.arm_received=robot.hand_received=0;
    robot.zero_mit_load=false;robot.hand_mode_request=0;
    return held;
}
static bool feedback_ready(const struct home_input *in) {
    if(in->q_mask!=63 || in->dq_mask!=63 || in->fault_mask!=63) return false;
    for(int i=0;i<6;i++) if(in->age[i]>=20000 || in->fault[i] || !isfinite(in->q[i]) || !isfinite(in->dq[i])) return false;
    return true;
}
void local_home_peer_receive(uint32_t local_sequence,uint64_t remote_boot,uint32_t remote_sequence,uint64_t remote_peer_boot,uint32_t remote_peer_age,bool remote_ready,uint32_t remote_home_source,uint32_t remote_home_state,const struct home_button_wire *w) {
    k_spinlock_key_t key=k_spin_lock(&robot_lock);struct home_input in;local_home_input(&in);
    if(home_pair_receive(&pair,robot.boot,local_sequence,remote_boot,remote_sequence,remote_peer_boot,remote_peer_age,remote_ready,feedback_ready(&in),w,robot_us())) remote_button_pending=true;
    bool related=(local_home.source==2 && remote_home_source==3 && pair.sequence && w->ack==pair.sequence && remote_boot==pair.target_boot)
        || (local_home.source==3 && remote_home_source==2 && pair.ack && w->seq==pair.ack && w->target_boot==robot.boot);
    paired_peer_fault=related && (remote_home_state==HOME_FAULT || remote_home_state==HOME_SOFT_HOLD);
    k_spin_unlock(&robot_lock,key);
}
void local_home_button_state(struct home_button_wire *w,uint64_t now) {home_pair_wire(&pair,w,now);}
void local_home_tick(uint64_t now) {
    if(!ready) return;
    bool trigger=false;
    if(button_ready) {
        int raw=gpio_pin_get_dt(&sw2);
        if(raw>=0) {
            if((bool)raw!=raw_previous) {raw_previous=raw;changed=now;}
            if(now-changed>=20000 && pressed!=raw_previous) {pressed=raw_previous;trigger=pressed;}
        }
    }
    struct home_context h;struct home_input in;
    k_spinlock_key_t key=k_spin_lock(&robot_lock);
    local_home_input(&in);
    bool physical=trigger && home_pair_press(&pair,feedback_ready(&in),now);
    bool remote_request=remote_button_pending;remote_button_pending=false;
    /* Simultaneous physical presses merge while the arms are already stopping. */
    if(remote_request && local_home.source==2 && now-local_home.entered<100000) remote_request=false;
    if(physical || remote_request) {
        home_request(&local_home,now,physical?2:3);paired_peer_fault=false;
        robot.lease=0;robot.active=0;robot.arm_received=robot.hand_received=0;
        robot.zero_mit_load=false;robot.motor_disable_request=0;robot.hand_mode_request=0;
    }
    bool paired=local_home.source==2 || local_home.source==3;
    if(local_home.phase==HOME_IDLE && robot.active) {
        if(!feedback_ready(&in)) local_home_soft_stop(HOME_STALE);
        else if(home_host_command_expired(robot.active,robot.arm_received,robot.arm_at,now)) local_home_soft_stop(HOME_HOST_STALE);
    }
    bool running=local_home.phase!=HOME_IDLE && local_home.phase!=HOME_FAULT && local_home.phase!=HOME_STOPPED && local_home.phase!=HOME_SOFT_HOLD;
    bool handshake_failed=local_home.phase==HOME_WAIT && now-local_home.entered>100000
        && (local_home.source==2?pair.status!=2:!pair.received_committed);
    if(paired && running && (handshake_failed || paired_peer_fault || now-pair.peer_at>100000 || !pair.peer_ready)) {
        local_home_soft_stop(HOME_PEER_LOST);
    }
    h=local_home;uint32_t phase=h.phase,generation=h.generation;
    k_spin_unlock(&robot_lock,key);
    home_tick(&h,&in,now);
    key=k_spin_lock(&robot_lock);
    if(local_home.phase==phase && local_home.generation==generation) {
        local_home=h;local_home_updated=now;
        if(h.phase==HOME_SOFT_HOLD && phase!=HOME_SOFT_HOLD) {
            robot.active=0;robot.lease=0;robot.arm_received=robot.hand_received=0;robot.hand_mode_request=0;
        }
    }
    k_spin_unlock(&robot_lock,key);
}
