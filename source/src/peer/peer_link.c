#include "peer_codec.h"
#include "peer_link.h"
#include "core/controller.h"
#include "homing/local_home.h"
#include <zephyr/net/socket.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <math.h>
#include <string.h>

BUILD_ASSERT(CONFIG_ROBOT_PEER_NODE_ID != CONFIG_ROBOT_NODE_ID);
BUILD_ASSERT(CONFIG_ROBOT_PEER_PORT != CONFIG_ROBOT_RT_PORT && CONFIG_ROBOT_PEER_PORT != CONFIG_ROBOT_MGMT_PORT);
BUILD_ASSERT(PANTHERA_PEER_REPLY_SIZE + 28 <= CONFIG_NET_BUF_DATA_SIZE, "Keep Ethernet TX in one fragment");
void robot_can_diagnostics(diagnostics_value_t *v);
K_SEM_DEFINE(peer_begin,0,1);
static struct panthera_peer_state remote;
static uint64_t received_at;
static uint32_t sequence, tx, rx, tx_error, bad, old, gaps, missed, reboots;

void panthera_peer_start(void) { k_sem_give(&peer_begin); }
static uint32_t age(uint64_t now, uint64_t then) {
    return !then || now-then>UINT32_MAX ? UINT32_MAX : (uint32_t)(now-then);
}
static void snapshot(struct panthera_peer_state *s) {
    memset(s,0,sizeof(*s));
    s->node=CONFIG_ROBOT_NODE_ID; s->target=CONFIG_ROBOT_PEER_NODE_ID;
    s->time_us=robot_us(); s->sequence=sequence;
    s->actuators=IS_ENABLED(CONFIG_ROBOT_LOCAL_ACTUATORS);
    k_spinlock_key_t key=k_spin_lock(&robot_lock);
    s->boot=robot.boot; s->active=robot.active;
    s->home_state=local_home.phase;s->home_generation=local_home.generation;s->home_error=local_home.error;s->home_source=local_home.source;memcpy(s->home_zero,local_home.zero,sizeof(s->home_zero));
    struct home_button_wire w;local_home_button_state(&w,s->time_us);
    s->home_button_seq=w.seq;s->home_button_target_boot=w.target_boot;s->home_button_ref_seq=w.target_sequence;
    s->home_button_age_us=w.age_us;s->home_button_ack=w.ack;s->home_button_status=w.status;
    const arm_state_value_t *a=&robot.arm_feedback;
    s->q_mask=a->valid_q_mask; s->dq_mask=a->valid_dq_mask; s->tau_mask=a->valid_tau_mask;
    s->fault_mask=a->motor_fault_valid_mask;
    memcpy(s->arm_age_us,a->feedback_age_us,sizeof(s->arm_age_us));
    memcpy(s->q,a->q_rad,sizeof(s->q)); memcpy(s->dq,a->dq_rad_s,sizeof(s->dq));
    memcpy(s->tau,a->tau_measured_nm,sizeof(s->tau));
    memcpy(s->fault,a->fault_code_raw,sizeof(s->fault)); memcpy(s->mode,a->motor_mode_raw,sizeof(s->mode));
    s->hand_fields=robot.hand_feedback.valid_fields; s->hand_mode=robot.hand_feedback.mode_raw;
    memcpy(s->hand_age_us,robot.hand_feedback.field_age_us,sizeof(s->hand_age_us));
    memcpy(s->hand_q,robot.hand_feedback.joint_q_rad,sizeof(s->hand_q));
    s->arm_command_sequence=robot.arm_seq; s->hand_command_sequence=robot.hand_seq;
    k_spin_unlock(&robot_lock,key);
    diagnostics_value_t d; init_diagnostics(&d); robot_can_diagnostics(&d);
    memcpy(s->motor_control_tx,d.motor_control_tx,sizeof(s->motor_control_tx));
    memcpy(s->motor_feedback_count,d.motor_feedback_count,sizeof(s->motor_feedback_count));
    memcpy(s->hand_field_feedback_count,d.hand_field_feedback_count,sizeof(s->hand_field_feedback_count));
    s->hand_control_tx=d.hand_control_tx; s->hand_control_ack=d.hand_control_ack;
    s->hand_ack_status=d.hand_control_ack_latest_status; s->can_state=d.can_bus_state;
    s->can_tx_error=d.can_tx_error_count; s->can_rx_error=d.can_rx_error_count;
    s->tx=tx; s->rx=rx; s->tx_error=tx_error; s->bad=bad; s->old=old; s->gaps=gaps; s->missed=missed; s->reboots=reboots;
    s->peer_boot=remote.boot; s->peer_sequence=remote.sequence; s->peer_age_us=age(s->time_us,received_at);
}
static void peer_task(void *a, void *b, void *c) {
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    k_sem_take(&peer_begin,K_FOREVER);
    int fd=zsock_socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    struct sockaddr_in local={.sin_family=AF_INET,.sin_port=htons(CONFIG_ROBOT_PEER_PORT)};
    if(fd<0 || zsock_bind(fd,(struct sockaddr*)&local,sizeof(local))) {
        printk("PHD1 peer socket failed errno=%d\n",errno); if(fd>=0) zsock_close(fd); return;
    }
    struct sockaddr_in peer={.sin_family=AF_INET,.sin_port=htons(CONFIG_ROBOT_PEER_PORT)};
    struct in_addr host;
    zsock_inet_pton(AF_INET,CONFIG_ROBOT_PEER_IPV4_ADDR,&peer.sin_addr);
    zsock_inet_pton(AF_INET,CONFIG_NET_CONFIG_PEER_IPV4_ADDR,&host);
    printk("PHD1 arm+hand peer %s:%d period=2000us local_actuators=%d\n",CONFIG_ROBOT_PEER_IPV4_ADDR,CONFIG_ROBOT_PEER_PORT,IS_ENABLED(CONFIG_ROBOT_LOCAL_ACTUATORS));
    uint64_t deadline=robot_us()+2000, last_query=0;
    for(;;) {
        k_sleep(K_TIMEOUT_ABS_US(deadline));
        for(int i=0;i<8;i++) {
            uint8_t input[PANTHERA_PEER_SIZE+1]; struct sockaddr_in from; socklen_t size=sizeof(from);
            int n=zsock_recvfrom(fd,input,sizeof(input),ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&from,&size);
            if(n<0) { if(errno!=EAGAIN && errno!=EWOULDBLOCK) bad++; break; }
            if(from.sin_addr.s_addr==peer.sin_addr.s_addr && from.sin_port==peer.sin_port) {
                struct panthera_peer_state state;
                if(!panthera_peer_decode(input,n,&state) || state.node!=CONFIG_ROBOT_PEER_NODE_ID || state.target!=CONFIG_ROBOT_NODE_ID) { bad++; continue; }
                if(state.boot==remote.boot) {
                    if(!panthera_peer_newer(state.sequence,remote.sequence)) { old++; continue; }
                    gaps+=state.sequence-remote.sequence-1;
                } else if(remote.boot) reboots++;
                bool ready=state.q_mask==63 && state.dq_mask==63 && state.fault_mask==63;
                for(int j=0;j<6;j++) if(state.arm_age_us[j]>=20000 || state.fault[j] || !isfinite(state.q[j]) || !isfinite(state.dq[j])) ready=false;
                struct home_button_wire w={.seq=state.home_button_seq,.target_boot=state.home_button_target_boot,.target_sequence=state.home_button_ref_seq,.age_us=state.home_button_age_us,.ack=state.home_button_ack,.status=state.home_button_status};
                local_home_peer_receive(sequence,state.boot,state.sequence,state.peer_boot,state.peer_age_us,ready,state.home_source,state.home_state,&w);
                remote=state; received_at=robot_us(); rx++;
            } else if(from.sin_addr.s_addr==host.s_addr && n==8 && !memcmp(input,"PDQ1",4)) {
                /* Read-only inspection, bounded to 100 replies/s. Echo opaque nonce. */
                uint64_t now=robot_us(); if(now-last_query<10000) continue; last_query=now;
                uint8_t reply[PANTHERA_PEER_REPLY_SIZE]; struct panthera_peer_state state;
                memcpy(reply,"PDR1",4); memcpy(reply+4,input+4,4); snapshot(&state);
                panthera_peer_encode(&state,reply+8);
                if(remote.boot) panthera_peer_encode(&remote,reply+8+PANTHERA_PEER_SIZE);
                else memset(reply+8+PANTHERA_PEER_SIZE,0,PANTHERA_PEER_SIZE);
                if(zsock_sendto(fd,reply,sizeof(reply),ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&from,size)!=(int)sizeof(reply)) tx_error++;
            } else bad++;
        }
        struct panthera_peer_state state; uint8_t packet[PANTHERA_PEER_SIZE];
        ++sequence; snapshot(&state); panthera_peer_encode(&state,packet);
        if(zsock_sendto(fd,packet,sizeof(packet),ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&peer,sizeof(peer))==(int)sizeof(packet)) tx++; else tx_error++;
        deadline+=2000; uint64_t now=robot_us();
        if(now>=deadline) { uint32_t skip=(now-deadline)/2000+1; missed+=skip; deadline+=(uint64_t)skip*2000; }
    }
}
K_THREAD_DEFINE(peer_thread,6144,peer_task,NULL,NULL,NULL,6,0,0);
