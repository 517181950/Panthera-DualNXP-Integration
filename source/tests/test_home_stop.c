#include "homing/homing.h"
#include "devices/can_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
int main(void) {
    struct home_context h={0};struct robot_can_frame f;
    const uint8_t expected[]={1,0,0,0x1c,4,0,0x11,0x0f};
    const unsigned errors[]={HOME_STALE,HOME_MOTOR_FAULT,HOME_RANGE,HOME_TURN_JUMP,
        HOME_TIMEOUT,HOME_NONFINITE,HOME_PEER_LOST,HOME_TRACKING,HOME_CONTROL_STALE,HOME_MAINTENANCE};
    for(unsigned e=0;e<sizeof(errors)/sizeof(errors[0]);e++) {
        h.output=true;h.mit[7]=61.2f;h.mit[5]=1.5f;home_abort(&h,errors[e]);
        for(uint64_t t=1000;t<1000000;t+=2000) {
            assert(home_stop_required(&h,1000,t) && !h.output);
            for(unsigned j=0;j<6;j++) {
                assert(!ht_encode_stop(&f,robot_motor_id(j)));
                assert(f.id==(0x8011u+j) && f.size==8 && !memcmp(f.data,expected,8));
            }
        }
        for(int i=0;i<30;i++)assert(h.mit[i]==0);
        home_host_stop(&h);
        assert(h.phase==HOME_FAULT && h.error==errors[e]);
    }
    struct home_input in={.q_mask=63,.dq_mask=63,.fault_mask=63};
    h=(struct home_context){.phase=HOME_IDLE,.output=true};h.mit[0]=1;
    home_host_stop(&h);
    assert(h.phase==HOME_STOPPED && !h.output && h.error==HOME_OK);
    for(int i=0;i<30;i++)assert(h.mit[i]==0);
    for(uint64_t t=1000;t<1000000;t+=2000) {
        home_tick(&h,&in,t);
        assert(h.phase==HOME_STOPPED && home_stop_required(&h,t,t));
    }
    assert(home_host_restartable(&h,&in));
    float target[30]={0};target[2]=10;target[3]=1;
    assert(home_takeover_ok(&h,&in,target,0));
    assert(!home_takeover_ok(&h,&in,target,20000));
    target[0]=1;assert(!home_takeover_ok(&h,&in,target,0));target[0]=0;
    in.age[0]=20000;assert(!home_host_restartable(&h,&in));in.age[0]=0;
    in.fault[0]=1;assert(!home_host_restartable(&h,&in));in.fault[0]=0;
    in.dq[0]=1;assert(!home_host_restartable(&h,&in));in.dq[0]=0;
    in.q[0]=NAN;assert(!home_host_restartable(&h,&in));in.q[0]=0;
    in.fault_mask=0;assert(!home_host_restartable(&h,&in));in.fault_mask=63;
    home_request(&h,1000,2);assert(h.phase==HOME_WAIT); /* SW2 still works. */
    assert(ht_encode_stop(NULL,0x11));assert(ht_encode_stop(&f,0));assert(ht_encode_stop(&f,127));
    puts("All homing faults latch repeated mode-zero STOP for all six IDs, without retained MIT targets");
}
