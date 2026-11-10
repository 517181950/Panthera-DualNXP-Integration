#include "homing/homing.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define DEG (3.14159265358979323846f/180.f)
int main(void) {
    struct home_input in={.q_mask=63,.dq_mask=63,.fault_mask=63};
    struct home_context h={.phase=HOME_IDLE};
    float q[6]={.2f,.5f,1.f,-.4f,.2f,-.2f},gravity[6];
    memcpy(in.q,q,sizeof(q));in.dq[1]=-.8f;
    assert(home_soft_stop(&h,&in,1000,HOME_OK));
    assert(h.phase==HOME_SOFT_HOLD && h.output);
    home_gravity(in.q,gravity);
    for(int j=0;j<6;j++) {
        assert(h.mit[5*j]==in.q[j] && h.mit[5*j+1]==0);
        assert(h.mit[5*j+2]>0 && h.mit[5*j+2]<=10);
        assert(h.mit[5*j+3]>0 && fabsf(h.mit[5*j+4]-gravity[j])<1e-6);
    }
    /* Damping opposes motion, with no push along the abandoned trajectory. */
    assert(-h.mit[8]*in.dq[1]>0);
    in.dq[1]=0;
    for(uint64_t t=3000;t<2000000;t+=2000) {
        in.q[1]+=.001f;
        home_tick(&h,&in,t);
        assert(h.phase==HOME_SOFT_HOLD && h.output);
        assert(fabsf(h.mit[5]-in.q[1])<=3*DEG+1e-6f);
        assert(!home_stop_required(&h,t,t));
    }
    float anchor=h.origin[1];
    assert(home_soft_stop(&h,&in,2000000,HOME_OK));assert(h.origin[1]==anchor);
    assert(home_host_restartable(&h,&in));
    assert(home_takeover_ok(&h,&in,h.mit,0)==(fabsf(h.mit[5]-in.q[1])<=2*DEG));
    /* Fresh measured-pose takeover is allowed, old target/velocity is not. */
    float target[30];memcpy(target,h.mit,sizeof(target));
    for(int j=0;j<6;j++)target[5*j]=in.q[j];
    assert(home_takeover_ok(&h,&in,target,0));
    target[5]+=4*DEG;assert(!home_takeover_ok(&h,&in,target,0));
    struct home_context stopped=h;home_host_stop(&stopped);
    assert(stopped.phase==HOME_STOPPED && !stopped.output && home_stop_required(&stopped,0,1));
    in.age[2]=20000;home_tick(&h,&in,2002000);
    assert(h.phase==HOME_FAULT && h.error==HOME_STALE && !h.output);
    in.age[2]=0;assert(!home_soft_stop(&h,&in,2004000,HOME_OK));
    h=(struct home_context){0};in.fault[0]=1;
    assert(!home_soft_stop(&h,&in,1,HOME_OK));assert(h.error==HOME_MOTOR_FAULT);
    in.fault[0]=0;h=(struct home_context){0};in.q[1]=NAN;
    assert(!home_soft_stop(&h,&in,1,HOME_OK));assert(h.error==HOME_NONFINITE);
    assert(!home_host_command_expired(false,true,0,100000));
    assert(!home_host_command_expired(true,true,100000,99999));
    assert(!home_host_command_expired(true,true,1000,50999));
    assert(home_host_command_expired(true,true,1000,51000));
    assert(home_host_command_expired(true,false,1000,1001));
    puts("Soft stop: gravity+damping, bounded yielding anchor, no auto-resume, hard fault fallback, 50ms watchdog passed");
}
