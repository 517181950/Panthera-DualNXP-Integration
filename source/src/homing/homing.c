#include "homing.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
#define DEG (PI/180.f)
static const float kp[6]={24.48f,61.2f,61.2f,36.72f,12.24f,12.24f};
static const float kd[6]={1.2f,1.92f,1.92f,.8313843876f,.48f,.48f};
static unsigned validity(const struct home_input *in) {
    if(in->q_mask!=63 || in->dq_mask!=63 || in->fault_mask!=63) return HOME_STALE;
    for(int i=0;i<6;i++) {
        if(in->age[i]>=20000) return HOME_STALE;
        if(in->fault[i]) return HOME_MOTOR_FAULT;
        if(!isfinite(in->q[i]) || !isfinite(in->dq[i])) return HOME_NONFINITE;
    }
    return HOME_OK;
}
void home_request(struct home_context *h,uint64_t now,uint32_t source) {
    uint32_t generation=h->generation+1;
    memset(h,0,sizeof(*h)); h->phase=HOME_WAIT;h->generation=generation;h->source=source;h->entered=now;
}
void home_abort(struct home_context *h,unsigned error) {
    h->phase=HOME_FAULT;h->error=error;h->output=false;
    memset(h->mit,0,sizeof(h->mit));
}
void home_host_stop(struct home_context *h) {
    /* Normal application exit is not a tracking fault. Never erase a real fault. */
    if(h->phase==HOME_FAULT) return;
    h->phase=HOME_STOPPED;h->error=HOME_OK;h->output=false;
    memset(h->mit,0,sizeof(h->mit));
}
bool home_host_restartable(const struct home_context *h,const struct home_input *in) {
    if((h->phase!=HOME_STOPPED && h->phase!=HOME_SOFT_HOLD) || validity(in)) return false;
    for(int i=0;i<6;i++) if(fabsf(in->dq[i])>3*DEG) return false;
    return true;
}
bool home_host_command_expired(bool active,bool received,uint64_t at,uint64_t now) {
    return active && (!received || (now>=at && now-at>=50000));
}
static bool soft_reason(unsigned reason) {
    return reason==HOME_OK || reason==HOME_TRACKING || reason==HOME_TIMEOUT ||
        reason==HOME_PEER_LOST || reason==HOME_HOST_STALE;
}
static void soft_tick(struct home_context *h,const struct home_input *in) {
    unsigned error=validity(in);
    if(error) {home_abort(h,error);return;}
    /* Low stiffness already used on the new arm; damping from homing.
     * Yield the anchor beyond 3 degrees so contact cannot accumulate a
     * large restoring torque. No continuation towards the old trajectory. */
    static const float soft_kp[6]={4,10,10,2,2,1};
    float tau[6];home_gravity(in->q,tau);
    for(int i=0;i<6;i++) {
        if(fabsf(in->q[i]-h->previous[i])>PI || !isfinite(tau[i])) {
            home_abort(h,HOME_TURN_JUMP);return;
        }
        h->previous[i]=in->q[i];
        float error=fmaxf(-3*DEG,fminf(3*DEG,h->origin[i]-in->q[i]));
        h->origin[i]=in->q[i]+error;
        h->mit[5*i]=h->origin[i];h->mit[5*i+1]=0;
        h->mit[5*i+2]=soft_kp[i];h->mit[5*i+3]=kd[i];h->mit[5*i+4]=tau[i];
    }
    h->output=true;
}
bool home_soft_stop(struct home_context *h,const struct home_input *in,uint64_t now,unsigned reason) {
    unsigned error=validity(in);
    if(error) {home_abort(h,error);return false;}
    if(h->phase==HOME_FAULT && !soft_reason(h->error)) return false;
    if(h->phase!=HOME_SOFT_HOLD) {
        memcpy(h->origin,in->q,sizeof(h->origin));memcpy(h->previous,in->q,sizeof(h->previous));
        h->phase=HOME_SOFT_HOLD;h->error=reason;h->entered=now;
    }
    soft_tick(h,in);return h->phase==HOME_SOFT_HOLD;
}
bool home_stop_required(struct home_context *h,uint64_t updated,uint64_t now) {
    /* A higher-priority control tick may publish AFTER CAN sampled `now`.
     * Such an output is newer, not 2^64 microseconds stale. */
    if(h->phase!=HOME_IDLE && h->output && now>=updated && now-updated>=10000) home_abort(h,HOME_CONTROL_STALE);
    return h->phase==HOME_FAULT || h->phase==HOME_STOPPED;
}
#define fault home_abort
static bool tracking_ok(struct home_context *h,const struct home_input *in,const float target[6],uint64_t now) {
    float worst=0;
    for(int i=0;i<6;i++) worst=fmaxf(worst,fabsf(in->q[i]-target[i]));
    if(worst>5*DEG) {home_soft_stop(h,in,now,HOME_TRACKING);return false;}
    return true;
}
void home_tick(struct home_context *h,const struct home_input *in,uint64_t now) {
    if(h->phase==HOME_SOFT_HOLD) {soft_tick(h,in);return;}
    if(h->phase==HOME_IDLE || h->phase==HOME_FAULT || h->phase==HOME_STOPPED) {h->output=false;return;}
    unsigned error=validity(in);
    if(h->phase==HOME_WAIT) {
        /* Boot may precede drive power. Require 0.5 s of fresh, stationary feedback. */
        if(error) {h->stable=0;memset(&h->still,0,sizeof(h->still));h->error=error;if(h->output || ((h->source==2 || h->source==3) && now-h->entered>10000000)) fault(h,error);return;}
        h->error=0;
        if((h->source==2 || h->source==3)) {
            /* SW2 first arrests the current motion at measured pose. */
            for(int i=0;i<6;i++) {
                if(h->output && fabsf(in->q[i]-h->previous[i])>PI) {fault(h,HOME_TURN_JUMP);return;}
                h->previous[i]=in->q[i];
            }
            float tau[6];home_gravity(in->q,tau);
            for(int i=0;i<6;i++) {
                if(!h->output) {h->mit[5*i]=in->q[i];h->mit[5*i+1]=0;h->mit[5*i+2]=kp[i];h->mit[5*i+3]=kd[i];}
                h->mit[5*i+4]=tau[i];
            }
            h->output=true;
            float held[6];for(int i=0;i<6;i++) held[i]=h->mit[5*i];
            if(!tracking_ok(h,in,held,now)) return;
            if(now-h->entered>10000000) {home_soft_stop(h,in,now,HOME_TIMEOUT);return;}
        }
        if(!home_still_update(&h->still,in->q,in->dq,now,true)) return;
        memset(&h->still,0,sizeof(h->still));
        if(!home_path_reference(in->q,h->zero)) {fault(h,HOME_NONFINITE);return;}
        for(int i=0;i<6;i++) {
            h->origin[i]=in->q[i]-h->zero[i]; h->previous[i]=in->q[i];
        }
        h->stage=0;home_path_begin(&h->path,h->origin,0);
        h->entered=now;h->stable=0;h->phase=HOME_RAMP;
    } else if(error) {fault(h,error);return;}
    float q[6],tau[6];
    for(int i=0;i<6;i++) {
        if(fabsf(in->q[i]-h->previous[i])>PI) {fault(h,HOME_TURN_JUMP);return;}
        h->previous[i]=in->q[i];q[i]=in->q[i]-h->zero[i];
    }
    float t=(now-h->entered)*1e-6f,blend=1;
    float target[6]={0},velocity[6]={0};
    static const unsigned moving[3]={HOME_RAISE,HOME_ALIGN,HOME_LOWER};
    static const unsigned settling[3]={HOME_RAISE_SETTLE,HOME_ALIGN_SETTLE,HOME_LOWER_SETTLE};
    if(h->phase==HOME_RAMP) {
        bool manual=h->source==2 || h->source==3;
        blend=manual?1:fminf(t,1);memcpy(target,h->origin,sizeof(target));
        if(manual || t>=1) {h->phase=HOME_RAISE;h->entered=now;}
    } else if(h->phase==moving[h->stage]) {
        home_path_sample(&h->path,t,target,velocity);
        if(t>=h->path.duration) {h->phase=settling[h->stage];h->entered=now;h->stable=0;memset(&h->still,0,sizeof(h->still));}
    } else if(h->phase==settling[h->stage]) {
        memcpy(target,h->path.goal,sizeof(target));bool position_ok=true;
        for(int i=0;i<6;i++) if(fabsf(q[i]-target[i])>2*DEG) position_ok=false;
        if(home_still_update(&h->still,q,in->dq,now,position_ok)) {
            if(h->stage<2) {h->stage++;home_path_begin(&h->path,target,h->stage);h->phase=moving[h->stage];}
            else h->phase=HOME_HOLD;
            h->entered=now;h->stable=0;
        } else if(t>5) {home_soft_stop(h,in,now,HOME_TIMEOUT);return;}
    }
    float raw_target[6];for(int i=0;i<6;i++)raw_target[i]=target[i]+h->zero[i];
    if(!tracking_ok(h,in,raw_target,now)) return;
    home_gravity(q,tau);
    for(int i=0;i<6;i++) {
        h->mit[5*i]=h->zero[i]+target[i];
        h->mit[5*i+1]=velocity[i];
        h->mit[5*i+2]=kp[i]*blend; h->mit[5*i+3]=kd[i];
        h->mit[5*i+4]=tau[i];
    }
    h->output=true;
}
bool home_takeover_ok(const struct home_context *h,const struct home_input *in,const float mit[30],uint64_t age) {
    if(h->phase==HOME_IDLE) return true;
    if((h->phase!=HOME_HOLD && !home_host_restartable(h,in)) || validity(in) || age>=20000) return false;
    for(int i=0;i<6;i++) {
        for(int k=0;k<5;k++) if(!isfinite(mit[i*5+k])) return false;
        if(mit[i*5+2]<0 || mit[i*5+3]<0 || fabsf(mit[i*5]-in->q[i])>2*DEG || fabsf(mit[i*5+1])>3*DEG) return false;
    }
    return true;
}
