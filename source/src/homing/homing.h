#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "home_path.h"
#include "home_still.h"
enum home_phase { HOME_IDLE, HOME_WAIT, HOME_RAMP, HOME_MOVE, HOME_SETTLE, HOME_HOLD, HOME_FAULT, HOME_J2_MOVE, HOME_J2_SETTLE, HOME_RAISE, HOME_RAISE_SETTLE, HOME_ALIGN, HOME_ALIGN_SETTLE, HOME_LOWER, HOME_LOWER_SETTLE, HOME_STOPPED, HOME_SOFT_HOLD };
enum home_error { HOME_OK, HOME_STALE, HOME_MOTOR_FAULT, HOME_RANGE, HOME_TURN_JUMP, HOME_TIMEOUT, HOME_NONFINITE, HOME_PEER_LOST, HOME_TRACKING, HOME_CONTROL_STALE, HOME_MAINTENANCE, HOME_HOST_STALE };
struct home_input { float q[6],dq[6]; uint32_t age[6],fault[6],q_mask,dq_mask,fault_mask; };
struct home_context {
    uint32_t phase,generation,error,source;
    uint64_t entered,stable;
    float zero[6],origin[6],previous[6],duration;
    struct home_path path;
    struct home_still still;
    unsigned stage;
    float mit[30];
    bool output;
};
void home_gravity(const float q[6],float tau[6]);
void home_request(struct home_context *h,uint64_t now,uint32_t source);
void home_abort(struct home_context *h,unsigned error);
void home_host_stop(struct home_context *h);
bool home_host_restartable(const struct home_context *h,const struct home_input *in);
bool home_soft_stop(struct home_context *h,const struct home_input *in,uint64_t now,unsigned reason);
bool home_host_command_expired(bool active,bool received,uint64_t at,uint64_t now);
/* CAN owner enforces this independently of the control task. */
bool home_stop_required(struct home_context *h,uint64_t updated,uint64_t now);
void home_tick(struct home_context *h,const struct home_input *in,uint64_t now);
bool home_takeover_ok(const struct home_context *h,const struct home_input *in,const float mit[30],uint64_t age);
