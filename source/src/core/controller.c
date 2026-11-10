#include "controller.h"
#include "homing/local_home.h"
#include "devices/can_trace.h"
#include "devices/can_codec.h"
#include <zephyr/random/random.h>
#include <math.h>
#include <string.h>
extern bool robot_can_hand_only_diagnostic(void);
struct robot_state robot; struct k_spinlock robot_lock;
uint64_t robot_us(void){return k_ticks_to_us_floor64(k_uptime_ticks());}
__weak bool robot_gravity(const float q[6],float t[6]){ARG_UNUSED(q);memset(t,0,24);return false;}
__weak bool robot_friction(const float q[6],float t[6]){ARG_UNUSED(q);memset(t,0,24);return false;}
__weak void robot_output(const arm_command_value_t *a,const hand_command_value_t *h){ARG_UNUSED(a);ARG_UNUSED(h);}
void robot_init(void){memset(&robot,0,sizeof(robot));robot.boot=((uint64_t)sys_rand32_get()<<32)|sys_rand32_get();if(!robot.boot)robot.boot=1;robot.revision=1;
 init_arm_command(&robot.arm);init_hand_command(&robot.hand);init_configure_request(&robot.config);
 init_arm_state(&robot.arm_feedback);init_hand_state(&robot.hand_feedback);
 robot.config.arm_state_period_us=2000;robot.config.hand_state_period_us=10000;robot.hand_poll_period_us=10000;robot.motor_poll_period_us=2000;robot.config.hand_mode=2;robot.hand_isolation=true;
 for(int i=0;i<6;i++){robot.arm_feedback.feedback_age_us[i]=UINT32_MAX;robot.config.force_feedback_gain[i]=1;}
 for(int i=0;i<5;i++)robot.hand_feedback.field_age_us[i]=UINT32_MAX;
}
BUILD_ASSERT(CONFIG_ROBOT_NODE_ID != CONFIG_ROBOT_HOST_NODE_ID, "Board and host identities must differ");
bool robot_header_ok(const stream_header_value_t *h,bool control){return h->schema_revision==7&&h->source_node==CONFIG_ROBOT_HOST_NODE_ID&&h->target_node==CONFIG_ROBOT_NODE_ID&&h->source_boot_id!=0&&h->target_boot_id==robot.boot&&(!control||h->source_boot_id==robot.peer_boot);}
void robot_header(stream_header_value_t *h,uint32_t seq){init_stream_header(h);h->schema_revision=7;h->source_node=CONFIG_ROBOT_NODE_ID;h->target_node=CONFIG_ROBOT_HOST_NODE_ID;h->source_boot_id=robot.boot;h->target_boot_id=robot.peer_boot;h->sequence=seq;h->tx_monotonic_us=robot_us();}
static bool newer(uint32_t a,uint32_t b){return (int32_t)(a-b)>0;}
int robot_arm(const arm_command_value_t *v){if(!robot_header_ok(&v->header,true)||v->control_lease!=robot.lease||!robot.lease||v->policy_revision!=robot.revision||(v->joint_mask&~63u))return -1;
 for(int i=0;i<30;i++)if(!isfinite(v->mit[i]))return -1;
 k_spinlock_key_t key=k_spin_lock(&robot_lock);
 if(local_home.phase!=HOME_IDLE && local_home.phase!=HOME_HOLD && local_home.phase!=HOME_STOPPED && local_home.phase!=HOME_SOFT_HOLD){k_spin_unlock(&robot_lock,key);return -16;}
 if(robot.arm_received&&!newer(v->header.sequence,robot.arm_seq)){robot.old++;k_spin_unlock(&robot_lock,key);return 1;}
 for(int i=0;i<6;i++)if(v->joint_mask&(1u<<i))memcpy(&robot.arm.mit[i*5],&v->mit[i*5],20);
 robot.arm.header=v->header;robot.arm.joint_mask=v->joint_mask;robot.arm_seq=v->header.sequence;robot.arm_at=robot_us();robot.arm_received++;k_spin_unlock(&robot_lock,key);return 0;}
int robot_hand(const hand_command_value_t *v){if(!robot_header_ok(&v->header,true)||v->control_lease!=robot.lease||!robot.lease||v->policy_revision!=robot.revision)return -1;
 for(int i=0;i<6;i++)if(!isfinite(v->target_q_rad[i]))return -1;
 k_spinlock_key_t key=k_spin_lock(&robot_lock);if(robot.hand_received&&!newer(v->header.sequence,robot.hand_seq)){robot.old++;k_spin_unlock(&robot_lock,key);return 1;}
 robot.hand=*v;robot.hand_seq=v->header.sequence;robot.hand_at=robot_us();robot.hand_received++;k_spin_unlock(&robot_lock,key);return 0;}
int robot_configure(const configure_request_value_t *v){if(!robot_header_ok(&v->header,true)||v->expected_policy_revision!=robot.revision||v->new_policy_revision<=robot.revision)return -1;
 /* Unsupported model/teleoperation is explicit, never report fake success. */
 if(v->gravity_owner==2||v->friction_owner==2||v->local_fallback_enabled||v->teleop_mode)return -2;
 if(v->hand_mode<1||v->hand_mode>3)return -1;
 if(v->arm_state_period_us==0||v->hand_state_period_us==0)return -1;
 k_spinlock_key_t key=k_spin_lock(&robot_lock);robot.config=*v;robot.revision=v->new_policy_revision;k_spin_unlock(&robot_lock,key);return 0;}
int robot_operation(const operation_request_value_t *v){if(!robot_header_ok(&v->header,false)||!v->operation_id)return -1;
 k_spinlock_key_t key=k_spin_lock(&robot_lock);int status=0;
 switch(v->operation){case 1:{struct home_input in;local_home_input(&in);if(local_home.phase!=HOME_IDLE && local_home.phase!=HOME_HOLD && !home_host_restartable(&local_home,&in)){status=-16;break;}if(robot.lease&&robot.peer_boot!=v->header.source_boot_id){status=-3;break;}if(!robot.lease){robot.lease=((uint64_t)sys_rand32_get()<<32)|sys_rand32_get();if(!robot.lease)robot.lease=1;robot.arm_received=robot.hand_received=0;}robot.peer_boot=v->header.source_boot_id;break;}
 case 2:case 3:case 4:case 7:if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease){status=-1;break;}if(v->operation==2){robot.active=0;robot.zero_mit_load=false;robot.hand_mode_request=0;robot.lease=0;}else {if((v->operation==3||v->operation==7) && !local_home_activate()){status=-16;break;}robot.active=v->operation==3||v->operation==7;}break;
 case 8:if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease||(v->target_mask&~0x303u)||(v->target_mask&255)<1||(v->target_mask&255)>3){status=-1;break;}
 if(robot.hand_mode_request){status=-16;break;}
 if(!robot_can_hand_only_diagnostic()&&(!(robot.arm_feedback.valid_q_mask&8u)||robot.arm_feedback.feedback_age_us[3]>10000)){status=-11;break;}
 if(!(v->target_mask&0x100u)&&(!robot.hand_feedback.online||(robot.hand_feedback.valid_fields&34u)!=34u||robot.hand_feedback.field_age_us[1]>100000)){status=-19;break;}
 robot.hand_guard_keep_disabled=(v->target_mask&0x200u)!=0;robot.hand_mode_request=v->target_mask&255;robot.hand_guard_token=v->operation_id;robot.hand_guard_status=0;robot.hand_guard_phase=0;break;
 case 9:if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease||(v->target_mask&~63u)||!v->target_mask){status=-1;break;}robot.motor_disable_request|=v->target_mask;robot.arm.joint_mask&=~v->target_mask;break;
 case 10:robot.hand_reply_driven=(v->target_mask&1024u)!=0;robot.hand_full_state=(v->target_mask&256u)!=0;hand_full_checksum_compatibility(robot.hand_full_state&&(v->target_mask&512u));robot.hand_current_fast=(v->target_mask&64u)!=0;robot.hand_velocity_fast=(v->target_mask&128u)!=0;robot.hand_angle_fast=(v->target_mask&32u)!=0;robot.hand_isolation=(v->target_mask&8u)!=0;if(!robot.state_stream_active){robot.prior_arm_state_us=robot.config.arm_state_period_us;robot.prior_hand_state_us=robot.config.hand_state_period_us;robot.prior_trace_enabled=robot_can_trace_enabled();robot.prior_hand_poll_us=robot.hand_poll_period_us;robot.prior_motor_poll_us=robot.motor_poll_period_us;robot.state_stream_active=true;}robot.config.arm_state_period_us=2000;robot.config.hand_state_period_us=2000;robot.hand_poll_period_us=(v->target_mask&16u)?100000:(v->target_mask&2u)?10000:2000;robot.motor_poll_period_us=(v->target_mask&4u)?1000:2000;uint32_t hand_hz=v->target_mask>>16;if(hand_hz>=1&&hand_hz<=2000)robot.hand_poll_period_us=1000000/hand_hz;robot_can_trace_enable(v->target_mask&1u);break;
 case 11:robot.hand_reply_driven=false;robot.hand_full_state=false;hand_full_checksum_compatibility(0);robot.hand_isolation=true;robot.hand_angle_fast=false;robot.hand_current_fast=false;robot.hand_velocity_fast=false;if(robot.state_stream_active){robot.config.arm_state_period_us=robot.prior_arm_state_us;robot.config.hand_state_period_us=robot.prior_hand_state_us;robot.hand_poll_period_us=robot.prior_hand_poll_us;robot.motor_poll_period_us=robot.prior_motor_poll_us;robot_can_trace_enable(robot.prior_trace_enabled);robot.state_stream_active=false;}break;
 case 13:if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease||v->target_mask>1){status=-1;break;}robot.zero_mit_load=v->target_mask!=0;robot.hand_isolation=false;break;
 case 14:if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease){status=-1;break;}robot.hand_read_all_once=true;break;
 case 15:if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease||(v->target_mask!=11&&v->target_mask!=17)){status=-1;break;}robot.motor_probe_base=v->target_mask;break;
 /* Explicit bus selection persists across stream stop/release. No motion command. */
 case 16:
 if(v->target_mask>1||(robot.lease&&(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease))){status=-1;break;}
 atomic_set(&robot.hand_tx_disabled,v->target_mask==0);
 robot.hand_mode_request=0;robot.hand_read_all_once=false;robot.hand_received=0;
 robot.hand_guard_phase=0;robot.hand_guard_status=0;
 break;
 /* Host homing abort: revoke old targets and latch repeated mode-zero STOP.
  * Unlike deactivate/release, this does not leave the last MIT target active. */
 case 17:
 /* The last owner can explicitly cut drive during its soft hold too.
  * This path is not used by normal host cleanup or the watchdog. */
 if(v->header.source_boot_id!=robot.peer_boot ||
    ((v->control_lease!=robot.lease||!robot.lease) && !(local_home.phase==HOME_SOFT_HOLD&&!robot.lease))){status=-1;break;}
 home_host_stop(&local_home);robot.active=0;robot.lease=0;
 robot.arm_received=robot.hand_received=0;robot.zero_mit_load=false;robot.hand_mode_request=0;
 break;
 /* Soft stop remains board-owned after the host releases/exits. */
 case 18:
 if(v->header.source_boot_id==robot.peer_boot && !robot.lease && local_home.phase==HOME_SOFT_HOLD) break;
 if(v->header.source_boot_id!=robot.peer_boot||v->control_lease!=robot.lease||!robot.lease){status=-1;break;}
 if(!local_home_soft_stop(HOME_OK)) status=-5;
 break;
 case 19:break; /* Read-only capability probe: soft hold + host watchdog. */
 case 5:break;default:status=-2;break;}
 k_spin_unlock(&robot_lock,key);return status;}
static void control(void*a,void*b,void*c){ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);uint64_t last=robot_us(),deadline=last+2000;
 while(1){k_sleep(K_TIMEOUT_ABS_US(deadline));uint64_t now=robot_us();deadline+=2000;if(now>=deadline)deadline+=((now-deadline)/2000+1)*2000;uint32_t dt=now-last;last=now;
 local_home_tick(now);
 arm_command_value_t arm;hand_command_value_t hand;uint8_t active;
 k_spinlock_key_t key=k_spin_lock(&robot_lock);robot.cycles++;robot.period_sum+=dt;if(dt>robot.period_max)robot.period_max=dt;uint32_t j=dt>2000?dt-2000:2000-dt;if(j>robot.jitter_max)robot.jitter_max=j;arm=robot.arm;hand=robot.hand;active=robot.active;k_spin_unlock(&robot_lock,key);
 /* No application speed/position/torque/slew clamps. CAN has a separate owner task. */
 if(active)robot_output(&arm,&hand);
 }}
K_THREAD_DEFINE(control_task,3072,control,NULL,NULL,NULL,3,0,2000);
