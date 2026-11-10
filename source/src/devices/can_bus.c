#include "can_codec.h"
#include "can_trace.h"
#include "can_feedback.h"
#include "stopped_can.h"
#include "core/fault_manager.h"
#include "core/controller.h"
#include "homing/local_home.h"
#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
#include <string.h>
#include <errno.h>
bool robot_can_hand_only_diagnostic(void){return true;}
#define TWO_PI 6.2831853071795864769f
static const struct device *const bus=DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
K_MSGQ_DEFINE(rxq,sizeof(struct can_frame),32,4);
K_SEM_DEFINE(can_begin,0,1);
K_SEM_DEFINE(can_ready,0,1);
static int can_init_result=-EAGAIN;
static struct stopped_can stop_bus;
static uint64_t motor_mode_seen[6];
int robot_can_initialize(void){
 if(!IS_ENABLED(CONFIG_ROBOT_LOCAL_ACTUATORS))return 0;
 k_sem_give(&can_begin);
 int r=k_sem_take(&can_ready,K_SECONDS(5));
 return r?r:can_init_result;
}
static void can_initialization_done(int result){can_init_result=result;k_sem_give(&can_ready);}

static atomic_t tx_good,tx_bad,rx_good,rx_bad,rx_unknown;
static atomic_t tx_errors,rx_errors;
static atomic_t motor_feedback_count[6],hand_field_feedback_count[5];
static uint64_t hand_mode_seen,hand_mode_started;static uint8_t hand_mode_expected;
static uint32_t hand_read_control_generation,hand_control_tx,hand_control_ack,hand_full_query_tx,hand_control_negative_ack,hand_control_ack_latest_status;
static atomic_t motor_control_tx[6];
static bool hand_query_pending;static uint8_t hand_query_cmd,hand_query_index;static uint64_t hand_query_sent;static uint32_t hand_query_timeouts,hand_rtt_n,hand_rtt_sum,hand_rtt_max;
static uint32_t bus_status=UINT32_MAX;static uint64_t arm_seen[6],hand_seen[5];
/* Model calibration must be supplied; unknown scaling is not fabricated. */
/* Official standard PantheraHT J6 model4438_30; only J6 commissioned here.
 * Native register position is encoder turns, not URDF joint coordinates.
 * Torque/gain k=0.5256 from official v3.1.2 convert.c; no gravity enabled. */
bool robot_motor_conversion(int joint,float *torque_scale,float *torque_offset,float *kp_scale,float *kd_scale){
 if(joint<0||joint>=6)return false;
 /* Official PantheraHT follower YAML + pinned v3.1.2 torque conversion table. */
 static const float factors[6]={.803f,.677f,.677f,.803f,.5256f,.5256f};float k=factors[joint];*torque_scale=k;*torque_offset=0;*kp_scale=TWO_PI/k;*kd_scale=TWO_PI/k;return true;
}

static void rx_cb(const struct device*d,struct can_frame*f,void*u){ARG_UNUSED(d);ARG_UNUSED(u);robot_can_trace(f,1,0);if(f->dlc>15||k_msgq_put(&rxq,f,K_NO_WAIT)){atomic_inc(&rx_bad);fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_BAD,ENOBUFS,true);}}
static void tx_cb(const struct device*d,int error,void*u){ARG_UNUSED(d);ARG_UNUSED(u);robot_can_trace(NULL,2,error);atomic_inc(error?&tx_bad:&tx_good);fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_TX,(uint32_t)error,error!=0);}
static int send(const struct robot_can_frame*f){if(atomic_get(&robot.hand_tx_disabled)&&(f->id>>8)==((ROBOT_HAND_HOST_ID<<8)|ROBOT_HAND_ID))return -ENODEV;struct can_frame frame={.id=f->id,.dlc=can_bytes_to_dlc(f->size),.flags=CAN_FRAME_FDF|CAN_FRAME_BRS};if(f->id>0x7ff)frame.flags|=CAN_FRAME_IDE;memcpy(frame.data,f->data,f->size);int r=can_send(bus,&frame,K_MSEC(1),tx_cb,NULL);robot_can_trace(&frame,0,r);if(!r&&f->id>=(0x8000u|ROBOT_MOTOR_ID_FIRST)&&f->id<=(0x8000u|ROBOT_MOTOR_ID_LAST)&&f->size==32)atomic_inc(&motor_control_tx[(f->id&255)-ROBOT_MOTOR_ID_FIRST]);if(r){atomic_inc(&tx_bad);fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_TX,(uint32_t)r,true);}return r;}


static bool ht_rx(const struct robot_can_frame*f){
 struct ht_feedback d;int status=ht_decode_feedback(f,&d);
 if(status==ROBOT_PARSE_OTHER)return false;
 if(status!=ROBOT_PARSE_OK){
  atomic_inc(status==ROBOT_PARSE_UNSUPPORTED?&rx_unknown:&rx_bad);
  fault_note(ROBOT_DEVICE_MCU,0,status==ROBOT_PARSE_UNSUPPORTED?ROBOT_CAN_RX_UNKNOWN:ROBOT_CAN_RX_BAD,f->id,true);
  return true;
 }
 uint32_t bit=1u<<d.joint;bool motion=(d.valid&7u)!=0;
 k_spinlock_key_t key=k_spin_lock(&robot_lock);
 arm_state_value_t*v=&robot.arm_feedback;
 if(d.present&HT_FEEDBACK_Q){if(d.valid&HT_FEEDBACK_Q){v->q_rad[d.joint]=d.position_turns*TWO_PI;v->valid_q_mask|=bit;}else v->valid_q_mask&=~bit;}
 if(d.present&HT_FEEDBACK_DQ){if(d.valid&HT_FEEDBACK_DQ){v->dq_rad_s[d.joint]=d.velocity_turns_s*TWO_PI;v->valid_dq_mask|=bit;}else v->valid_dq_mask&=~bit;}
 if(d.present&HT_FEEDBACK_TAU){float scale,offset,kp,kd;if((d.valid&HT_FEEDBACK_TAU)&&robot_motor_conversion(d.joint,&scale,&offset,&kp,&kd)){v->tau_measured_nm[d.joint]=d.torque_native*scale+offset;v->valid_tau_mask|=bit;}else v->valid_tau_mask&=~bit;}
 if(d.valid&HT_FEEDBACK_MODE){v->motor_mode_raw[d.joint]=d.mode_raw;motor_mode_seen[d.joint]=robot_us();}
 if(d.present&HT_FEEDBACK_FAULT){v->fault_code_raw[d.joint]=d.fault_raw;v->motor_fault_valid_mask|=bit;}
 if(motion){arm_seen[d.joint]=robot_us();v->online_mask|=bit;v->feedback_age_us[d.joint]=0;}
 k_spin_unlock(&robot_lock,key);
 if(d.present&HT_FEEDBACK_FAULT)fault_note(ROBOT_DEVICE_ARM,(uint16_t)(d.joint+1),0,d.fault_raw,d.fault_raw!=0);
 fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_BAD,0,false);
 fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_UNKNOWN,0,false);
 if(motion)atomic_inc(&motor_feedback_count[d.joint]);
 atomic_inc(&rx_good);return true;
}
static bool hand_rx(const struct robot_can_frame*f){
 struct hand_feedback d;int status=hand_decode_feedback(f,&d);
 if(status==ROBOT_PARSE_OTHER)return false;
 if(status!=ROBOT_PARSE_OK){atomic_inc(status==ROBOT_PARSE_UNSUPPORTED?&rx_unknown:&rx_bad);fault_note(ROBOT_DEVICE_MCU,0,status==ROBOT_PARSE_UNSUPPORTED?ROBOT_CAN_RX_UNKNOWN:ROBOT_CAN_RX_BAD,f->id,true);return true;}
 uint64_t now=robot_us();
 k_spinlock_key_t key=k_spin_lock(&robot_lock);hand_state_value_t*v=&robot.hand_feedback;
 if(d.fields&1)memcpy(v->motor_position_percent,d.position_percent,sizeof(d.position_percent));
 if(d.fields&2)memcpy(v->joint_q_rad,d.q_rad,sizeof(d.q_rad));
 if(d.fields&4)memcpy(v->motor_dq_rad_s,d.dq_rad_s,sizeof(d.dq_rad_s));
 if(d.fields&8)memcpy(v->motor_current_ma,d.current_ma,sizeof(d.current_ma));
 if(d.fields&16){memcpy(v->fault_code_raw,d.faults,sizeof(d.faults));v->fault_valid_mask=63;}
 if(d.ack&&d.cmd==9){hand_control_ack++;hand_control_ack_latest_status=d.ack_status;if(d.ack_status!=1)hand_control_negative_ack++;}
 if(hand_query_pending&&d.cmd==hand_query_cmd&&d.index==hand_query_index){hand_query_pending=false;uint32_t us=(uint32_t)(now-hand_query_sent);hand_rtt_n++;hand_rtt_sum+=us;if(us>hand_rtt_max)hand_rtt_max=us;}
 if(d.ack&&d.cmd==3&&d.index==7&&now>=hand_mode_started){robot.hand_guard_phase=d.ack_status==1?2:8;robot.hand_guard_status=d.ack_status==1?0:-EIO;}
 if(d.fields&32){v->mode_raw=d.mode_raw;hand_mode_seen=now;if((robot.hand_guard_phase==1||robot.hand_guard_phase==2)&&now>hand_mode_started&&d.mode_raw==hand_mode_expected){robot.hand_guard_phase=7;robot.hand_guard_status=0;robot.hand_guard_elapsed_us=(uint32_t)(now-hand_mode_started);robot.config.hand_mode=hand_mode_expected;}}
 for(unsigned i=0;i<5;i++)if(d.fields&(1u<<i)){hand_seen[i]=now;v->field_age_us[i]=0;atomic_inc(&hand_field_feedback_count[i]);}
 v->valid_fields|=d.fields;v->online=true;k_spin_unlock(&robot_lock,key);
 if(d.fields&16)for(unsigned i=0;i<6;i++)fault_note(ROBOT_DEVICE_HAND,ROBOT_HAND_ID,(uint8_t)(i+1),d.faults[i],d.faults[i]!=0);
 if(d.ack)fault_note(ROBOT_DEVICE_HAND,ROBOT_HAND_ID,(uint8_t)(0x80u|d.index),d.ack_status,d.ack_status!=1);
 fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_BAD,0,false);
 fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_UNKNOWN,0,false);
 atomic_inc(&rx_good);return true;
}
static void process_rx_frame(const struct can_frame*rx){if((rx->flags&CAN_FRAME_RTR)||rx->dlc>15){atomic_inc(&rx_bad);fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_BAD,EINVAL,true);return;}struct robot_can_frame f={.id=rx->id,.size=can_dlc_to_bytes(rx->dlc)};memcpy(f.data,rx->data,f.size);if(!hand_rx(&f)&&!ht_rx(&f)){atomic_inc(&rx_unknown);fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_RX_UNKNOWN,f.id,true);}}
static bool request_hand(uint8_t cmd,uint8_t idx){if(atomic_get(&robot.hand_tx_disabled))return false;uint64_t now=robot_us();if(hand_query_pending){if(now-hand_query_sent<5000)return false;hand_query_pending=false;hand_query_timeouts++;}uint8_t d[2]={0,(cmd==4&&idx==7)?1:0};struct robot_can_frame f;if(hand_encode(&f,cmd,idx,d,2)||send(&f))return false;if(cmd==0x0a&&idx==0)hand_full_query_tx++;hand_query_pending=true;hand_query_cmd=cmd;hand_query_index=idx;hand_query_sent=now;return true;}
void robot_can_diagnostics(diagnostics_value_t*v){v->hand_control_tx=hand_control_tx;v->hand_control_ack=hand_control_ack;v->hand_query_timeouts=hand_query_timeouts;v->hand_full_query_tx=hand_full_query_tx;v->hand_control_negative_ack=hand_control_negative_ack;v->hand_control_ack_latest_status=hand_control_ack_latest_status;for(int i=0;i<6;i++)v->motor_control_tx[i]=atomic_get(&motor_control_tx[i]);for(int i=0;i<6;i++)v->motor_feedback_count[i]=atomic_get(&motor_feedback_count[i]);for(int i=0;i<5;i++)v->hand_field_feedback_count[i]=atomic_get(&hand_field_feedback_count[i]);v->hand_state_query_mode=atomic_get(&robot.hand_tx_disabled)?4:robot.hand_full_state?3:2;v->hand_mode_guard_phase=robot.hand_guard_phase;v->hand_mode_guard_status=robot.hand_guard_status;v->hand_mode_guard_elapsed_us=robot.hand_guard_elapsed_us;v->hand_mode_guard_token=robot.hand_guard_token;v->can_tx_completed=atomic_get(&tx_good);v->can_tx_failed=atomic_get(&tx_bad);v->can_rx_valid=atomic_get(&rx_good);v->can_rx_malformed=atomic_get(&rx_bad);v->can_rx_unknown=atomic_get(&rx_unknown);v->counter_valid_mask|=bus_status==UINT32_MAX?0x3fu:0x1ffu;v->can_bus_state=bus_status;v->can_tx_error_count=atomic_get(&tx_errors);v->can_rx_error_count=atomic_get(&rx_errors);}

static int can_initialize_attempt(void){
 if(!device_is_ready(bus))return -ENODEV;
 int rc=can_set_mode(bus,CAN_MODE_FD); /* driver explicitly selects automatic bus-off recovery */
 if(!rc)rc=can_set_bitrate(bus,1000000);
 if(!rc)rc=can_set_bitrate_data(bus,5000000);
 if(rc)return rc;
 const struct can_filter ext={.id=0,.mask=0,.flags=CAN_FILTER_IDE},standard={.id=0,.mask=0};
 int a=can_add_rx_filter(bus,rx_cb,NULL,&ext);if(a<0)return a;
 int b=can_add_rx_filter(bus,rx_cb,NULL,&standard);if(b<0){can_remove_rx_filter(bus,a);return b;}
 rc=can_start(bus);
 if(rc){can_remove_rx_filter(bus,a);can_remove_rx_filter(bus,b);}
 return rc;
}
static void can_thread(void*a,void*b,void*c){
 ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);k_sem_take(&can_begin,K_FOREVER);
 bool announced=false;int previous=0;
 for(;;){
  int rc=can_initialize_attempt();can_init_result=rc;
  if(!announced){can_initialization_done(rc);announced=true;}
  fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_INIT,(uint32_t)rc,rc!=0);
  if(rc==0){printk("CAN ready: motor IDs11..16(hex) hand08 1M/5M; auto bus recovery\n");break;}
  if(rc!=previous){printk("CAN init %d; retaining network/control tasks, retrying\n",rc);previous=rc;}
  k_sleep(K_SECONDS(1));
 }
 uint64_t next=0,hand_next=0,hand_control_next=0,hand_aux_next=0,fault_next=0;uint32_t last_hand=0;unsigned hand_component=0,fast_component=0,next_motor_joint=0,motor_fraction=0;
 while(1){struct can_frame rx;for(int i=0;i<32&&!k_msgq_get(&rxq,&rx,K_NO_WAIT);i++)process_rx_frame(&rx);
 uint64_t now=robot_us();arm_command_value_t arm;hand_command_value_t hand;configure_request_value_t cfg;uint8_t active,arm_active,pending_mode,disable_mask;bool home_owned,home_stop;uint32_t hs;unsigned ar,hr;k_spinlock_key_t key=k_spin_lock(&robot_lock);arm=robot.arm;hand=robot.hand;cfg=robot.config;active=robot.active;pending_mode=robot.hand_mode_request;disable_mask=robot.motor_disable_request;hs=robot.hand_seq;if(hand_seen[1]&&now-hand_seen[1]>500000)robot.hand_feedback.online=false;ar=robot.arm_received;hr=robot.hand_received;arm_active=active;home_stop=home_stop_required(&local_home,local_home_updated,now);home_owned=local_home.phase!=HOME_IDLE;if(home_owned){arm_active=local_home.output && (now<local_home_updated || now-local_home_updated<10000);ar=arm_active;memcpy(arm.mit,local_home.mit,sizeof(arm.mit));arm.joint_mask=63;disable_mask=0;}for(int i=0;i<6;i++)if(arm_seen[i])robot.arm_feedback.feedback_age_us[i]=(uint32_t)(now-arm_seen[i]);for(int i=0;i<5;i++)if(hand_seen[i])robot.hand_feedback.field_age_us[i]=(uint32_t)(now-hand_seen[i]);k_spin_unlock(&robot_lock,key);
 stopped_can_update(&stop_bus,home_stop);
 bool hand_enabled=!atomic_get(&robot.hand_tx_disabled);
 if(!hand_enabled){hand_query_pending=false;hand_next=now;hand_aux_next=now;fault_next=now;last_hand=hs;}
 if(hand_query_pending&&now-hand_query_sent>=5000){hand_query_pending=false;hand_query_timeouts++;}
 if(disable_mask){for(unsigned j=0;j<6;j++)if(disable_mask&(1u<<j)){struct robot_can_frame f={.id=(0x8000u|robot_motor_id(j)),.size=8};uint8_t stop[]={1,0,0,0x14,4,0,0x11,0x0f};memcpy(f.data,stop,8);if(!send(&f)){key=k_spin_lock(&robot_lock);robot.motor_disable_request&=~(1u<<j);k_spin_unlock(&robot_lock,key);}}}
 /* New motor IDs0x11..0x16: hand mode changes never touch J4. */
 if(hand_enabled&&pending_mode&&!hand_query_pending){struct robot_can_frame f;uint8_t d[]={0,pending_mode};hand_encode(&f,3,7,d,2);int rc=send(&f);if(!rc){hand_query_pending=true;hand_query_cmd=3;hand_query_index=7;hand_query_sent=now;hand_mode_started=now;hand_mode_expected=pending_mode;}key=k_spin_lock(&robot_lock);robot.hand_mode_request=0;robot.hand_guard_status=rc;robot.hand_guard_phase=rc?8:1;k_spin_unlock(&robot_lock,key);}
 if((robot.hand_guard_phase==1||robot.hand_guard_phase==2)&&!hand_query_pending)request_hand(4,7);
 if(now>=next){next+=333;motor_fraction+=2;if(motor_fraction>=6){next++;motor_fraction-=6;}if(now>next+2000)next=now;for(int one=0;one<1;one++){int j=next_motor_joint;next_motor_joint=(next_motor_joint+1)%6;if(robot.hand_isolation && !home_owned && (!IS_ENABLED(CONFIG_ROBOT_LOCAL_HOME) || robot.lease))continue;
 struct robot_can_frame f;float scale,off,ks,ds;bool cal=robot_motor_conversion(j,&scale,&off,&ks,&ds);
 enum stopped_can_action stop_action=STOPPED_CAN_QUIET;
 if(home_stop){
  stop_action=stopped_can_action(&stop_bus,j,now,motor_mode_seen[j],robot.arm_feedback.motor_mode_raw[j],robot.state_stream_active&&!robot.hand_isolation);
  if(stop_action==STOPPED_CAN_QUIET)continue;
 }
 if(home_stop&&stop_action==STOPPED_CAN_STOP){ht_encode_stop(&f,robot_motor_id(j));}else if(arm_active&&!home_owned&&robot.zero_mit_load){const float zero[5]={0};ht_encode_mit(&f,robot_motor_id(j),zero);}else if(arm_active&&ar&&cal&&(arm.joint_mask&(1u<<j))){float v[5]={arm.mit[j*5]/TWO_PI,arm.mit[j*5+1]/TWO_PI,arm.mit[j*5+2]*ks,arm.mit[j*5+3]*ds,(arm.mit[j*5+4]-off)/scale};ht_encode_mit(&f,robot_motor_id(j),v);}else{memset(&f,0,sizeof(f));f.id=(0x8000u|robot_motor_id(j));f.size=8;uint8_t q[]={0x1c,4,0,0x11,0x0f,0x50,0x50,0x50};memcpy(f.data,q,8);}int send_result=send(&f);if(home_stop&&stop_action==STOPPED_CAN_STOP&&!send_result)stopped_can_sent(&stop_bus,j,now);}}
 if(robot.motor_probe_base){key=k_spin_lock(&robot_lock);uint8_t base=robot.motor_probe_base;robot.motor_probe_base=0;k_spin_unlock(&robot_lock,key);for(int probe=0;probe<6;probe++){struct robot_can_frame q={.id=0x8000u|(base+probe),.size=8};const uint8_t data[]={0x1c,4,0,0x11,0x0f,0x50,0x50,0x50};memcpy(q.data,data,8);send(&q);}}
 if(hand_enabled){
 if(robot.hand_read_all_once&&request_hand(0x0a,0)){key=k_spin_lock(&robot_lock);robot.hand_read_all_once=false;k_spin_unlock(&robot_lock,key);}
 bool control_waiting=active&&hr&&hs!=last_hand;
 if((robot.hand_reply_driven?(!active||!hr||hand_read_control_generation!=hand_control_tx):now>=hand_next)){unsigned fields=robot.hand_full_state?1:1+robot.hand_current_fast+robot.hand_velocity_fast;uint32_t period=robot.hand_poll_period_us/(robot.hand_angle_fast?fields:1);uint8_t idx;if(robot.hand_full_state)idx=0;else if(!robot.hand_angle_fast)idx=1+(hand_component++%4);else{unsigned slot=fast_component%fields;idx=slot==0?3:slot==1&&robot.hand_current_fast?2:4;}if(request_hand(0x0a,idx)){hand_read_control_generation=hand_control_tx;if(robot.hand_angle_fast)fast_component++;hand_next+=period;if(hand_next<now)hand_next=now;}}
 if(robot.hand_angle_fast&&(robot.hand_full_state||robot.hand_current_fast||robot.hand_velocity_fast)&&now>=hand_aux_next&&hand_next>now+200&&hand_next<=now+400){hand_aux_next=now+(robot.hand_full_state?100000:10000);unsigned aux=robot.hand_full_state?3+hand_component++%2:hand_component++%5;if(aux<3){const uint8_t part[]={1,2,4};uint8_t idx=part[aux];if(!(idx==2&&robot.hand_current_fast)&&!(idx==4&&robot.hand_velocity_fast))request_hand(0x0a,idx);}else request_hand(aux==3?7:4,aux==3?1:7);}
 if(!robot.hand_angle_fast&&now>=fault_next){fault_next=now+100000;request_hand(7,1);request_hand(4,7);}
 /* The user presets the physical mode off-bus. Readback is diagnostic;
  * it neither switches the hand nor gates commands on a mode-query reply. */
 if(control_waiting&&now>=hand_control_next){
 /* Hand device integer representation: reject unrepresentable encoding, do not clamp. */
 uint8_t d[28]={0};size_t n=cfg.hand_mode==3?14:cfg.hand_mode==2?28:26;bool valid=true;if(cfg.hand_mode==3){d[0]=cfg.hand_soft_current_ma[0]>>8;d[1]=cfg.hand_soft_current_ma[0];}else{d[0]=hand.duration_ms>>8;d[1]=hand.duration_ms;}
 for(int i=0;i<6;i++){float x=hand.target_q_rad[i]*1000;if(!isfinite(x)||x<0||x>65535){valid=false;break;}uint16_t u=(uint16_t)lroundf(x);d[2+2*i]=u>>8;d[3+2*i]=u;if(cfg.hand_mode!=3){uint32_t cur=cfg.hand_mode==1?cfg.hand_position_current_limit_ma[i]:cfg.hand_soft_current_ma[i];if(cur>65535){valid=false;break;}d[14+2*i]=cur>>8;d[15+2*i]=cur;}}
 if(cfg.hand_mode==2){uint32_t ff=cfg.hand_control_feedforward_ma;if(ff>65535)valid=false;d[26]=ff>>8;d[27]=ff;}
 if(valid){struct robot_can_frame f;hand_encode(&f,9,cfg.hand_mode,d,n);if(!send(&f)){last_hand=hs;hand_control_tx++;hand_control_next+=2000;if(hand_control_next<now)hand_control_next=now;}}}
 } /* hand_enabled: queries, auxiliary reads and control all disabled together. */
 /* No UART prints in the real-time CAN loop: polling console stalls scheduling. */
 enum can_state state;struct can_bus_err_cnt errors;if(!can_get_state(bus,&state,&errors)){bus_status=state;atomic_set(&tx_errors,errors.tx_err_cnt);atomic_set(&rx_errors,errors.rx_err_cnt);fault_note(ROBOT_DEVICE_MCU,0,ROBOT_CAN_BUS,(uint32_t)state,state!=CAN_STATE_ERROR_ACTIVE);}uint64_t end_now=robot_us();uint64_t hand_wake=hand_enabled?(hand_query_pending?hand_query_sent+5000:hand_next):next;uint64_t wake=next<hand_wake?next:hand_wake;if(wake<=end_now)wake=end_now+100;struct can_frame arrived;if(!k_msgq_get(&rxq,&arrived,K_TIMEOUT_ABS_US(wake)))process_rx_frame(&arrived);}}
K_THREAD_DEFINE(can_task,4096,can_thread,NULL,NULL,NULL,4,0,0);
