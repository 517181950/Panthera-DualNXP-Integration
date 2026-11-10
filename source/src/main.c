#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/printk.h>
#include <wirelink/link.h>
#include <wirelink/port.h>
#include <wirelink/frame.h>
#include "core/controller.h"
#include "homing/local_home.h"
#ifdef CONFIG_ROBOT_PEER_LINK
#include "peer/peer_link.h"
#endif
#include "core/fault_manager.h"
#include <errno.h>
#include <string.h>
BUILD_ASSERT(CONFIG_ROBOT_RT_PORT != CONFIG_ROBOT_MGMT_PORT, "Realtime and management ports must differ");

struct channel {wl_ctx_t link;uint8_t payload[512],tx[544],ctl[544],rx[544];int sock;uint16_t port;bool online;struct sockaddr_in peer;};
static struct channel rt={.sock=-1},mgmt={.sock=-1};
static uint32_t seq[4];
K_SEM_DEFINE(network_ready,0,1);
void robot_can_diagnostics(diagnostics_value_t *v);
int robot_can_initialize(void);
struct management_reply {uint16_t id,len;uint8_t bytes[128];};
static struct management_reply replies[8];
static unsigned reply_head,reply_count;
static bool management_inflight,management_is_fault;
static wl_tx_handle_t management_handle;
static uint64_t management_event_id;
static uint32_t management_retry_at;static bool management_retry_wait;
struct response_cache {bool used;uint16_t request_id,response_id,request_len,response_len;uint32_t operation_id;uint64_t source_boot;uint8_t request[512],response[128];};
static struct response_cache response_cache[8];static unsigned next_cache;

static void queue_response(uint16_t id,const uint8_t*d,size_t n){
 if(n>sizeof(replies[0].bytes)||reply_count==8){robot.tx_drop++;fault_note(ROBOT_DEVICE_UDP,CONFIG_ROBOT_MGMT_PORT,1,ENOBUFS,true);return;}
 struct management_reply*r=&replies[(reply_head+reply_count)%8];r->id=id;r->len=n;memcpy(r->bytes,d,n);reply_count++;
}
/* Canonical request excludes only transport-independent timing/sequence fields.
 * Full canonical bytes, not a short hash, detect operation-ID reuse conflicts. */
static int cached_response(uint16_t id,uint32_t operation,uint64_t boot,const uint8_t*d,size_t n){
 for(unsigned i=0;i<8;i++){
  const struct response_cache*c=&response_cache[i];
  if(!c->used||c->request_id!=id||c->operation_id!=operation||c->source_boot!=boot)continue;
  if(c->request_len!=n||memcmp(c->request,d,n))return -1;
  queue_response(c->response_id,c->response,c->response_len);return 1;
 }
 return 0;
}
static void remember_response(uint16_t request_id,uint16_t response_id,uint32_t operation,uint64_t boot,const uint8_t*request,size_t request_len,const uint8_t*response,size_t response_len){
 if(request_len>512||response_len>128)return;
 struct response_cache*c=&response_cache[next_cache++%8];
 c->used=true;c->request_id=request_id;c->response_id=response_id;c->operation_id=operation;c->source_boot=boot;c->request_len=request_len;c->response_len=response_len;
 memcpy(c->request,request,request_len);memcpy(c->response,response,response_len);
}
static wl_sink_result_t sink(void*u,wl_io_token_t token,const uint8_t*d,size_t n){
 ARG_UNUSED(token);struct channel*c=u;
 int rc=c->online?zsock_sendto(c->sock,d,n,ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&c->peer,sizeof(c->peer)):-1;
 int error=c->online?errno:ENETDOWN;
 if(rc==(int)n){fault_note(ROBOT_DEVICE_UDP,c->port,1,0,false);return WL_SINK_SENT;}
 robot.tx_drop++;fault_note(ROBOT_DEVICE_UDP,c->port,1,(uint32_t)error,true);return WL_SINK_FAILED;
}
static void channel_close(struct channel*c){c->online=false;if(c->sock>=0)zsock_close(c->sock);c->sock=-1;}
static int channel_init(struct channel*c,int port){
 c->port=port;c->online=false;c->sock=zsock_socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
 if(c->sock<0)return -errno;
 struct sockaddr_in local={.sin_family=AF_INET,.sin_port=htons(port)};
 if(zsock_bind(c->sock,(struct sockaddr*)&local,sizeof(local))){int error=errno;channel_close(c);return -error;}
 c->peer.sin_family=AF_INET;c->peer.sin_port=htons(port);zsock_inet_pton(AF_INET,CONFIG_NET_CONFIG_PEER_IPV4_ADDR,&c->peer.sin_addr);
 uint64_t session=((uint64_t)sys_rand32_get()<<32)|sys_rand32_get();if(!session)session=1;
 const wl_config_t cfg={.max_payload_len=512,.envelope=WL_ENVELOPE_NATIVE_PACKET,.integrity=WL_INTEGRITY_CRC32C,.session_id=session,.max_retries=5,.ack_timeout_ms=20,.max_transmission_unit=544};
 const wl_storage_t storage={.tx_payload=c->payload,.tx_payload_size=sizeof(c->payload),.tx_unit=c->tx,.tx_unit_size=sizeof(c->tx),.control_unit=c->ctl,.control_unit_size=sizeof(c->ctl),.rx_fallback=c->rx,.rx_fallback_size=sizeof(c->rx)};
 int rc=wl_init(&c->link,&cfg,&storage);if(!rc)rc=wl_set_sink(&c->link,sink,c);
 if(rc){channel_close(c);return rc;}c->online=true;return 0;
}
static void send_msg(struct channel*c,uint16_t id,uint8_t*d,size_t n,bool reliable){
 if(reliable){queue_response(id,d,n);return;}
 /* Independent TX owns its frame buffer, never the RX-owned link context. */
 uint8_t bytes[544];size_t count=0;
 const wl_wire_packet_t packet={.type=WL_PACKET_DATA,.integrity=WL_INTEGRITY_CRC32C,.message_id=id,.payload=d,.payload_len=n};
 if(wl_frame_encode(&packet,WL_ENVELOPE_NATIVE_PACKET,bytes,sizeof(bytes),&count)){robot.tx_drop++;return;}
 (void)sink(c,0,bytes,count);
}
static void management_terminal(const wl_event_t*e){
 wl_tx_result_t terminal;bool success=e->type==WL_EVT_TX_SUCCESS;
 (void)wl_tx_take(&mgmt.link,e->handle,&terminal);
 if(!management_inflight||e->handle!=management_handle)return;
 if(success){
  if(management_is_fault)fault_event_delivered(management_event_id);
  else if(reply_count){reply_head=(reply_head+1)%8;reply_count--;}
  fault_note(ROBOT_DEVICE_UDP,CONFIG_ROBOT_MGMT_PORT,2,0,false);
 }else{
  fault_note(ROBOT_DEVICE_UDP,CONFIG_ROBOT_MGMT_PORT,2,(uint32_t)(e->io_result?e->io_result:-ETIMEDOUT),true);
  management_retry_at=(uint32_t)k_uptime_get()+50;
  management_retry_wait=true;
 }
 management_inflight=false;
}
static void management_pump(void){
 uint32_t now=(uint32_t)k_uptime_get();
 if(!mgmt.online||management_inflight)return;
 if(management_retry_wait){if((int32_t)(now-management_retry_at)<0)return;management_retry_wait=false;}
 uint8_t encoded[512];size_t n=0;uint16_t id;
 if(reply_count){struct management_reply*r=&replies[reply_head];id=r->id;n=r->len;memcpy(encoded,r->bytes,n);management_is_fault=false;}
 else{fault_event_value_t event;if(!fault_event_peek(&event,&management_event_id))return;
  if(fault_event_value_encode(&event,encoded,sizeof(encoded),&n))return;
  id=769;management_is_fault=true;}
 int rc=wl_send_reliable(&mgmt.link,id,encoded,n,now,&management_handle);
 if(!rc)management_inflight=true;
}
static void dispatch(struct channel*c,const wl_event_t*e){
 uint8_t out[128],canonical[512];size_t n=0,canonical_len=0;int status=-1;
 if(c==&rt&&e->message_id==257){arm_command_value_t v;if(!arm_command_value_decode(e->payload,e->payload_len,&v))status=robot_arm(&v);}
 else if(c==&rt&&e->message_id==1){stream_header_value_t h;if(!stream_header_value_decode(e->payload,e->payload_len,&h)&&robot_header_ok(&h,false)){k_spinlock_key_t key=k_spin_lock(&robot_lock);robot.state_probe_seq=h.sequence;robot.state_probe_count++;robot.state_probe_at=robot_us();k_spin_unlock(&robot_lock,key);status=0;}}
 else if(c==&rt&&e->message_id==258){hand_command_value_t v;if(!hand_command_value_decode(e->payload,e->payload_len,&v))status=robot_hand(&v);}
 else if(c==&mgmt&&e->type==WL_EVT_RELIABLE_RX&&e->message_id==1025){
  operation_request_value_t v;operation_result_value_t r;init_operation_result(&r);
  if(!operation_request_value_decode(e->payload,e->payload_len,&v)){
   uint32_t request_sequence=v.header.sequence;uint64_t source=v.header.source_boot_id;
   v.header.sequence=0;v.header.tx_monotonic_us=0;
   if(operation_request_value_encode(&v,canonical,sizeof(canonical),&canonical_len))goto malformed;
   int cached=cached_response(1025,v.operation_id,source,canonical,canonical_len);if(cached==1){robot.rx++;return;}
   if(reply_count==8){robot.tx_drop++;return;} /* No operation is applied when response storage is unavailable. */
   status=cached<0?-EINVAL:robot_operation(&v);
   robot_header(&r.header,request_sequence);r.operation_id=v.operation_id;r.status=status;r.reason=cached<0?2:status?1:0;r.observed_run_state=robot.active;r.applied_policy_revision=robot.revision;r.control_lease=robot.lease;
   if(!operation_result_value_encode(&r,out,sizeof(out),&n)){queue_response(1026,out,n);if(!cached)remember_response(1025,1026,v.operation_id,source,canonical,canonical_len,out,n);}
   robot.rx++;return;
  }
 }else if(c==&mgmt&&e->type==WL_EVT_RELIABLE_RX&&e->message_id==1027){
  configure_request_value_t v;configure_result_value_t r;init_configure_result(&r);
  if(!configure_request_value_decode(e->payload,e->payload_len,&v)){
   uint32_t request_sequence=v.header.sequence;uint64_t source=v.header.source_boot_id;v.header.sequence=0;v.header.tx_monotonic_us=0;
   if(configure_request_value_encode(&v,canonical,sizeof(canonical),&canonical_len))goto malformed;
   int cached=cached_response(1027,v.operation_id,source,canonical,canonical_len);if(cached==1){robot.rx++;return;}
   if(reply_count==8){robot.tx_drop++;return;}
   status=cached<0?-EINVAL:robot_configure(&v);robot_header(&r.header,request_sequence);r.operation_id=v.operation_id;r.status=status;r.reason=cached<0?2:status?1:0;r.applied_policy_revision=robot.revision;r.capabilities=3;
   if(!configure_result_value_encode(&r,out,sizeof(out),&n)){queue_response(1028,out,n);if(!cached)remember_response(1027,1028,v.operation_id,source,canonical,canonical_len,out,n);}
   robot.rx++;return;
  }
 }else{fault_note(ROBOT_DEVICE_UDP,c->port,0,e->message_id,true);robot.bad++;return;}
 if(status>=0){robot.rx++;fault_note(ROBOT_DEVICE_UDP,c->port,0,0,false);return;}
malformed:
 robot.bad++;fault_note(ROBOT_DEVICE_UDP,c->port,0,(uint32_t)-EINVAL,true);
}
static void process_event(struct channel*c,const wl_event_t*e){
 if(e->type==WL_EVT_UNRELIABLE_RX||e->type==WL_EVT_RELIABLE_RX)dispatch(c,e);
 else if(e->type>=WL_EVT_TX_SUCCESS){if(c==&mgmt)management_terminal(e);else{wl_tx_result_t result;(void)wl_tx_take(&c->link,e->handle,&result);}}
}
static bool service(struct channel*c){
 uint8_t bytes[545];bool healthy=true;
 for(int i=0;i<32;i++){
  struct sockaddr_in peer;socklen_t size=sizeof(peer);
  int n=zsock_recvfrom(c->sock,bytes,sizeof(bytes),ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&peer,&size);
  if(n<0){if(errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR){fault_note(ROBOT_DEVICE_UDP,c->port,0,(uint32_t)errno,true);if(errno==EBADF||errno==ENOTSOCK)healthy=false;}break;}
  if(peer.sin_addr.s_addr!=c->peer.sin_addr.s_addr||peer.sin_port!=c->peer.sin_port){robot.bad++;fault_note(ROBOT_DEVICE_UDP,c->port,0,EPERM,true);continue;}
  int rc=wl_feed_unit(&c->link,bytes,n);if(rc){robot.bad++;fault_note(ROBOT_DEVICE_UDP,c->port,0,(uint32_t)rc,true);continue;}
  for(int j=0;j<4;j++){wl_event_t e;if(wl_poll(&c->link,(uint32_t)k_uptime_get(),&e)||e.type==WL_EVT_NONE)break;process_event(c,&e);wl_event_release(&c->link,&e);}
 }
 wl_event_t e;if(!wl_poll(&c->link,(uint32_t)k_uptime_get(),&e)&&e.type!=WL_EVT_NONE){process_event(c,&e);wl_event_release(&c->link,&e);}
 return healthy;
}
static void telemetry(int which){uint8_t p[512];size_t n=0;uint16_t id=513+which;uint64_t now=robot_us();
 if(which==0){arm_state_value_t v;k_spinlock_key_t key=k_spin_lock(&robot_lock);v=robot.arm_feedback;v.last_applied_command_sequence=robot.arm_seq;v.command_age_us=robot.arm_received?(uint32_t)(now-robot.arm_at):UINT32_MAX;v.policy_revision=robot.revision;v.run_state=robot.active;v.control_owner=local_home.phase!=HOME_IDLE?2:robot.lease?1:0;v.last_state_probe_sequence=robot.state_probe_seq;k_spin_unlock(&robot_lock,key);robot_header(&v.header,++seq[0]);arm_state_value_encode(&v,p,sizeof(p),&n);}
 if(which==1){hand_state_value_t v;k_spinlock_key_t key=k_spin_lock(&robot_lock);v=robot.hand_feedback;v.last_applied_command_sequence=robot.hand_seq;v.command_age_us=robot.hand_received?(uint32_t)(now-robot.hand_at):UINT32_MAX;k_spin_unlock(&robot_lock,key);robot_header(&v.header,++seq[1]);hand_state_value_encode(&v,p,sizeof(p),&n);}
 if(which==2){fault_snapshot_value_t v;init_fault_snapshot(&v);robot_header(&v.header,++seq[2]);v.policy_revision=robot.revision;fault_snapshot_fill(&v);k_spinlock_key_t key=k_spin_lock(&robot_lock);memcpy(v.arm_fault_raw,robot.arm_feedback.fault_code_raw,sizeof(v.arm_fault_raw));v.arm_fault_valid_mask=robot.arm_feedback.motor_fault_valid_mask;memcpy(v.hand_fault_raw,robot.hand_feedback.fault_code_raw,sizeof(v.hand_fault_raw));v.hand_fault_valid_mask=robot.hand_feedback.fault_valid_mask;k_spin_unlock(&robot_lock,key);fault_snapshot_value_encode(&v,p,sizeof(p),&n);}
 if(which==3){diagnostics_value_t v;init_diagnostics(&v);robot_header(&v.header,++seq[3]);robot_can_diagnostics(&v);v.state_probe_received=robot.state_probe_count;v.last_state_probe_sequence=robot.state_probe_seq;v.udp_rx_valid=robot.rx;v.udp_rx_bad=robot.bad;v.udp_rx_old=robot.old;v.udp_tx_drop=robot.tx_drop;v.control_period_mean_us=robot.cycles?(uint32_t)(robot.period_sum/robot.cycles):0;v.control_period_max_us=robot.period_max;v.control_jitter_abs_max_us=robot.jitter_max;v.counter_valid_mask|=0xfe00;diagnostics_value_encode(&v,p,sizeof(p),&n);}
 if(n)send_msg(&rt,id,p,n,false);}

/* Absolute deadlines prevent work time and scheduler quantisation accumulating
 * into the next period. RX/RPC work cannot postpone the telemetry release. */
static void telemetry_task(void *a,void *b,void *c)
{
 ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
 k_sem_take(&network_ready,K_FOREVER);
 uint64_t origin=robot_us()+2000;
 uint64_t next[4]={origin,origin+500,origin+1000,origin+1500};
 for(;;){
  uint32_t periods[4];
  k_spinlock_key_t key=k_spin_lock(&robot_lock);
  periods[0]=robot.config.arm_state_period_us;
  periods[1]=robot.config.hand_state_period_us;
  k_spin_unlock(&robot_lock,key);
  periods[2]=50000;periods[3]=100000;
  uint64_t now=robot_us(),wake=UINT64_MAX;
  for(int i=0;i<4;i++){
   if(now>=next[i]){
    telemetry(i);
    /* Do not burst old snapshots after a missed deadline. */
    next[i]+=periods[i];
    uint64_t finished=robot_us();
    if(finished>=next[i])next[i]+=((finished-next[i])/periods[i]+1)*periods[i];
   }
   if(next[i]<wake)wake=next[i];
  }
  k_sleep(K_TIMEOUT_ABS_US(wake));
 }
}
K_THREAD_DEFINE(telemetry_thread,4096,telemetry_task,NULL,NULL,NULL,5,0,0);


static void network_open_until_ready(void){
 for(;;){
  int rc=channel_init(&rt,CONFIG_ROBOT_RT_PORT);if(!rc)rc=channel_init(&mgmt,CONFIG_ROBOT_MGMT_PORT);
  if(!rc){fault_note(ROBOT_DEVICE_UDP,0,0,0,false);return;}
  channel_close(&rt);channel_close(&mgmt);fault_note(ROBOT_DEVICE_UDP,0,0,(uint32_t)rc,true);
  printk("UDP init %d; retrying without stopping CAN/control\n",rc);k_sleep(K_SECONDS(1));
 }
}
int main(void){
 robot_init();local_home_init();int can_result=robot_can_initialize();printk("CAN init result=%d\n",can_result);
#if defined(CONFIG_ROBOT_GELLO_UART) || defined(CONFIG_ROBOT_GELLO_USB)
 extern void gello_uart_start(void);
 gello_uart_start();
#endif
 k_thread_priority_set(k_current_get(),7);
 printk("Robot v2 resilient gateway, %s peer %s\n",CONFIG_NET_CONFIG_MY_IPV4_ADDR,CONFIG_NET_CONFIG_PEER_IPV4_ADDR);
 network_open_until_ready();k_sem_give(&network_ready);
#ifdef CONFIG_ROBOT_PEER_LINK
 panthera_peer_start();
#endif
 for(;;){
  struct zsock_pollfd descriptors[2]={{.fd=rt.sock,.events=ZSOCK_POLLIN},{.fd=mgmt.sock,.events=ZSOCK_POLLIN}};
  int ready=zsock_poll(descriptors,2,5);
  bool rt_ok=service(&rt),management_ok=service(&mgmt);
  if(!rt_ok||!management_ok||(ready<0&&errno==EBADF)){
   channel_close(&rt);channel_close(&mgmt);management_inflight=false;
   network_open_until_ready();
  }
  management_pump();
 }
 return 0;
}
