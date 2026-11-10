/* Diagnostic capture: every API TX attempt, completion and RX callback.
 * Timestamps are MCU software times, not hardware arbitration timestamps.
 * All integers in this trace-only format are little endian (MCXN947).
 */
#include "can_trace.h"
#include "core/controller.h"
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
struct __packed trace_record {uint64_t us;uint32_t seq,id;uint8_t direction,flags,len;int16_t status;uint8_t data[64];};
BUILD_ASSERT(sizeof(struct trace_record)==85);
K_MSGQ_DEFINE(trace_queue,sizeof(struct trace_record),512,4);
static struct k_spinlock trace_lock;
static bool tracing_live;
static atomic_t trace_enabled=1;
void robot_can_trace_enable(bool enabled){atomic_set(&trace_enabled,enabled);}
bool robot_can_trace_enabled(void){return atomic_get(&trace_enabled)!=0;}
static uint32_t trace_sequence;static atomic_t queue_drops,udp_drops;
void robot_can_trace(const struct can_frame*f,unsigned dir,int status){
 if(!tracing_live||!atomic_get(&trace_enabled))return;
 struct trace_record r={.us=robot_us(),.direction=dir,.status=status};
 if(f){r.id=f->id;r.flags=f->flags;r.len=can_dlc_to_bytes(f->dlc);if(r.len>64)r.len=64;memcpy(r.data,f->data,r.len);}
 k_spinlock_key_t key=k_spin_lock(&trace_lock);r.seq=++trace_sequence;
 if(k_msgq_put(&trace_queue,&r,K_NO_WAIT))atomic_inc(&queue_drops);
 k_spin_unlock(&trace_lock,key);
}
/* 20 + 11*85 + IPv4/UDP(28) = 983 bytes: one 1024-byte ENET data block.
 * Twelve records require a second TX fragment, which stalled this driver in HIL. */
static void trace_worker(void*a,void*b,void*c){
 ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);k_sleep(K_SECONDS(1));
 int sock=-1;uint32_t packet=0;
 struct sockaddr_in peer={.sin_family=AF_INET,.sin_port=htons(5007)};zsock_inet_pton(AF_INET,CONFIG_NET_CONFIG_PEER_IPV4_ADDR,&peer.sin_addr);
 struct __packed {char magic[4];uint32_t packet,queue_drop,udp_drop,count;struct trace_record record[11];} out;
 memcpy(out.magic,"CTR1",4);tracing_live=true;
 for(;;){
  if(sock<0){sock=zsock_socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(sock<0){k_sleep(K_MSEC(100));continue;}}
  if(k_msgq_get(&trace_queue,&out.record[0],K_MSEC(2)))continue;
  out.count=1;while(out.count<11&&!k_msgq_get(&trace_queue,&out.record[out.count],K_USEC(150)))out.count++;
  out.packet=++packet;out.queue_drop=atomic_get(&queue_drops);out.udp_drop=atomic_get(&udp_drops);
  size_t n=20+out.count*sizeof(struct trace_record);
  if(zsock_sendto(sock,&out,n,ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&peer,sizeof(peer))!=(int)n)atomic_inc(&udp_drops);
 }
}
K_THREAD_DEFINE(can_trace_task,3072,trace_worker,NULL,NULL,NULL,6,0,0);
