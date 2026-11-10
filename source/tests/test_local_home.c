#include "homing/homing.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846f
static struct home_input good(void) {struct home_input in={.q_mask=63,.dq_mask=63,.fault_mask=63};return in;}
static void begin(struct home_context *h,struct home_input *in) {home_request(h,1000,1);home_tick(h,in,1000);home_tick(h,in,501000);assert(h->phase==HOME_RAMP);}
static void simulate_arbitrary_start(const float raw[6]) {
 struct home_context h={0};struct home_input in=good();float previous[6],zero[6];
 memcpy(in.q,raw,sizeof(in.q));memcpy(previous,raw,sizeof(previous));assert(home_path_reference(raw,zero));
 begin(&h,&in);
 for(uint64_t t=503000;t<60000000;t+=2000) {
  home_tick(&h,&in,t);assert(h.phase!=HOME_FAULT);
  for(int j=0;j<6;j++) {
   assert(fabsf(h.mit[5*j]-previous[j])<.002f);
   assert(fabsf(h.mit[5*j+1])<=30*PI/180+1e-6);
   previous[j]=in.q[j]=h.mit[5*j];in.dq[j]=h.mit[5*j+1];
  }
  if(h.phase==HOME_HOLD)break;
 }
 assert(h.phase==HOME_HOLD);
 for(int j=0;j<6;j++)assert(fabsf(in.q[j]-zero[j])<1e-5);
 assert(home_takeover_ok(&h,&in,h.mit,0));
}
int main(void) {
 const float starts[][6]={{0,-164.18f*PI/180,93.55f*PI/180,93.47f*PI/180,0,0},
  {-4*PI,-524.18f*PI/180,-190*PI/180,93.47f*PI/180,350*PI/180,-3*PI},
  {PI,1.5f*PI,4.3f,-2*PI,PI,3*PI}};
 for(unsigned i=0;i<sizeof(starts)/sizeof(starts[0]);i++)simulate_arbitrary_start(starts[i]);
 struct home_context h={0};struct home_input in=good();
 for(unsigned source=1;source<=3;source++) {
  in=good();in.q[1]=-164.18f*PI/180;in.q[3]=93.47f*PI/180;
  home_request(&h,1000,source);home_tick(&h,&in,1000);home_tick(&h,&in,501000);
  assert(h.phase==(source==1?HOME_RAMP:HOME_RAISE) && h.output);
  assert(fabsf(h.zero[1]+2*PI)<1e-5);
  assert(fabsf(h.zero[1]+h.path.goal[1]+1.5f*PI)<1e-5);
  assert(h.path.goal[1]<h.origin[1]); /* -164 -> -270, not +90. */
 }
 in=good();
 home_request(&h,1000,1);in.age[3]=20000;home_tick(&h,&in,1000000);assert(h.phase==HOME_WAIT && !h.output);
 in=good();in.q[1]=-361*PI/180;in.q[0]=-.3f;begin(&h,&in);assert(fabsf(h.zero[1]+2*PI)<1e-6);
 assert(fabsf(h.origin[1]+PI/180)<1e-6);
 float previous[6];memcpy(previous,in.q,sizeof(previous));bool phases[15]={0};unsigned last=0;uint64_t stage_at=0;
 for(uint64_t t=503000;t<60000000;t+=2000) {
  home_tick(&h,&in,t);assert(h.phase!=HOME_FAULT);phases[h.phase]=true;
  if(h.phase!=last) {if(last==HOME_RAISE_SETTLE || last==HOME_ALIGN_SETTLE || last==HOME_LOWER_SETTLE) assert(t-stage_at>=500000);last=h.phase;stage_at=t;}
  if(h.output) for(int i=0;i<6;i++) {
   assert(fabsf(h.mit[5*i+1])<=30*PI/180+1e-6);
   assert(fabsf(h.mit[5*i]-previous[i])<.002f);
   previous[i]=h.mit[5*i];in.q[i]=h.mit[5*i];in.dq[i]=h.mit[5*i+1];
  }
  if(h.phase==HOME_RAISE || h.phase==HOME_RAISE_SETTLE) assert(fabsf(h.mit[0]+.3f)<1e-6);
  if(h.phase==HOME_ALIGN || h.phase==HOME_ALIGN_SETTLE) {
   assert(fabsf((h.mit[5]-h.zero[1])-PI/2)<1e-5);assert(fabsf(h.mit[10]-2.886410263f)<1e-5);
  }
  if(h.phase==HOME_HOLD) break;
 }
 assert(h.phase==HOME_HOLD);for(int p=9;p<=14;p++) assert(phases[p]);
 assert(home_takeover_ok(&h,&in,h.mit,0));float bad[30];memcpy(bad,h.mit,sizeof(bad));bad[5]=0;
 assert(!home_takeover_ok(&h,&in,bad,0));assert(!home_takeover_ok(&h,&in,h.mit,20000));
 in.q[1]+=2*PI;home_tick(&h,&in,60000000);assert(h.phase==HOME_FAULT && h.error==HOME_TURN_JUMP && !h.output);
 in=good();in.q[2]=4.3f;begin(&h,&in);assert(h.phase==HOME_RAMP);
 in=good();begin(&h,&in);in.fault[2]=1;home_tick(&h,&in,503000);assert(h.phase==HOME_FAULT && h.error==HOME_MOTOR_FAULT);
 in=good();begin(&h,&in);in.age[0]=20001;home_tick(&h,&in,503000);assert(h.phase==HOME_FAULT && h.error==HOME_STALE);
 in=good();home_tick(&h,&in,60000000);assert(h.phase==HOME_FAULT); /* Never retry on CAN return. */
 in=good();begin(&h,&in);home_tick(&h,&in,1501000);assert(h.phase==HOME_RAISE);
 uint64_t end=1501000+(uint64_t)(h.path.duration*1000000)+2000;
 home_tick(&h,&in,end);assert(h.phase==HOME_SOFT_HOLD && h.error==HOME_TRACKING && h.output); /* Stop the path, retain gravity support. */
 for(unsigned source=2;source<=3;source++) {
  in=good();in.q[0]=.2f;in.dq[0]=.1f;home_request(&h,1000,source);home_tick(&h,&in,3000);
  assert(h.phase==HOME_WAIT && h.output && h.mit[0]==in.q[0] && h.mit[1]==0);
  in.dq[0]=0;for(uint64_t t=5000;t<=1005000;t+=2000) home_tick(&h,&in,t);assert(h.phase==HOME_RAISE); /* No extra one-second idle gain ramp. */
 }
 in=good();begin(&h,&in);in.q[2]=NAN;home_tick(&h,&in,503000);assert(h.phase==HOME_FAULT && h.error==HOME_NONFINITE);
 /* Joint-specific range permits J3 beyond pi, without blanket half-turn rejection. */
 in=good();in.q[2]=4;begin(&h,&in);assert(h.phase==HOME_RAMP);
 /* One tracking threshold: 4 degrees does not accumulate a second timer. */
 in=good();in.q[2]=4*PI/180;h=(struct home_context){.phase=HOME_HOLD};
 for(uint64_t t=1000;t<=1001000;t+=2000) {home_tick(&h,&in,t);assert(h.phase==HOME_HOLD && h.output);}
 in.q[2]=5.1f*PI/180;home_tick(&h,&in,1003000);assert(h.phase==HOME_SOFT_HOLD && h.error==HOME_TRACKING);
 /* A stalled J2 from a valid zero start aborts early, while other joints follow. */
 in=good();begin(&h,&in);uint64_t stopped_at=0;
 for(uint64_t t=503000;t<5000000;t+=2000) {
  home_tick(&h,&in,t);
  if(h.phase==HOME_SOFT_HOLD) {assert(h.error==HOME_TRACKING);stopped_at=t;break;}
  for(int j=0;j<6;j++) if(j!=1) {in.q[j]=h.mit[5*j];in.dq[j]=h.mit[5*j+1];}
 }
 assert(stopped_at && stopped_at<3000000 && h.output);
 for(int i=0;i<6;i++) assert(fabsf(h.mit[5*i]-in.q[i])<1e-6f);
 assert(!home_stop_required(&h,stopped_at,stopped_at));
 home_tick(&h,&in,stopped_at+2000);assert(h.phase==HOME_SOFT_HOLD && h.output);
 /* Independent CAN owner timeout latches STOP even if control stops ticking. */
 in=good();begin(&h,&in);assert(!home_stop_required(&h,501000,510999));
 assert(!home_stop_required(&h,511001,511000));assert(h.output); /* Control preempted CAN after its time sample. */
 assert(home_stop_required(&h,501000,511000));assert(h.error==HOME_CONTROL_STALE);
 home_request(&h,1000,1);assert(!home_stop_required(&h,0,5000000)); /* Boot waits for feedback without output. */
 home_abort(&h,HOME_MAINTENANCE);assert(home_stop_required(&h,0,5000000));
 puts("Upright home: three-stage continuity/settle gating, speed, J2 wrap, fresh takeover, latched faults and both manual sources passed");
}
