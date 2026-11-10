#include "homing/home_still.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#define DEG .01745329251994329577f
int main(void) {
 struct home_still s={0};float q[6]={0},v[6]={0};bool ready=false;
 for(uint64_t t=1000;t<=601000;t+=2000) {v[0]=(t/2000)%10==0?3.8f*DEG:0;ready=home_still_update(&s,q,v,t,true);}
 assert(ready); /* Isolated velocity estimator spikes with stationary encoders. */
 s=(struct home_still){0};
 for(uint64_t t=1000;t<=2001000;t+=2000) {q[0]=(t-1000)*1e-6f*2*DEG;v[0]=0;assert(!home_still_update(&s,q,v,t,true));} /* Drift cannot masquerade as settled. */
 s=(struct home_still){0};q[0]=0;v[0]=4*DEG;
 for(uint64_t t=1000;t<=1001000;t+=2000) assert(!home_still_update(&s,q,v,t,true));
 s=(struct home_still){0};v[0]=0;
 for(uint64_t t=1000;t<=1001000;t+=2000) assert(!home_still_update(&s,q,v,t,false));
 puts("Settling: tolerates isolated velocity spikes only with bounded measured pose; rejects continuous speed, encoder drift and position error");
}
