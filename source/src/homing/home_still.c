#include "home_still.h"
#include <math.h>
#include <string.h>
#define DEG .01745329251994329577f
bool home_still_update(struct home_still *s,const float q[6],const float dq[6],uint64_t now,bool position_ok) {
    if(!s->initialized) {memcpy(s->velocity,dq,sizeof(s->velocity));s->initialized=true;}
    float dt=(float)(now-s->updated),alpha=dt/(20000.f+dt);s->updated=now;
    bool quiet=position_ok;
    for(int i=0;i<6;i++) {s->velocity[i]+=alpha*(dq[i]-s->velocity[i]);if(fabsf(s->velocity[i])>3*DEG) quiet=false;}
    if(!quiet) {s->since=0;return false;}
    if(!s->since) {s->since=now;memcpy(s->low,q,sizeof(s->low));memcpy(s->high,q,sizeof(s->high));}
    bool moved=false;
    for(int i=0;i<6;i++) {s->low[i]=fminf(s->low[i],q[i]);s->high[i]=fmaxf(s->high[i],q[i]);if(s->high[i]-s->low[i]>.5f*DEG) moved=true;}
    if(moved) {s->since=now;memcpy(s->low,q,sizeof(s->low));memcpy(s->high,q,sizeof(s->high));}
    return now-s->since>=500000;
}
