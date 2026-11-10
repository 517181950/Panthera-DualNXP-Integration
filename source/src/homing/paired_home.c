#include "paired_home.h"
static bool newer(uint32_t a,uint32_t b) {return (int32_t)(a-b)>0;}
bool home_pair_press(struct home_pair *p,bool local_ready,uint64_t now) {
    if(!local_ready) {p->status=5;return false;}
    if(!p->peer_boot || !p->peer_ready || now-p->peer_at>20000) {p->status=4;return false;}
    if(++p->sequence==0) ++p->sequence;
    p->target_boot=p->peer_boot;p->target_sequence=p->peer_sequence;p->sent_at=now;p->status=1;return true;
}
bool home_pair_receive(struct home_pair *p,uint64_t local_boot,uint32_t local_sequence,
    uint64_t remote_boot,uint32_t remote_sequence,uint64_t remote_peer_boot,uint32_t remote_peer_age,
    bool remote_ready,bool local_ready,const struct home_button_wire *w,uint64_t now) {
    bool continuous=p->peer_boot==remote_boot && p->peer_at && now-p->peer_at<=100000;
    if(!continuous) {p->seen=w->seq;p->ack=0;p->received_committed=false;}
    p->peer_boot=remote_boot;p->peer_at=now;p->peer_sequence=remote_sequence;
    p->peer_ready=remote_ready && remote_peer_boot==local_boot && remote_peer_age<=20000;
    if(p->status==1 && remote_boot==p->target_boot && remote_peer_boot==local_boot && w->ack==p->sequence && now-p->sent_at<=100000) p->status=2;
    if(p->ack && w->seq==p->ack && w->target_boot==local_boot && w->status==2 && w->age_us<=100000) p->received_committed=true;
    if(!continuous || !w->seq || !newer(w->seq,p->seen)) return false;
    p->seen=w->seq; /* Consume rejected/expired events too; never replay later. */
    if(!local_ready || !p->peer_ready || w->target_boot!=local_boot || w->age_us>100000 || (uint32_t)(local_sequence-w->target_sequence)>50) return false;
    p->ack=w->seq;p->received_committed=false;return true;
}
void home_pair_wire(struct home_pair *p,struct home_button_wire *w,uint64_t now) {
    uint64_t age=now-p->sent_at;
    if(p->status==1 && age>100000) p->status=3;
    *w=(struct home_button_wire){.seq=p->sequence,.target_boot=p->target_boot,.target_sequence=p->target_sequence,
        .age_us=p->sequence && age<=UINT32_MAX?(uint32_t)age:UINT32_MAX,.ack=p->ack,.status=p->status};
}
