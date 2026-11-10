#include "homing/paired_home.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static bool receive(struct home_pair *p,uint32_t local_seq,uint64_t boot,const struct home_button_wire *w,uint64_t now) {
 return home_pair_receive(p,11,local_seq,boot,100,11,1000,true,true,w,now);
}
int main(void) {
 struct home_pair a={0};struct home_button_wire w={0},out;
 assert(!home_pair_press(&a,true,1000) && a.status==4);
 assert(!receive(&a,50,12,&w,1000));assert(!receive(&a,51,12,&w,3000));
 assert(home_pair_press(&a,true,5000));home_pair_wire(&a,&out,6000);
 assert(out.seq==1 && out.target_boot==12 && out.target_sequence==100 && out.age_us==1000);
 w=(struct home_button_wire){.seq=1,.target_boot=11,.target_sequence=50,.age_us=2000};
 assert(receive(&a,52,12,&w,7000));assert(!receive(&a,53,12,&w,9000));assert(a.ack==1 && !a.received_committed);w.status=2;assert(!receive(&a,53,12,&w,10000));assert(a.received_committed);
 w.seq=2;w.target_boot=99;assert(!receive(&a,54,12,&w,11000));w.target_boot=11;assert(!receive(&a,55,12,&w,13000)); /* Invalid event consumed, no later replay. */
 w.seq=3;w.age_us=100001;assert(!receive(&a,55,12,&w,15000));
 w.seq=4;w.age_us=1;w.target_sequence=1;assert(!receive(&a,100,12,&w,17000)); /* Receiver reference expires even for queued old packets. */
 w.seq=5;w.target_sequence=100;assert(!receive(&a,100,12,&w,200000));assert(!receive(&a,101,12,&w,202000)); /* Link recovery is NOT a button. */
 w.seq=6;w.target_sequence=100;assert(receive(&a,102,12,&w,204000));
 assert(!receive(&a,103,13,&w,206000));assert(!a.received_committed);assert(!receive(&a,104,13,&w,208000)); /* Reboot cannot replay source event. */
 home_pair_wire(&a,&out,210000);assert(out.status==3 && out.seq==1); /* Incoming request does not emit another button/echo. */
 assert(!home_pair_press(&a,true,300000));assert(!home_pair_press(&a,false,210001));
 struct home_pair b={0};w=(struct home_button_wire){0};receive(&b,10,12,&w,1000);receive(&b,11,12,&w,3000);assert(home_pair_press(&b,true,4000));
 w.ack=1;receive(&b,12,12,&w,5000);assert(b.status==2);
 puts("Paired SW2: live bidirectional link, once-only event, boot binding, local sequence expiry, no replay/echo/reconnect trigger and ACK passed");
}
