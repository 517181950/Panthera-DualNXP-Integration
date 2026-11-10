#include "peer/peer_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(int argc, char **argv) {
    struct panthera_peer_state a={.node=12,.target=11,.boot=0x123456789abcdef0ULL,.sequence=0xfffffffe,
        .time_us=123456789012ULL,.active=1,.actuators=1,.q_mask=63,.dq_mask=63,.hand_fields=2,.hand_mode=3,
        .tx=123,.rx=122,.peer_boot=0xabc123,.peer_age_us=2000,.hand_control_ack=456,.home_button_seq=19,.home_button_target_boot=0x12345678abcddcbaULL,.home_button_ref_seq=999,.home_button_age_us=1200,.home_button_ack=18,.home_button_status=2};
    for(int i=0;i<6;i++){a.q[i]=i*.125f-.25f;a.arm_age_us[i]=2000+i;a.fault[i]=i;a.motor_feedback_count[i]=1000+i;}
    for(int i=0;i<11;i++)a.hand_q[i]=i*.0625f;
    uint8_t packet[PANTHERA_PEER_SIZE], bad[PANTHERA_PEER_SIZE];
    panthera_peer_encode(&a,packet);
    struct panthera_peer_state b={0}; assert(panthera_peer_decode(packet,sizeof(packet),&b));
    assert(b.boot==a.boot && b.sequence==a.sequence && b.peer_boot==a.peer_boot && b.hand_control_ack==456);
    assert(b.home_button_seq==19 && b.home_button_target_boot==0x12345678abcddcbaULL && b.home_button_ref_seq==999 && b.home_button_age_us==1200 && b.home_button_ack==18 && b.home_button_status==2);
    assert(!memcmp(a.q,b.q,sizeof(a.q))); assert(!memcmp(a.hand_q,b.hand_q,sizeof(a.hand_q)));
    assert(!memcmp(a.motor_feedback_count,b.motor_feedback_count,sizeof(a.motor_feedback_count)));
    for(unsigned i=0;i<sizeof(packet);i++){memcpy(bad,packet,sizeof(packet));bad[i]^=1;assert(!panthera_peer_decode(bad,sizeof(bad),&b));}
    assert(!panthera_peer_decode(packet,sizeof(packet)-1,&b));
    assert(panthera_peer_newer(0,0xffffffff));assert(!panthera_peer_newer(7,7));assert(!panthera_peer_newer(6,7));
    if(argc==2){FILE*f=fopen(argv[1],"wb");assert(f);assert(fwrite(packet,1,sizeof(packet),f)==sizeof(packet));fclose(f);}
    puts("Peer codec roundtrip, every-byte corruption, truncation and sequence rollover passed");
}
