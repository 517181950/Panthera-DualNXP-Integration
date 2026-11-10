#include "peer_codec.h"
#include <string.h>
_Static_assert(sizeof(float) == 4, "Peer protocol requires binary32 float");
static uint32_t crc(const uint8_t *p, size_t n)
{
    static const uint32_t t[16] = {
        0,0x1db71064,0x3b6e20c8,0x26d930ac,0x76dc4190,0x6b6b51f4,0x4db26158,0x5005713c,
        0xedb88320,0xf00f9344,0xd6d6a3e8,0xcb61b38c,0x9b64c2b0,0x86d3d2d4,0xa00ae278,0xbdbdf21c};
    uint32_t c = UINT32_MAX;
    while (n--) { c ^= *p++; c = (c >> 4) ^ t[c & 15]; c = (c >> 4) ^ t[c & 15]; }
    return ~c;
}
static void put(uint8_t **p, uint64_t v, unsigned n) { while (n--) { *(*p)++ = (uint8_t)v; v >>= 8; } }
static uint64_t get(const uint8_t **p, unsigned n) { uint64_t v=0; for(unsigned i=0;i<n;i++) v |= (uint64_t)*(*p)++ << (8*i); return v; }
#define U32S(X) X(active) X(actuators) X(q_mask) X(dq_mask) X(tau_mask) X(fault_mask)
#define COUNTERS(X) X(tx) X(rx) X(tx_error) X(bad) X(old) X(gaps) X(missed) X(reboots)
#define CAN(X) X(hand_control_tx) X(hand_control_ack) X(hand_ack_status) X(can_state) X(can_tx_error) X(can_rx_error)
#define END(X) X(peer_sequence) X(peer_age_us) X(arm_command_sequence) X(hand_command_sequence)
void panthera_peer_encode(const struct panthera_peer_state *s, uint8_t out[PANTHERA_PEER_SIZE])
{
    uint8_t *p=out; memcpy(p,"PHD1",4); p+=4;
    put(&p,3,2); put(&p,PANTHERA_PEER_SIZE,2); put(&p,s->node,2); put(&p,s->target,2);
    put(&p,s->boot,8); put(&p,s->sequence,4); put(&p,s->time_us,8);
#define SCALAR(f) put(&p,s->f,4);
#define ARRAY(f,n) for(int i=0;i<n;i++) put(&p,s->f[i],4);
#define FLOATS(f,n) for(int i=0;i<n;i++){uint32_t v;memcpy(&v,&s->f[i],4);put(&p,v,4);}
    U32S(SCALAR) ARRAY(arm_age_us,6) FLOATS(q,6) FLOATS(dq,6) FLOATS(tau,6)
    ARRAY(fault,6) ARRAY(mode,6) SCALAR(hand_fields) SCALAR(hand_mode) ARRAY(hand_age_us,5) FLOATS(hand_q,11)
    COUNTERS(SCALAR) put(&p,s->peer_boot,8); END(SCALAR)
    ARRAY(motor_control_tx,6) ARRAY(motor_feedback_count,6) ARRAY(hand_field_feedback_count,5) CAN(SCALAR)
    SCALAR(home_state) SCALAR(home_generation) SCALAR(home_error) SCALAR(home_source) FLOATS(home_zero,6)
    SCALAR(home_button_seq) put(&p,s->home_button_target_boot,8); SCALAR(home_button_ref_seq) SCALAR(home_button_age_us) SCALAR(home_button_ack) SCALAR(home_button_status)
    put(&p,crc(out,PANTHERA_PEER_SIZE-4),4);
#undef SCALAR
#undef ARRAY
#undef FLOATS
}
bool panthera_peer_decode(const uint8_t *data, size_t n, struct panthera_peer_state *s)
{
    if(n!=PANTHERA_PEER_SIZE || memcmp(data,"PHD1",4)) return false;
    const uint8_t *p=data+4;
    if(get(&p,2)!=3 || get(&p,2)!=PANTHERA_PEER_SIZE) return false;
    const uint8_t *end=data+n-4;
    if(get(&end,4)!=crc(data,n-4)) return false;
    s->node=get(&p,2); s->target=get(&p,2); s->boot=get(&p,8); s->sequence=get(&p,4); s->time_us=get(&p,8);
#define SCALAR(f) s->f=get(&p,4);
#define ARRAY(f,n) for(int i=0;i<n;i++) s->f[i]=get(&p,4);
#define FLOATS(f,n) for(int i=0;i<n;i++){uint32_t v=get(&p,4);memcpy(&s->f[i],&v,4);}
    U32S(SCALAR) ARRAY(arm_age_us,6) FLOATS(q,6) FLOATS(dq,6) FLOATS(tau,6)
    ARRAY(fault,6) ARRAY(mode,6) SCALAR(hand_fields) SCALAR(hand_mode) ARRAY(hand_age_us,5) FLOATS(hand_q,11)
    COUNTERS(SCALAR) s->peer_boot=get(&p,8); END(SCALAR)
    ARRAY(motor_control_tx,6) ARRAY(motor_feedback_count,6) ARRAY(hand_field_feedback_count,5) CAN(SCALAR)
    SCALAR(home_state) SCALAR(home_generation) SCALAR(home_error) SCALAR(home_source) FLOATS(home_zero,6)
    SCALAR(home_button_seq) s->home_button_target_boot=get(&p,8); SCALAR(home_button_ref_seq) SCALAR(home_button_age_us) SCALAR(home_button_ack) SCALAR(home_button_status)
#undef SCALAR
#undef ARRAY
#undef FLOATS
    return s->boot!=0;
}
bool panthera_peer_newer(uint32_t sequence, uint32_t previous) { return (int32_t)(sequence-previous)>0; }
