#include "gello_codec.h"
#include <limits.h>
#include <string.h>

static uint8_t checksum(const uint8_t *p, size_t n) {
    uint8_t sum=0; for(size_t i=0;i<n;i++) sum+=p[i]; return (uint8_t)~sum;
}
static int16_t signed_magnitude(const uint8_t *p) {
    unsigned n=p[0]|((unsigned)p[1]<<8);
    return n&0x8000 ? -(int)(n&0x7fff) : (int)n;
}
void gello_read_request(uint8_t id, uint8_t out[8]) {
    const uint8_t p[8]={0xff,0xff,id,4,2,56,4,0};
    memcpy(out,p,8); out[7]=checksum(out+2,5);
}
static void discard(struct gello_parser *p, size_t n) {
    memmove(p->bytes,p->bytes+n,p->used-n); p->used-=n;
}
bool gello_parse(struct gello_parser *p, uint8_t byte, struct gello_sample *out) {
    if(p->used==sizeof(p->bytes)) { discard(p,1); p->bad++; }
    p->bytes[p->used++]=byte;
    while(p->used>=2) {
        if(p->bytes[0]!=0xff || p->bytes[1]!=0xff) { discard(p,1); continue; }
        if(p->used<4) return false;
        size_t length=p->bytes[3], total=length+4;
        if(p->bytes[2]==0xff || length<2 || length>32) { discard(p,1); p->bad++; continue; }
        if(p->used<total) return false;
        if(checksum(p->bytes+2,total-3)!=p->bytes[total-1]) { discard(p,1); p->bad++; continue; }
        /* Position + velocity replies have LEN=6 and error=0. Read-request
         * echoes have LEN=4, so cannot masquerade as feedback. */
        bool valid=length==6 && p->bytes[4]==0 && gello_servo_index(p->bytes[2])>=0;
        if(valid) {
            out->id=p->bytes[2];
            int pos=signed_magnitude(p->bytes+5);
            out->raw=(uint16_t)((pos%4096+4096)%4096);
            out->velocity=signed_magnitude(p->bytes+7);
        } else p->bad++;
        discard(p,total);
        if(valid) return true;
    }
    return false;
}
static int delta(int a,int b) {
    int d=a-b; if(d>2048)d-=4096; if(d< -2048)d+=4096; return d;
}
bool gello_track(struct gello_tracker *t, uint16_t raw) {
    if(raw>=4096) return false;
    if(!t->valid) { t->raw=raw;t->ticks=raw;t->valid=true;t->pending=false;return true; }
    int d=delta(raw,t->raw);
    if(d>100 || d< -100) {
        int confirm=delta(raw,t->candidate);
        if(!t->pending || confirm>20 || confirm< -20) {
            t->candidate=raw;t->pending=true;return false;
        }
    }
    if((d>0 && t->ticks>INT32_MAX-d)||(d<0 && t->ticks<INT32_MIN-d)) return false;
    t->ticks+=d;t->raw=raw;t->pending=false;return true;
}

int gello_servo_index(uint8_t id) {
    if(id>=1 && id<=7)return id-1;
    if(id>=11 && id<=17)return id-4;
    return -1;
}
uint8_t gello_servo_id(unsigned index) {
    return index<7 ? index+1 : index<14 ? index+4 : 0;
}
static void put16(uint16_t value,uint8_t *p) { p[0]=value;p[1]=value>>8; }
static void put32(uint32_t value,uint8_t *p) { put16(value,p);put16(value>>16,p+2); }
static void put64(uint64_t value,uint8_t *p) { put32(value,p);put32(value>>32,p+4); }
size_t gello_encode(uint8_t *out,bool dual,uint32_t nonce,uint64_t boot,
                    uint32_t scan,uint64_t now,uint16_t node,bool available,
                    const struct gello_wire_sample samples[GELLO_SERVO_COUNT],
                    const uint32_t counters[5]) {
    size_t size=dual?GELLO_DUAL_REPLY_SIZE:GELLO_LEGACY_REPLY_SIZE;
    unsigned count=dual?14:7,offset=dual?36:32;uint16_t mask=0;
    memset(out,0,size);memcpy(out,dual?"GS02":"GS01",4);
    put32(nonce,out+4);put64(boot,out+8);put32(scan,out+16);put64(now,out+20);
    if(dual) { out[30]=available;out[31]=2;put16(node,out+32); }
    else { out[29]=available;put16(node,out+30); }
    for(unsigned i=0;i<count;i++) {
        const struct gello_wire_sample *s=&samples[i];uint8_t *p=out+offset+12*i;
        if(s->valid && s->age_us<150000)mask|=1U<<i;
        put16(s->raw,p);put16((uint16_t)s->velocity,p+2);
        put32((uint32_t)s->ticks,p+4);put32(s->age_us,p+8);
    }
    if(dual)put16(mask,out+28);else out[28]=(uint8_t)mask;
    for(unsigned i=0;i<5;i++)put32(counters[i],out+offset+12*count+4*i);
    return size;
}
