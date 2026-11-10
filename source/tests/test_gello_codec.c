#include "gello/gello_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void packet(uint8_t id,uint16_t pos,uint16_t vel,uint8_t error,uint8_t p[10]) {
    uint8_t init[]={255,255,id,6,error,pos&255,pos>>8,vel&255,vel>>8,0};
    memcpy(p,init,10);uint8_t sum=0;for(int i=2;i<9;i++)sum+=p[i];p[9]=~sum;
}
static bool feed(struct gello_parser *p,const uint8_t *s,size_t n,struct gello_sample *o) {
    bool seen=false;for(size_t i=0;i<n;i++)seen|=gello_parse(p,s[i],o);return seen;
}
int main(int argc,char **argv) {

    uint8_t q[8],p[10];gello_read_request(1,q);
    const uint8_t expected[]={255,255,1,4,2,56,4,188};assert(!memcmp(q,expected,8));
    struct gello_parser parser={0};struct gello_sample sample={0};
    assert(!feed(&parser,q,8,&sample)); /* local echo cannot be accepted */
    packet(1,0x800a,0x8005,0,p);assert(feed(&parser,p,10,&sample));
    assert(sample.id==1 && sample.raw==4086 && sample.velocity==-5);
    packet(7,110,42,4,p);assert(!feed(&parser,p,10,&sample));
    packet(7,110,42,0,p);p[9]^=1;assert(!feed(&parser,p,10,&sample));
    packet(7,110,42,0,p);assert(feed(&parser,p,10,&sample));assert(sample.raw==110);
    const uint8_t noise[]={0,255,3,255,255,255,255,1,255};
    feed(&parser,noise,sizeof(noise),&sample);assert(feed(&parser,p,10,&sample));
    for(unsigned i=0;i<14;i++) {
        uint8_t id=gello_servo_id(i);assert(gello_servo_index(id)==(int)i);
        packet(id,100+i,42,0,p);assert(feed(&parser,p,10,&sample));assert(sample.id==id);
        gello_read_request(id,q);assert(q[2]==id && q[4]==2);
    }
    for(uint8_t id=8;id<=10;id++) { packet(id,100,0,0,p);assert(!feed(&parser,p,10,&sample)); }
    assert(gello_servo_index(0)==-1 && gello_servo_index(18)==-1 && gello_servo_id(14)==0);
    struct gello_wire_sample samples[14];
    for(unsigned i=0;i<14;i++)samples[i]=(struct gello_wire_sample){.raw=100+i,.ticks=4196+i,.velocity=-5,.age_us=1000+i,.valid=true};
    uint32_t counters[5]={7,8,9,10,11};uint8_t wire[GELLO_DUAL_REPLY_SIZE+1];
    memset(wire,0xa5,sizeof(wire));
    assert(gello_encode(wire,false,123,456,42,100000,12,true,samples,counters)==136);
    assert(!memcmp(wire,"GS01",4) && wire[28]==127 && wire[29]==1 && wire[30]==12 && wire[136]==0xa5);
    assert(gello_encode(wire,true,123,456,42,100000,12,true,samples,counters)==224);
    assert(!memcmp(wire,"GS02",4) && wire[28]==255 && wire[29]==63 && wire[30]==1 && wire[31]==2 && wire[32]==12 && wire[224]==0xa5);
    if(argc==2) { FILE *file=fopen(argv[1],"wb");assert(file);assert(fwrite(wire,1,224,file)==224);assert(!fclose(file)); }
    samples[7].age_us=150000;samples[13].valid=false;
    gello_encode(wire,true,123,456,42,100000,12,true,samples,counters);
    assert(wire[28]==127 && wire[29]==31); /* stale first and invalid last node of group 1 */
    struct gello_tracker t={0};assert(gello_track(&t,4090));assert(gello_track(&t,4));assert(t.ticks==4100);
    assert(gello_track(&t,4095));assert(t.ticks==4095);
    assert(!gello_track(&t,1000));assert(t.ticks==4095); /* isolated glitch */
    assert(gello_track(&t,4090));assert(t.ticks==4090 && !t.pending);
    assert(!gello_track(&t,1000));assert(!gello_track(&t,1500));
    assert(gello_track(&t,1502));assert(t.ticks==5598); /* second physical read confirms */
    assert(!gello_track(&t,4096));
    puts("GELLO framing/echo/checksum/status/signed-magnitude/wrap/jump tests passed");
}
