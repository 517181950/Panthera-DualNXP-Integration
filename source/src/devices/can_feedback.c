#include "can_feedback.h"
#include <string.h>
#include <math.h>
static bool legal_size(unsigned n){const unsigned s[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};for(unsigned i=0;i<sizeof(s)/sizeof(s[0]);i++)if(s[i]==n)return true;return false;}
static uint32_t little(const uint8_t*p,unsigned n){uint32_t x=0;for(unsigned i=0;i<n;i++)x|=(uint32_t)p[i]<<(8*i);return x;}
static bool number(const uint8_t*p,unsigned kind,unsigned reg,float*out){
 unsigned width=kind==3?4:1u<<kind;uint32_t raw=little(p,width);
 if(kind==3){memcpy(out,&raw,4);return isfinite(*out);}
 if(raw==(width==1?0x80u:width==2?0x8000u:0x80000000u))return false;
 int32_t v=width==1?(int8_t)raw:width==2?(int16_t)raw:(int32_t)raw;float scale=1;
 if(reg==1)scale=kind==0?100:kind==1?10000:100000;
 if(reg==2)scale=kind==0?100:kind==1?4000:100000;
 if(reg==3)scale=kind==0?2:kind==1?100:1000;
 *out=(float)v/scale;return true;
}
int ht_decode_feedback(const struct robot_can_frame*f,struct ht_feedback*out){
 if(!f||!out||!legal_size(f->size))return ROBOT_PARSE_MALFORMED;
 int j=ht_feedback_id(f->id);if(j<0)return ROBOT_PARSE_OTHER;
 struct ht_feedback r={.joint=j};size_t at=0;
 while(at<f->size){
  uint8_t cmd=f->data[at++];if(cmd==0x50)continue;
  if((cmd>>4)!=2)return ROBOT_PARSE_UNSUPPORTED;
  if(at>=f->size)return ROBOT_PARSE_MALFORMED;
  unsigned kind=(cmd>>2)&3,count=cmd&3,width=kind==3?4:1u<<kind;
  if(!count){count=f->data[at++];if(!count||at>=f->size)return ROBOT_PARSE_MALFORMED;}
  unsigned reg=f->data[at++];if(count>(f->size-at)/width)return ROBOT_PARSE_MALFORMED;
  for(unsigned k=0;k<count;k++,reg++,at+=width){
   if(reg==0||reg==15){
    uint32_t value=little(f->data+at,width);
    if(kind==3){float fp;memcpy(&fp,&value,4);if(!isfinite(fp)||(double)fp<0||(double)fp>4294967295.0||floorf(fp)!=fp)return ROBOT_PARSE_MALFORMED;value=(uint32_t)fp;}
    /* Enum/fault registers retain unsigned raw bits, including unknown 0xFF. */
    if(reg==0){r.mode_raw=value;r.present|=HT_FEEDBACK_MODE;r.valid|=HT_FEEDBACK_MODE;}
    else{r.fault_raw=value;r.present|=HT_FEEDBACK_FAULT;r.valid|=HT_FEEDBACK_FAULT;}
   }else if(reg>=1&&reg<=3){
    uint8_t bit=1u<<(reg-1);float v=0;r.present|=bit;
    if(number(f->data+at,kind,reg,&v)){r.valid|=bit;if(reg==1)r.position_turns=v;else if(reg==2)r.velocity_turns_s=v;else r.torque_native=v;}
   }
  }
 }
 *out=r;return ROBOT_PARSE_OK;
}
static uint16_t be(const uint8_t*p){return ((uint16_t)p[0]<<8)|p[1];}
int hand_decode_feedback(const struct robot_can_frame*f,struct hand_feedback*out){
 if(!f||!out)return ROBOT_PARSE_MALFORMED;
 if(((f->id>>16)&255)!=ROBOT_HAND_HOST_ID||((f->id>>8)&255)!=ROBOT_HAND_ID)return ROBOT_PARSE_OTHER;
 const uint8_t*d;size_t n;struct hand_feedback r={0};
 if(hand_decode(f,&r.cmd,&r.index,&d,&n))return ROBOT_PARSE_MALFORMED;
 if(r.cmd==0x0a){
  if(r.index==0){
   if(n!=58)return ROBOT_PARSE_MALFORMED;
   for(unsigned i=0;i<6;i++)r.position_percent[i]=be(d+2*i);
   for(unsigned i=0;i<6;i++)r.current_ma[i]=(int16_t)be(d+12+2*i);
   for(unsigned i=0;i<11;i++)r.q_rad[i]=be(d+24+2*i)*.001f;
   for(unsigned i=0;i<6;i++)r.dq_rad_s[i]=(int16_t)be(d+46+2*i)*.001f;
   r.fields=15;
  }else{
   if(r.index<1||r.index>4)return ROBOT_PARSE_UNSUPPORTED;
   unsigned bytes=r.index==3?22:12;if(n!=bytes)return ROBOT_PARSE_MALFORMED;
   unsigned group=r.index==1?0:r.index==2?3:r.index==3?1:2;
   float*dest=r.index==1?r.position_percent:r.index==2?r.current_ma:r.index==3?r.q_rad:r.dq_rad_s;
   for(unsigned i=0;i<bytes/2;i++)dest[i]=(r.index==2||r.index==4?(float)(int16_t)be(d+2*i):(float)be(d+2*i))*(r.index>=3?.001f:1);
   r.fields=1u<<group;
  }
 }else if(r.cmd==7&&r.index==1){if(n!=6)return ROBOT_PARSE_MALFORMED;for(unsigned i=0;i<6;i++)r.faults[i]=d[i];r.fields=16;}
 else if(r.cmd==4&&r.index==7){if(n!=2)return ROBOT_PARSE_MALFORMED;r.mode_raw=d[1];r.fields=32;}
 else if((r.cmd==9&&r.index>=1&&r.index<=3)||(r.cmd==3&&r.index==7)){
  if(n!=2)return ROBOT_PARSE_MALFORMED;
  r.ack=true;r.ack_status=be(d);
 }else return ROBOT_PARSE_UNSUPPORTED;
 *out=r;return ROBOT_PARSE_OK;
}
