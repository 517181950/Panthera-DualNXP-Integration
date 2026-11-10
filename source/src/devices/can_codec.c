#include "can_codec.h"
#include <string.h>
static int full_checksum_compatibility;
void hand_full_checksum_compatibility(int enabled){full_checksum_compatibility=enabled;}
static uint8_t sum(uint32_t id,const uint8_t*d,size_t n){uint32_t s=0;for(int i=0;i<4;i++)s+=(id>>(8*i))&255;while(n--)s+=*d++;return s;}
static uint8_t dlc(size_t n){const uint8_t sizes[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};for(size_t i=0;i<sizeof(sizes);i++)if(n<=sizes[i])return sizes[i];return 0;}
int hand_encode(struct robot_can_frame*f,uint8_t cmd,uint8_t idx,const uint8_t*d,size_t n){if(!f||n>61||(!d&&n)||cmd==0x0c)return -1;memset(f,0,sizeof(*f));size_t len=n+2;f->id=(ROBOT_HAND_HOST_ID<<16)|(ROBOT_HAND_ID<<8)|len;f->size=dlc(len+1);f->data[0]=cmd;f->data[1]=idx;if(n)memcpy(f->data+2,d,n);f->data[len]=sum(f->id,f->data,len);return 0;}
static int checksum_valid(const struct robot_can_frame*f,size_t len){uint8_t expected=sum(f->id,f->data,len);if(expected==f->data[len])return 1;if(!full_checksum_compatibility||f->id!=0x1083c||f->size!=64||len!=60||f->data[0]!=0x0a||f->data[1]!=0||f->data[61]||f->data[62]||f->data[63])return 0;return (uint8_t)(expected+0x24)==f->data[60];}
int hand_decode(const struct robot_can_frame*f,uint8_t*cmd,uint8_t*idx,const uint8_t**d,size_t*n){if(!f||!cmd||!idx||!d||!n||f->size>64)return -1;size_t len=f->id&255;if((f->id>>24)!=0||((f->id>>16)&255)!=ROBOT_HAND_HOST_ID||((f->id>>8)&255)!=ROBOT_HAND_ID||len<2||len>63||f->size!=dlc(len+1)||!checksum_valid(f,len)||f->data[0]==0x0c)return -1;*cmd=f->data[0];*idx=f->data[1];*d=f->data+2;*n=len-2;return 0;}
static void f32(uint8_t*d,float f){uint32_t x;memcpy(&x,&f,4);for(int i=0;i<4;i++)d[i]=x>>(8*i);}
int ht_encode_mit(struct robot_can_frame*f,uint8_t id,const float v[5]){if(!f||!v||!id||id>=127)return -1;memset(f,0,sizeof(*f));f->id=0x8000|id;f->size=32;uint8_t*p=f->data;p[0]=1;p[1]=0;p[2]=0x15;p[3]=0x0f;p[4]=0x20;f32(p+5,v[0]);f32(p+9,v[1]);f32(p+13,v[4]);p[17]=0x0e;p[18]=0x2b;f32(p+19,v[2]);f32(p+23,v[3]);p[27]=0x1c;p[28]=4;p[29]=0;p[30]=0x11;p[31]=0x0f;return 0;}
int ht_feedback_id(uint32_t id){if(id>0xffff||(id&255)!=0)return -1;unsigned src=id>>8;return src>=ROBOT_MOTOR_ID_FIRST&&src<=ROBOT_MOTOR_ID_LAST?(int)src-ROBOT_MOTOR_ID_FIRST:-1;}

/* Program all MIT setpoints/gains while register0 remains STOP. No enable byte. */
int ht_encode_mit_preload(struct robot_can_frame*f,uint8_t id,const float v[5]){
 if(!f||!v||!id||id>=127)return -1;
 memset(f,0,sizeof(*f));f->id=0x8000|id;f->size=32;memset(f->data,0x50,32);
 uint8_t*p=f->data;p[0]=0x0f;p[1]=0x20;f32(p+2,v[0]);f32(p+6,v[1]);f32(p+10,v[4]);p[14]=0x0e;p[15]=0x2b;f32(p+16,v[2]);f32(p+20,v[3]);p[24]=0x11;p[25]=0;return 0;
}
int ht_encode_mit_enable_prepared(struct robot_can_frame*f,uint8_t id,const float v[5]){
 if(ht_encode_mit_preload(f,id,v))return -1;
 /* Repeat setpoints/gains before the LAST mode write within this frame. */
 uint8_t tail[]={1,0,21,0x1c,4,0,0x11,0x0f};memcpy(f->data+24,tail,8);return 0;
}

int ht_encode_stop(struct robot_can_frame *f,uint8_t id) {
 if(!f || !id || id>=127)return -1;
 memset(f,0,sizeof(*f));f->id=0x8000u|id;f->size=8;
 /* Same mode-zero write as operation9; float feedback preserves q precision. */
 const uint8_t stop[]={1,0,0,0x1c,4,0,0x11,0x0f};memcpy(f->data,stop,8);return 0;
}
