#include "fault_ledger.h"
#include <string.h>
#include <limits.h>

static void increment(uint32_t *n){if(*n<UINT32_MAX)(*n)++;}
void robot_fault_ledger_init(struct robot_fault_ledger*l){memset(l,0,sizeof(*l));}
static void push(struct robot_fault_ledger*l,struct robot_fault_record*r){
 r->event_id=++l->next_event_id;
 if(l->count==ROBOT_FAULT_HISTORY){l->head=(l->head+1)%ROBOT_FAULT_HISTORY;l->count--;increment(&l->overflow_count);}
 l->history[(l->head+l->count)%ROBOT_FAULT_HISTORY]=*r;l->count++;
}
void robot_fault_observe(struct robot_fault_ledger*l,uint8_t cls,uint16_t dev,uint8_t channel,
 uint32_t raw,bool active,uint32_t normalized,uint8_t severity,uint64_t now){
 struct robot_fault_slot*slot=NULL,*empty=NULL;
 for(size_t i=0;i<ROBOT_FAULT_SLOTS;i++){
  if(!l->slots[i].used){if(!empty)empty=&l->slots[i];continue;}
  struct robot_fault_record*r=&l->slots[i].value;
  if(r->device_class==cls&&r->device_id==dev&&r->channel==channel){slot=&l->slots[i];break;}
 }
 if(!slot){
  if(!active)return;
  if(!empty){struct robot_fault_record r={.device_class=cls,.device_id=dev,.channel=channel,.raw_code=raw,.active=true,.normalized_code=normalized,.severity=severity,.occurrences=1,.first_seen_us=now,.last_seen_us=now};increment(&l->overflow_count);push(l,&r);return;}
  slot=empty;slot->used=true;
 }
 struct robot_fault_record*r=&slot->value;
 if(r->active&&active&&r->raw_code==raw){increment(&r->occurrences);r->last_seen_us=now;return;}
 if(r->active){r->active=false;r->last_seen_us=now;push(l,r);}
 if(!active)return;
 *r=(struct robot_fault_record){.device_class=cls,.device_id=dev,.channel=channel,.raw_code=raw,.active=true,.normalized_code=normalized,.severity=severity,.occurrences=1,.first_seen_us=now,.last_seen_us=now};
 push(l,r);
}
bool robot_fault_peek(const struct robot_fault_ledger*l,struct robot_fault_record*r){
 if(!l->count)return false;
 *r=l->history[l->head];
 for(size_t i=0;i<ROBOT_FAULT_SLOTS;i++)if(l->slots[i].used&&l->slots[i].value.event_id==r->event_id){*r=l->slots[i].value;break;}
 return true;
}
bool robot_fault_ack_delivery(struct robot_fault_ledger*l,uint64_t id){
 if(!l->count||l->history[l->head].event_id!=id)return false;
 l->head=(l->head+1)%ROBOT_FAULT_HISTORY;l->count--;return true;
}
uint8_t robot_fault_severity(uint8_t cls,uint32_t raw){
 if(cls==ROBOT_DEVICE_HAND){if(raw==1)return 1;if(raw>=2&&raw<=10)return 3;}
 if(cls==ROBOT_DEVICE_ARM&&(raw==34||raw==38||raw==40||raw==44))return 3;
 return 2;
}
uint32_t robot_fault_normalized(uint8_t cls,uint32_t raw){
 if(raw==0)return 0;
 if(cls==ROBOT_DEVICE_ARM&&((raw>=1&&raw<=7)||(raw>=32&&raw<=47)))return 0x1000+raw;
 if(cls==ROBOT_DEVICE_HAND&&raw>=1&&raw<=10)return 0x2000+raw;
 return UINT32_MAX;
}
