#include "fault_manager.h"
#include "controller.h"
static struct robot_fault_ledger ledger;
static struct k_spinlock ledger_lock;
void fault_note(uint8_t cls,uint16_t dev,uint8_t channel,uint32_t raw,bool active){
 k_spinlock_key_t key=k_spin_lock(&ledger_lock);
 uint32_t normalized=robot_fault_normalized(cls,raw);
 if(cls==ROBOT_DEVICE_HAND&&channel>=0x80&&active)normalized=raw==0?(0x2100u|(channel&0x7fu)):UINT32_MAX;
 robot_fault_observe(&ledger,cls,dev,channel,raw,active,normalized,robot_fault_severity(cls,raw),robot_us());
 k_spin_unlock(&ledger_lock,key);
}
void fault_snapshot_fill(fault_snapshot_value_t*v){
 k_spinlock_key_t key=k_spin_lock(&ledger_lock);
 v->latest_event_id=ledger.next_event_id;v->oldest_retained_event_id=ledger.count?ledger.history[ledger.head].event_id:0;v->event_overflow_count=ledger.overflow_count;
 v->mcu_fault_valid_mask=0x0007001f;
 for(size_t i=0;i<ROBOT_FAULT_SLOTS;i++){
  const struct robot_fault_record*r=&ledger.slots[i].value;
  if(!ledger.slots[i].used||!r->active)continue;
  if(r->device_class==ROBOT_DEVICE_MCU&&r->channel<5)v->mcu_fault_active|=1u<<r->channel;
  if(r->device_class==ROBOT_DEVICE_UDP&&r->channel<2)v->mcu_fault_active|=1u<<(16+r->channel);
 }
 /* Observations are not an automatic stop policy. Device faults remain raw. */
 v->local_action=0;
 k_spin_unlock(&ledger_lock,key);
}
bool fault_event_peek(fault_event_value_t*v,uint64_t*id){
 struct robot_fault_record r;k_spinlock_key_t key=k_spin_lock(&ledger_lock);bool ok=robot_fault_peek(&ledger,&r);k_spin_unlock(&ledger_lock,key);if(!ok)return false;
 init_fault_event(v);robot_header(&v->header,(uint32_t)r.event_id);
 v->event_id=r.event_id;v->device_class=r.device_class;v->device_id=r.device_id;v->device_channel=r.channel;v->raw_code=r.raw_code;v->normalized_code=r.normalized_code;v->severity=r.severity;v->active=r.active;v->local_action=0;v->policy_revision=robot.revision;v->first_seen_us=r.first_seen_us;v->last_seen_us=r.last_seen_us;v->occurrences=r.occurrences;*id=r.event_id;return true;
}
void fault_event_delivered(uint64_t id){k_spinlock_key_t key=k_spin_lock(&ledger_lock);robot_fault_ack_delivery(&ledger,id);k_spin_unlock(&ledger_lock,key);}
