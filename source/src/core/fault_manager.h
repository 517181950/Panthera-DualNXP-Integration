#pragma once
#include "fault_ledger.h"
#include "generated/value_init.h"
enum { ROBOT_CAN_INIT=0,ROBOT_CAN_BUS=1,ROBOT_CAN_TX=2,ROBOT_CAN_RX_BAD=3,ROBOT_CAN_RX_UNKNOWN=4 };
void fault_note(uint8_t cls,uint16_t dev,uint8_t channel,uint32_t raw,bool active);
void fault_snapshot_fill(fault_snapshot_value_t *v);
bool fault_event_peek(fault_event_value_t *v,uint64_t *id);
void fault_event_delivered(uint64_t id);
