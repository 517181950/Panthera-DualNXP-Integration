#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

enum robot_device_class { ROBOT_DEVICE_MCU=0, ROBOT_DEVICE_ARM=1, ROBOT_DEVICE_HAND=2, ROBOT_DEVICE_UDP=3 };
enum { ROBOT_FAULT_SLOTS=32, ROBOT_FAULT_HISTORY=64 };
struct robot_fault_record {
 uint64_t event_id,first_seen_us,last_seen_us;
 uint32_t raw_code,normalized_code,occurrences;
 uint16_t device_id;
 uint8_t device_class,channel,severity;
 bool active;
};
struct robot_fault_slot {bool used;struct robot_fault_record value;};
struct robot_fault_ledger {
 struct robot_fault_slot slots[ROBOT_FAULT_SLOTS];
 struct robot_fault_record history[ROBOT_FAULT_HISTORY];
 uint64_t next_event_id;
 uint32_t overflow_count;
 size_t head,count;
};
/* Observational only: this library has no control/stop/clear-device callbacks. */
void robot_fault_ledger_init(struct robot_fault_ledger *l);
void robot_fault_observe(struct robot_fault_ledger *l,uint8_t cls,uint16_t device,uint8_t channel,
                         uint32_t raw,bool active,uint32_t normalized,uint8_t severity,uint64_t now);
bool robot_fault_peek(const struct robot_fault_ledger *l,struct robot_fault_record *r);
bool robot_fault_ack_delivery(struct robot_fault_ledger *l,uint64_t event_id);
uint8_t robot_fault_severity(uint8_t cls,uint32_t raw);
uint32_t robot_fault_normalized(uint8_t cls,uint32_t raw);
