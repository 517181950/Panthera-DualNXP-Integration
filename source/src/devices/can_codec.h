#pragma once
#include <stdint.h>
#include <stddef.h>
enum { ROBOT_MOTOR_ID_FIRST=0x11, ROBOT_MOTOR_ID_LAST=0x16 };
static inline uint8_t robot_motor_id(unsigned joint){return (uint8_t)(ROBOT_MOTOR_ID_FIRST+joint);}
enum { ROBOT_HAND_ID=0x08,ROBOT_HAND_HOST_ID=1 };
struct robot_can_frame {uint32_t id;uint8_t size;uint8_t data[64];};
void hand_full_checksum_compatibility(int enabled);
int hand_encode(struct robot_can_frame *f,uint8_t cmd,uint8_t index,const uint8_t *data,size_t n);
int hand_decode(const struct robot_can_frame *f,uint8_t *cmd,uint8_t *index,const uint8_t **data,size_t *n);
int ht_encode_mit(struct robot_can_frame *f,uint8_t id,const float motor_native[5]);
/* Write mode register 0 to STOP, then query feedback including mode/fault. */
int ht_encode_stop(struct robot_can_frame *f,uint8_t id);
/* motor_native order q,dq,kp,kd,tau; model-specific SI conversion belongs above this codec. */
int ht_feedback_id(uint32_t id);

int ht_encode_mit_preload(struct robot_can_frame *f,uint8_t id,const float v[5]);
int ht_encode_mit_enable_prepared(struct robot_can_frame *f,uint8_t id,const float v[5]);
