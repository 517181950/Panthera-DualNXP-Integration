#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Feetech read-only protocol. See Winora_Firmwares firmware/lib/fetech.
 * No servo writes, zero writes, or torque commands are implemented here. */
struct gello_parser { uint8_t bytes[36]; size_t used; uint32_t bad; };
struct gello_sample { uint8_t id; uint16_t raw; int16_t velocity; };
struct gello_tracker {
    bool valid, pending;
    uint16_t raw, candidate;
    int32_t ticks;
};
void gello_read_request(uint8_t id, uint8_t out[8]);
bool gello_parse(struct gello_parser *p, uint8_t byte, struct gello_sample *out);
bool gello_track(struct gello_tracker *t, uint16_t raw);

#define GELLO_SERVO_COUNT 14
#define GELLO_LEGACY_REPLY_SIZE 136
#define GELLO_DUAL_REPLY_SIZE 224
/* Two disjoint groups on one bus: IDs 1..7 and 11..17. */
int gello_servo_index(uint8_t id);
uint8_t gello_servo_id(unsigned index);
struct gello_wire_sample { uint16_t raw; int16_t velocity; int32_t ticks; uint32_t age_us; bool valid; };
size_t gello_encode(uint8_t *out, bool dual, uint32_t nonce, uint64_t boot,
                    uint32_t scan, uint64_t now, uint16_t node, bool available,
                    const struct gello_wire_sample samples[GELLO_SERVO_COUNT],
                    const uint32_t counters[5]);
