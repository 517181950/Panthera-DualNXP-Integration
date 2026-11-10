#ifndef ROBOT_PROTOCOL_WIRE_H
#define ROBOT_PROTOCOL_WIRE_H
#include <stddef.h>
#include <stdint.h>
#define RB_HEADER_SIZE 20u
#define RB_MAX_PAYLOAD 256u
#define RB_MAX_PACKET (RB_HEADER_SIZE + RB_MAX_PAYLOAD + 4u)
#define RB_ECHO_REQUEST 0x01u
#define RB_ECHO_REPLY 0x81u
struct rb_packet {
    uint8_t type;
    uint32_t sequence;
    uint64_t host_timestamp;
    const uint8_t *payload;
    uint16_t payload_size;
};
uint32_t rb_crc32(const uint8_t *data, size_t size);
uint32_t rb_get_u32(const uint8_t *p);
void rb_put_u32(uint8_t *p, uint32_t v);
void rb_put_u64(uint8_t *p, uint64_t v);
int rb_decode(const uint8_t *data, size_t size, struct rb_packet *packet);
int rb_encode(uint8_t *data, size_t capacity, const struct rb_packet *packet);
#endif
