#include "protocol_wire.h"
#include <string.h>
uint32_t rb_get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}
void rb_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
void rb_put_u64(uint8_t *p, uint64_t v)
{
    rb_put_u32(p, (uint32_t)(v >> 32)); rb_put_u32(p + 4, (uint32_t)v);
}
uint32_t rb_crc32(const uint8_t *data, size_t size)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        }
    }
    return ~crc;
}
int rb_decode(const uint8_t *data, size_t size, struct rb_packet *packet)
{
    if (!data || !packet || size < RB_HEADER_SIZE + 4u || size > RB_MAX_PACKET) return -1;
    if (memcmp(data, "RBT1", 4) || data[4] != 1) return -2;
    uint16_t n = ((uint16_t)data[6] << 8) | data[7];
    if (n > RB_MAX_PAYLOAD || size != RB_HEADER_SIZE + n + 4u) return -3;
    if (rb_crc32(data, size - 4u) != rb_get_u32(data + size - 4u)) return -4;
    packet->type = data[5]; packet->sequence = rb_get_u32(data + 8);
    packet->host_timestamp = ((uint64_t)rb_get_u32(data + 12) << 32) | rb_get_u32(data + 16);
    packet->payload = data + RB_HEADER_SIZE; packet->payload_size = n;
    return 0;
}
int rb_encode(uint8_t *data, size_t capacity, const struct rb_packet *packet)
{
    if (!data || !packet || packet->payload_size > RB_MAX_PAYLOAD ||
        (packet->payload_size && !packet->payload)) return -1;
    size_t size = RB_HEADER_SIZE + packet->payload_size + 4u;
    if (capacity < size) return -1;
    memcpy(data, "RBT1", 4); data[4] = 1; data[5] = packet->type;
    data[6] = (uint8_t)(packet->payload_size >> 8); data[7] = (uint8_t)packet->payload_size;
    rb_put_u32(data + 8, packet->sequence); rb_put_u64(data + 12, packet->host_timestamp);
    if (packet->payload_size) memcpy(data + RB_HEADER_SIZE, packet->payload, packet->payload_size);
    rb_put_u32(data + size - 4u, rb_crc32(data, size - 4u));
    return (int)size;
}
