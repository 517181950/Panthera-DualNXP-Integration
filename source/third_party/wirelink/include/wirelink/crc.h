#ifndef INCLUDE_WIRELINK_CRC_H_
#define INCLUDE_WIRELINK_CRC_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t wl_crc16_ccitt_false(const uint8_t *data, size_t length);
uint32_t wl_crc32c(const uint8_t *data, size_t length);
size_t wl_crc_size_bytes(uint8_t integrity_selector);

#ifdef __cplusplus
}
#endif

#endif // INCLUDE_WIRELINK_CRC_H_
