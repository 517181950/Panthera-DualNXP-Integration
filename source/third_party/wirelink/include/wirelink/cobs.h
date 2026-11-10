#ifndef INCLUDE_WIRELINK_COBS_H_
#define INCLUDE_WIRELINK_COBS_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

size_t wl_cobs_encoded_max_size(size_t input_len);
int wl_cobs_encode(const uint8_t *input, size_t input_len, uint8_t *output,
                   size_t output_capacity, size_t *output_len);
int wl_cobs_decode(const uint8_t *input, size_t input_len, uint8_t *output,
                   size_t output_capacity, size_t *output_len);
/* Decoding is safe in place because COBS output never exceeds its input. */
int wl_cobs_decode_in_place(uint8_t *buffer, size_t input_len,
                            size_t *output_len);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // INCLUDE_WIRELINK_COBS_H_
