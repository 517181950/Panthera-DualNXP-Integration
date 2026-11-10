/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WIRELINK_CODEC_H
#define WIRELINK_CODEC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A borrowed byte sequence. The data may be NULL only when length is zero. */
typedef struct {
    const uint8_t *data;
    size_t length;
} wl_codec_bytes_t;

/** A borrowed, non-NUL-terminated UTF-8 byte sequence. */
typedef struct {
    const char *data;
    size_t length;
} wl_codec_string_t;

/** Result of a generated WLC codec operation. */
typedef int32_t wl_codec_status_t;
enum {
    WL_CODEC_OK = 0,
    WL_CODEC_ERR_MALFORMED,
    WL_CODEC_ERR_OVERFLOW,
    WL_CODEC_ERR_CAPACITY,
    WL_CODEC_ERR_WIRE_TYPE,
    WL_CODEC_ERR_DUPLICATE_FIELD,
    WL_CODEC_ERR_UTF8,
    WL_CODEC_ERR_INVALID_VALUE,
    /** A schema-required field was absent while encoding or decoding. */
    WL_CODEC_ERR_MISSING_REQUIRED_FIELD,
};

#ifdef __cplusplus
}
#endif

#endif /* WIRELINK_CODEC_H */
