/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_FRAME_H_
#define INCLUDE_WIRELINK_FRAME_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/profile.h"
#include "wirelink/span.h"
#include "wirelink/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WL_FRAME_MARKER 0xA0U
#define WL_FRAME_MARKER_MASK 0xF0U
#define WL_FRAME_VERSION 1U
#define WL_FRAME_VERSION_SHIFT 2U
#define WL_FRAME_VERSION_MASK 0x0CU
#define WL_FRAME_KIND_MASK 0x03U
#define WL_FRAME_PREFIX                                                     \
  (WL_FRAME_MARKER | (WL_FRAME_VERSION << WL_FRAME_VERSION_SHIFT))
#define WL_FRAME_BASE_HEADER_SIZE 4U
#define WL_FRAME_RELIABLE_HEADER_SIZE 16U
/* Worst-case header size, used by storage requirement helpers. */
#define WL_FRAME_HEADER_SIZE WL_FRAME_RELIABLE_HEADER_SIZE
#define WL_FRAME_MAX_PAYLOAD 2048U
#define WL_FRAME_MAX_CRC 4U
#define WL_FRAME_MAX_RAW_LEN (WL_FRAME_HEADER_SIZE + WL_FRAME_MAX_PAYLOAD + WL_FRAME_MAX_CRC)

/* COBS output plus the terminating zero delimiter. */
#define WL_FRAME_MAX_COBS_LEN                                                \
  (WL_FRAME_MAX_RAW_LEN + (WL_FRAME_MAX_RAW_LEN / 254U) + 2U)

typedef int32_t wl_packet_type_t;
enum {
  WL_PACKET_DATA = 0x01,
  WL_PACKET_ACK = 0x02,
  WL_PACKET_NACK = 0x03,
  WL_PACKET_PRIVATE_USE = 0x80,
};

typedef int32_t wl_packet_flag_t;
enum {
  /* V1: only bit0 is defined as WL_PACKET_FLAG_RELIABLE. */
  WL_PACKET_FLAG_RELIABLE = 0x01,
  /* Used to reject illegal non-zero reserved bits deterministically. */
  WL_PACKET_FLAG_RESERVED_MASK = 0xFEU,
};

typedef struct {
  uint8_t marker_version_kind;
  uint8_t reserved;
  uint8_t version;
  uint8_t header_length;
  uint8_t packet_type;
  uint8_t flags;
  uint64_t session_id;
  uint32_t sequence;
  uint16_t message_id;
  uint16_t payload_length;
} wl_frame_header_t;

typedef struct {
  wl_packet_type_t type;
  wl_integrity_t integrity;
  uint8_t flags;
  uint16_t message_id;
  uint64_t session_id;
  uint32_t sequence;
  const uint8_t *payload;
  size_t payload_len;
} wl_wire_packet_t;

typedef struct {
  wl_packet_type_t type;
  uint8_t flags;
  uint16_t message_id;
  uint64_t session_id;
  uint32_t sequence;
  wl_integrity_t integrity;
  wl_span_t payload;
  wl_span_t integrity_bytes;
  size_t total_len;
} wl_frame_view_t;

/* Internal hard limit used by the current parser/encoder implementation. */
#define WL_PACKET_MAX_PAYLOAD WL_FRAME_MAX_PAYLOAD

size_t wl_frame_overhead(wl_integrity_t integrity);
size_t wl_frame_packet_header_size(wl_packet_type_t type, uint8_t flags);
size_t wl_frame_encode_overhead(wl_envelope_type_t envelope, wl_integrity_t integrity);
size_t wl_frame_raw_size(size_t payload_len, wl_integrity_t integrity);

/* Overlap is allowed and produces the same result as disjoint buffers. */
int wl_frame_encode(const wl_wire_packet_t *packet, wl_envelope_type_t envelope,
                   uint8_t *out, size_t out_cap, size_t *out_len);
int wl_frame_decode(const uint8_t *in, size_t in_len, wl_integrity_t integrity,
                   wl_frame_view_t *out_view);
wl_err_t wl_frame_validate_header(const wl_frame_header_t *hdr);

#ifdef __cplusplus
}
#endif

#endif // INCLUDE_WIRELINK_FRAME_H_
