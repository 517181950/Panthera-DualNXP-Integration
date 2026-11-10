/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>

#include "wirelink/crc.h"
#include "wirelink/cobs.h"
#include "wirelink/frame.h"

#include "crc_internal.h"

static uint16_t read_u16_be(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] << 8u) | (uint16_t)p[1];
}

static uint32_t read_u32_be(const uint8_t *p) {
  return ((uint32_t)p[0] << 24u) | ((uint32_t)p[1] << 16u) |
         ((uint32_t)p[2] << 8u) | (uint32_t)p[3];
}

static uint64_t read_u64_be(const uint8_t *p) {
  return ((uint64_t)p[0] << 56u) | ((uint64_t)p[1] << 48u) |
         ((uint64_t)p[2] << 40u) | ((uint64_t)p[3] << 32u) |
         ((uint64_t)p[4] << 24u) | ((uint64_t)p[5] << 16u) |
         ((uint64_t)p[6] << 8u) | (uint64_t)p[7];
}

static void write_u16_be(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8u);
  p[1] = (uint8_t)(v & 0xFFu);
}

static void write_u32_be(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24u);
  p[1] = (uint8_t)(v >> 16u);
  p[2] = (uint8_t)(v >> 8u);
  p[3] = (uint8_t)(v & 0xFFu);
}

static void write_u64_be(uint8_t *p, uint64_t v) {
  p[0] = (uint8_t)(v >> 56u);
  p[1] = (uint8_t)(v >> 48u);
  p[2] = (uint8_t)(v >> 40u);
  p[3] = (uint8_t)(v >> 32u);
  p[4] = (uint8_t)(v >> 24u);
  p[5] = (uint8_t)(v >> 16u);
  p[6] = (uint8_t)(v >> 8u);
  p[7] = (uint8_t)(v & 0xFFu);
}

typedef struct {
  size_t encoded_len;
  unsigned int code;
} wl_cobs_count_state_t;

typedef struct {
  uint8_t *out;
  size_t write_index;
  size_t code_index;
  uint8_t code;
} wl_cobs_encode_state_t;

static uint8_t frame_packet_kind(const wl_wire_packet_t *packet) {
  if (packet->type == WL_PACKET_ACK) {
    return 2U;
  }
  return (packet->flags & WL_PACKET_FLAG_RELIABLE) != 0U ? 1U : 0U;
}

size_t wl_frame_packet_header_size(wl_packet_type_t type, uint8_t flags) {
  if (type == WL_PACKET_ACK && flags == 0U) {
    return WL_FRAME_RELIABLE_HEADER_SIZE;
  }
  if (type != WL_PACKET_DATA ||
      (flags & (uint8_t)~WL_PACKET_FLAG_RELIABLE) != 0U) {
    return 0U;
  }
  return (flags & WL_PACKET_FLAG_RELIABLE) != 0U
             ? WL_FRAME_RELIABLE_HEADER_SIZE
             : WL_FRAME_BASE_HEADER_SIZE;
}

static void frame_write_header(const wl_wire_packet_t *packet, size_t header_len,
                                uint8_t header[WL_FRAME_HEADER_SIZE]) {
  header[0] = (uint8_t)(WL_FRAME_PREFIX | frame_packet_kind(packet));
  header[1] = 0U;
  write_u16_be(&header[2], packet->message_id);
  if (header_len == WL_FRAME_RELIABLE_HEADER_SIZE) {
    write_u64_be(&header[4], (uint64_t)packet->session_id);
    write_u32_be(&header[12], packet->sequence);
  }
}

static size_t frame_write_integrity(const wl_wire_packet_t *packet,
                                    const uint8_t *header,
                                    size_t header_len,
                                    uint8_t integrity[WL_FRAME_MAX_CRC]) {
  if (packet->integrity == WL_INTEGRITY_CRC16) {
    uint16_t state = wl_crc16_ccitt_false_update(
        0xFFFFu, header, header_len);
    state = wl_crc16_ccitt_false_update(state, packet->payload,
                                        packet->payload_len);
    write_u16_be(integrity, state);
    return 2U;
  }

  if (packet->integrity == WL_INTEGRITY_CRC32C) {
    uint32_t state =
        wl_crc32c_update(0xFFFFFFFFu, header, header_len);
    state = wl_crc32c_update(state, packet->payload, packet->payload_len);
    write_u32_be(integrity, state ^ 0xFFFFFFFFu);
    return 4U;
  }

  return 0U;
}

static void cobs_count_span(wl_cobs_count_state_t *state, const uint8_t *data,
                            size_t length) {
  /* Each input byte contributes one output byte, including a replaced zero.
   * Only a full nonzero run contributes an extra code byte. */
  size_t encoded_len = state->encoded_len + length;
  unsigned int code = state->code;
  for (size_t i = 0U; i < length; ++i) {
    code = data[i] == 0U ? 1U : code + 1U;
    if (code == 0xFFU) {
      ++encoded_len;
      code = 1U;
    }
  }
  state->encoded_len = encoded_len;
  state->code = code;
}

static size_t frame_cobs_encoded_size(const uint8_t *header,
                                      size_t header_len,
                                      const uint8_t *payload,
                                      size_t payload_len,
                                      const uint8_t *integrity,
                                      size_t integrity_len) {
  wl_cobs_count_state_t state = {.encoded_len = 1U, .code = 1U};

  cobs_count_span(&state, header, header_len);
  cobs_count_span(&state, payload, payload_len);
  cobs_count_span(&state, integrity, integrity_len);
  return state.encoded_len;
}

static void cobs_encode_span(wl_cobs_encode_state_t *state,
                             const uint8_t *data, size_t length) {
  if (length == 0U) {
    return;
  }
  /* State is private stack storage, separate from every input/output span.
   * Publish once per span instead of reloading it after every byte store.
   * The caller stages aliased payload ahead of output; read each byte before
   * writing it to preserve that forward-encoding invariant. */
  uint8_t *const out = state->out;
  size_t write_index = state->write_index;
  size_t code_index = state->code_index;
  uint8_t code = state->code;
  for (size_t i = 0U; i < length; ++i) {
    const uint8_t value = data[i];
    if (value == 0U) {
      out[code_index] = code;
      code = 1U;
      code_index = write_index++;
      continue;
    }

    out[write_index++] = value;
    ++code;
    if (code == 0xFFU) {
      out[code_index] = code;
      code = 1U;
      code_index = write_index++;
    }
  }
  state->write_index = write_index;
  state->code_index = code_index;
  state->code = code;
}

static size_t frame_cobs_encode(const uint8_t *header,
                                size_t header_len,
                                const uint8_t *payload, size_t payload_len,
                                const uint8_t *integrity,
                                size_t integrity_len, uint8_t *out) {
  wl_cobs_encode_state_t state = {
      .out = out, .write_index = 1U, .code_index = 0U, .code = 1U};

  cobs_encode_span(&state, header, header_len);
  cobs_encode_span(&state, payload, payload_len);
  cobs_encode_span(&state, integrity, integrity_len);
  state.out[state.code_index] = state.code;
  return state.write_index;
}

static int spans_overlap(const uint8_t *left, size_t left_len,
                         const uint8_t *right, size_t right_len) {
  uintptr_t left_address;
  uintptr_t right_address;

  if (left_len == 0U || right_len == 0U) {
    return 0;
  }

  left_address = (uintptr_t)left;
  right_address = (uintptr_t)right;
  if (left_address <= right_address) {
    return (right_address - left_address) < left_len;
  }
  return (left_address - right_address) < right_len;
}

static int frame_envelope_is_valid(wl_envelope_type_t envelope) {
  int64_t value = (int64_t)envelope;
  return value >= (int64_t)WL_ENVELOPE_COBS_STREAM &&
         value <= (int64_t)WL_ENVELOPE_BUS_LENGTH16;
}

static int frame_integrity_is_valid(wl_integrity_t integrity) {
  int64_t value = (int64_t)integrity;
  return value >= (int64_t)WL_INTEGRITY_NONE &&
         value <= (int64_t)WL_INTEGRITY_CRC32C;
}

static inline void frame_write_raw(const uint8_t *header,
                            size_t header_len,
                            const wl_wire_packet_t *packet,
                            const uint8_t *integrity, size_t integrity_len,
                            uint8_t *out) {
  /* Moving the payload first also preserves supported overlapping inputs. */
  if (packet->payload_len != 0U) {
    memmove(out + header_len, packet->payload, packet->payload_len);
  }
  /* Both validated header variants share the four-byte base. Fixed-size
   * copies let the compiler inline these small fields on packet envelopes. */
  memcpy(out, header, WL_FRAME_BASE_HEADER_SIZE);
  if (header_len == WL_FRAME_RELIABLE_HEADER_SIZE) {
    memcpy(out + WL_FRAME_BASE_HEADER_SIZE, header + WL_FRAME_BASE_HEADER_SIZE,
           WL_FRAME_RELIABLE_HEADER_SIZE - WL_FRAME_BASE_HEADER_SIZE);
  }
  if (integrity_len != 0U) {
    memcpy(out + header_len + packet->payload_len, integrity,
           integrity_len);
  }
}

size_t wl_frame_overhead(wl_integrity_t integrity) {
  if (!frame_integrity_is_valid(integrity)) {
    return 0U;
  }
  return (size_t)WL_FRAME_HEADER_SIZE + wl_crc_size_bytes((uint8_t)integrity);
}

size_t wl_frame_encode_overhead(wl_envelope_type_t envelope,
                               wl_integrity_t integrity) {
  size_t raw_size;

  if (!frame_envelope_is_valid(envelope) ||
      !frame_integrity_is_valid(integrity)) {
    return 0U;
  }
  raw_size = wl_frame_raw_size(0, integrity);
  if (raw_size == 0) {
    return 0;
  }

  switch (envelope) {
  case WL_ENVELOPE_COBS_STREAM:
    return wl_cobs_encoded_max_size(raw_size) + 1;
  case WL_ENVELOPE_BUS_LENGTH16:
    return raw_size + 2;
  case WL_ENVELOPE_NATIVE_PACKET:
    return raw_size;
  default:
    return 0U;
  }
}

size_t wl_frame_raw_size(size_t payload_len, wl_integrity_t integrity) {
  size_t crc_len;

  if (!frame_integrity_is_valid(integrity)) {
    return 0U;
  }
  crc_len = wl_crc_size_bytes((uint8_t)integrity);
  if (payload_len > (SIZE_MAX - WL_FRAME_HEADER_SIZE - crc_len)) {
    return 0;
  }
  return WL_FRAME_HEADER_SIZE + payload_len + crc_len;
}

wl_err_t wl_frame_validate_header(const wl_frame_header_t *hdr) {
  if (hdr == NULL) {
    return WL_ERR_INVALID_ARG;
  }

  if ((hdr->marker_version_kind & WL_FRAME_MARKER_MASK) != WL_FRAME_MARKER ||
      hdr->reserved != 0U) {
    return WL_ERR_BAD_FRAME;
  }
  if (hdr->version != WL_FRAME_VERSION) {
    return WL_ERR_PROTOCOL_VERSION;
  }
  if (hdr->payload_length > WL_PACKET_MAX_PAYLOAD) {
    return WL_ERR_FRAME_TOO_LONG;
  }

  switch (hdr->packet_type) {
  case WL_PACKET_DATA:
    if (hdr->message_id == 0U) {
      return WL_ERR_BAD_FRAME;
    }
    if (hdr->flags & ~WL_PACKET_FLAG_RELIABLE) {
      return WL_ERR_BAD_FRAME;
    }
    if ((hdr->flags & WL_PACKET_FLAG_RELIABLE) != 0U) {
      if (hdr->header_length != WL_FRAME_RELIABLE_HEADER_SIZE ||
          hdr->session_id == 0ULL) {
        return WL_ERR_BAD_FRAME;
      }
    } else if (hdr->header_length != WL_FRAME_BASE_HEADER_SIZE ||
               hdr->session_id != 0ULL || hdr->sequence != 0U) {
      return WL_ERR_BAD_FRAME;
    }
    break;
  case WL_PACKET_ACK:
    if (hdr->message_id != 0U || hdr->payload_length != 0U || hdr->flags != 0U ||
        hdr->header_length != WL_FRAME_RELIABLE_HEADER_SIZE ||
        hdr->session_id == 0ULL) {
      return WL_ERR_BAD_FRAME;
    }
    break;
  case WL_PACKET_NACK:
    return WL_ERR_NOT_SUPPORTED;
  default:
    return WL_ERR_NOT_SUPPORTED;
  }

  return WL_OK;
}

int wl_frame_encode(const wl_wire_packet_t *packet, wl_envelope_type_t envelope,
                   uint8_t *out, size_t out_cap, size_t *out_len) {
  uint8_t header[WL_FRAME_HEADER_SIZE];
  uint8_t integrity[WL_FRAME_MAX_CRC];
  size_t raw_len = 0;
  size_t crc_len;
  size_t header_len;

  if (out_len == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  *out_len = 0;

  if (packet == NULL || (packet->payload_len != 0 && packet->payload == NULL)) {
    return WL_ERR_INVALID_ARG;
  }
  if (!frame_envelope_is_valid(envelope) ||
      !frame_integrity_is_valid(packet->integrity)) {
    return WL_ERR_INVALID_ARG;
  }
  if (packet->type == WL_PACKET_DATA) {
    if (packet->message_id == 0U) {
      return WL_ERR_INVALID_ARG;
    }
    if (packet->flags & ~WL_PACKET_FLAG_RELIABLE) {
      return WL_ERR_INVALID_ARG;
    }
    if ((packet->flags & WL_PACKET_FLAG_RELIABLE) != 0U &&
        packet->session_id == 0ULL) {
      return WL_ERR_INVALID_ARG;
    }
  } else if (packet->type == WL_PACKET_ACK) {
    if (packet->message_id != 0U || packet->payload_len != 0U ||
        packet->flags != 0U) {
      return WL_ERR_INVALID_ARG;
    }
    if (packet->session_id == 0ULL) {
      return WL_ERR_INVALID_ARG;
    }
  } else {
    return WL_ERR_BAD_FRAME;
  }
  if (packet->payload_len > WL_PACKET_MAX_PAYLOAD) {
    return WL_ERR_PAYLOAD_TOO_LONG;
  }

  header_len = wl_frame_packet_header_size(packet->type, packet->flags);
  raw_len = header_len + packet->payload_len +
            wl_crc_size_bytes((uint8_t)packet->integrity);
  if (raw_len == 0 || raw_len > WL_FRAME_MAX_RAW_LEN) {
    return WL_ERR_PAYLOAD_TOO_LONG;
  }

  frame_write_header(packet, header_len, header);
  crc_len = frame_write_integrity(packet, header, header_len, integrity);

  switch (envelope) {
  case WL_ENVELOPE_COBS_STREAM: {
    const uint8_t *payload = packet->payload;
    size_t cobs_len = wl_cobs_encoded_max_size(raw_len);
    const int overlap = spans_overlap(payload, packet->payload_len, out, cobs_len);
    /* Normal link storage fits the worst case. Disjoint input can therefore
     * be encoded in one pass. Tight buffers and aliases still need the exact
     * size: preserve failure-before-write and the overlap staging offset. */
    if (out_cap < cobs_len + 1U || overlap) {
      cobs_len = frame_cobs_encoded_size(
          header, header_len, payload, packet->payload_len, integrity, crc_len);
    }
    if (out_cap < cobs_len + 1U) {
      return WL_ERR_BUF_TOO_SMALL;
    }
    if (out == NULL) {
      return WL_ERR_INVALID_ARG;
    }
    if (overlap) {
      /*
       * COBS expands by at least one byte. Staging the aliased payload at
       * its logical raw-frame offset leaves enough lead for safe forward
       * encoding, while keeping the normal non-aliased path copy-free. A
       * worst-case-only overlap may stage unnecessarily, but all staging
       * writes fit the exact output size and need no second overlap check.
       */
      payload = out + (cobs_len - raw_len) + header_len;
      memmove((uint8_t *)payload, packet->payload, packet->payload_len);
    }
    cobs_len = frame_cobs_encode(header, header_len, payload, packet->payload_len,
                                 integrity, crc_len, out);
    out[cobs_len] = 0U;
    *out_len = cobs_len + 1U;
    return WL_OK;
  }
  case WL_ENVELOPE_BUS_LENGTH16:
    if (raw_len > UINT16_MAX) {
      return WL_ERR_FRAME_TOO_LONG;
    }
    if (out_cap < raw_len + 2U) {
      return WL_ERR_BUF_TOO_SMALL;
    }
    if (out == NULL) {
      return WL_ERR_INVALID_ARG;
    }
    frame_write_raw(header, header_len, packet, integrity, crc_len, out + 2U);
    write_u16_be(out, (uint16_t)raw_len);
    *out_len = raw_len + 2U;
    return WL_OK;
  case WL_ENVELOPE_NATIVE_PACKET:
  default:
    if (out_cap < raw_len) {
      return WL_ERR_BUF_TOO_SMALL;
    }
    if (out == NULL) {
      return WL_ERR_INVALID_ARG;
    }
    frame_write_raw(header, header_len, packet, integrity, crc_len, out);
    *out_len = raw_len;
    return WL_OK;
  }
}

int wl_frame_decode(const uint8_t *in, size_t in_len, wl_integrity_t integrity,
                   wl_frame_view_t *out_view) {
  wl_frame_header_t hdr = {0};
  size_t header_len;
  size_t payload_len;
  size_t crc_len;
  uint8_t kind;
  uint16_t expected_crc16;
  uint16_t check_crc16;
  uint32_t expected_crc32;
  uint32_t check_crc32;

  if (out_view == NULL || in == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (!frame_integrity_is_valid(integrity)) {
    return WL_ERR_INVALID_ARG;
  }

  crc_len = wl_crc_size_bytes((uint8_t)integrity);
  if (in_len < WL_FRAME_BASE_HEADER_SIZE + crc_len) {
    return WL_ERR_BAD_FRAME;
  }

  hdr.marker_version_kind = in[0];
  hdr.reserved = in[1];
  if ((in[0] & WL_FRAME_MARKER_MASK) != WL_FRAME_MARKER) {
    return WL_ERR_BAD_FRAME;
  }
  hdr.version = (in[0] & WL_FRAME_VERSION_MASK) >> WL_FRAME_VERSION_SHIFT;
  if (hdr.version != WL_FRAME_VERSION) {
    return WL_ERR_PROTOCOL_VERSION;
  }
  kind = in[0] & WL_FRAME_KIND_MASK;
  switch (kind) {
  case 0U:
    hdr.packet_type = WL_PACKET_DATA;
    hdr.flags = 0U;
    header_len = WL_FRAME_BASE_HEADER_SIZE;
    break;
  case 1U:
    hdr.packet_type = WL_PACKET_DATA;
    hdr.flags = WL_PACKET_FLAG_RELIABLE;
    header_len = WL_FRAME_RELIABLE_HEADER_SIZE;
    break;
  case 2U:
    hdr.packet_type = WL_PACKET_ACK;
    hdr.flags = 0U;
    header_len = WL_FRAME_RELIABLE_HEADER_SIZE;
    break;
  default:
    return WL_ERR_NOT_SUPPORTED;
  }
  if (in_len < header_len + crc_len) {
    return WL_ERR_BAD_FRAME;
  }
  payload_len = in_len - header_len - crc_len;
  if (payload_len > WL_PACKET_MAX_PAYLOAD) {
    return WL_ERR_FRAME_TOO_LONG;
  }
  hdr.header_length = (uint8_t)header_len;
  hdr.message_id = read_u16_be(in + 2U);
  hdr.payload_length = (uint16_t)payload_len;
  if (header_len == WL_FRAME_RELIABLE_HEADER_SIZE) {
    hdr.session_id = read_u64_be(in + 4U);
    hdr.sequence = read_u32_be(in + 12U);
  }

  wl_err_t e = wl_frame_validate_header(&hdr);
  if (e != WL_OK) {
    return e;
  }

  if (crc_len == 2) {
    expected_crc16 = read_u16_be(in + header_len + payload_len);
    check_crc16 = wl_crc16_ccitt_false(in, header_len + payload_len);
    if (expected_crc16 != check_crc16) {
      return WL_ERR_CRC;
    }
  } else if (crc_len == 4) {
    expected_crc32 = read_u32_be(in + header_len + payload_len);
    check_crc32 = wl_crc32c(in, header_len + payload_len);
    if (expected_crc32 != check_crc32) {
      return WL_ERR_CRC;
    }
  }

  out_view->type = (wl_packet_type_t)hdr.packet_type;
  out_view->flags = hdr.flags;
  out_view->message_id = hdr.message_id;
  out_view->session_id = hdr.session_id;
  out_view->sequence = hdr.sequence;
  out_view->integrity = integrity;
  out_view->payload.data = (uint8_t *)(in + header_len);
  out_view->payload.length = hdr.payload_length;
  out_view->integrity_bytes.data =
      (crc_len != 0U) ? (uint8_t *)(in + header_len + payload_len)
                     : NULL;
  out_view->integrity_bytes.length = crc_len;
  out_view->total_len = in_len;

  return WL_OK;
}
