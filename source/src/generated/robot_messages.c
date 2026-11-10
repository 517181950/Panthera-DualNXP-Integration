#include "robot_messages.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

enum {
  WLC_OPTIONAL,
  WLC_REPEATED,
  WLC_PACKED,
  WLC_BOOL,
  WLC_U8,
  WLC_U16,
  WLC_U32,
  WLC_U64,
  WLC_I8,
  WLC_I16,
  WLC_I32,
  WLC_I64,
  WLC_F32,
  WLC_F64,
  WLC_FLOAT32,
  WLC_FLOAT64,
  WLC_BYTES,
  WLC_STRING,
  WLC_ENUM,
  WLC_MESSAGE
};
typedef struct wlc_desc wlc_desc_t;
typedef struct {
  uint16_t number;
  uint8_t card, kind, required, wire, key_size;
  size_t value, has, count, capacity, element, packed_count;
  uint16_t max_length;
  uint8_t key[3];
  int64_t signed_default;
  uint64_t unsigned_default;
  const char *string_default;
  const wlc_desc_t *nested;
} wlc_field_t;
enum { WLC_LOOKUP_LINEAR, WLC_LOOKUP_DENSE, WLC_LOOKUP_BINARY };
struct wlc_desc { const wlc_field_t *fields; size_t count; uint8_t lookup; };

static inline wl_codec_status_t wlc_add(size_t *a, size_t b) {
  if (b > SIZE_MAX - *a) return WL_CODEC_ERR_OVERFLOW;
  *a += b;
  return WL_CODEC_OK;
}
static inline size_t wlc_vsize(uint64_t v) {
  size_t n = 1U;
  while (v >= 128U) { v >>= 7U; ++n; }
  return n;
}
static inline void wlc_putv(uint8_t **p, uint64_t v) {
  while (v >= 128U) { *(*p)++ = (uint8_t)(v | 128U); v >>= 7U; }
  *(*p)++ = (uint8_t)v;
}
static wl_codec_status_t wlc_getv(const uint8_t *in, size_t length, size_t *at,
                                  uint64_t *out) {
  size_t start = *at, n = 0U;
  uint64_t v = 0U;
  while (*at < length && n < 10U) {
    uint8_t b = in[(*at)++];
    if (n == 9U && b > 1U) return WL_CODEC_ERR_OVERFLOW;
    v |= (uint64_t)(b & 127U) << (7U * n++);
    if ((b & 128U) == 0U) {
      if (wlc_vsize(v) != *at - start) return WL_CODEC_ERR_MALFORMED;
      *out = v;
      return WL_CODEC_OK;
    }
  }
  return *at == length ? WL_CODEC_ERR_MALFORMED : WL_CODEC_ERR_OVERFLOW;
}
static bool wlc_utf8(const uint8_t *s, size_t n) {
  size_t i = 0U;
  while (i < n) {
    uint8_t a = s[i++];
    if (a < 0x80U) continue;
    size_t need;
    uint32_t v;
    if (a >= 0xC2U && a <= 0xDFU) { need = 1U; v = a & 0x1FU; }
    else if (a >= 0xE0U && a <= 0xEFU) { need = 2U; v = a & 0x0FU; }
    else if (a >= 0xF0U && a <= 0xF4U) { need = 3U; v = a & 0x07U; }
    else return false;
    if (need > n - i) return false;
    while (need-- != 0U) {
      uint8_t b = s[i++];
      if ((b & 0xC0U) != 0x80U) return false;
      v = (v << 6U) | (b & 0x3FU);
    }
    if ((a == 0xE0U && v < 0x800U) ||
        (a == 0xEDU && v >= 0xD800U) ||
        (a == 0xF0U && v < 0x10000U) ||
        (a == 0xF4U && v > 0x10FFFFU)) return false;
  }
  return true;
}
static inline uint8_t wlc_wire(const wlc_field_t *f) {
  return f->wire;
}
static inline uint64_t wlc_z32(int32_t v) {
  return ((uint32_t)v << 1U) ^ (uint32_t)-(uint32_t)(v < 0);
}
static inline uint64_t wlc_z64(int64_t v) {
  return ((uint64_t)v << 1U) ^ (uint64_t)-(uint64_t)(v < 0);
}
static inline int32_t wlc_uz32(uint32_t v) {
  return (int32_t)((v >> 1U) ^ (uint32_t)-(v & 1U));
}
static inline int64_t wlc_uz64(uint64_t v) {
  return (int64_t)((v >> 1U) ^ (uint64_t)-(v & 1U));
}
static wl_codec_status_t wlc_measure(const wlc_desc_t *, const void *, size_t *);
static wl_codec_status_t wlc_measure_impl(const wlc_desc_t *, const void *, size_t *, bool);
static void wlc_clear(const wlc_desc_t *, void *);
static wl_codec_status_t wlc_decode(const wlc_desc_t *, const uint8_t *, size_t,
                                    void *);
static wl_codec_status_t wlc_emit_fields(const wlc_desc_t *, const void *,
                                         uint8_t **);

static wl_codec_status_t wlc_packed_bytes(const wlc_field_t *f, size_t *bytes) {
  if (f->packed_count != 0U && f->element > SIZE_MAX / f->packed_count)
    return WL_CODEC_ERR_OVERFLOW;
  *bytes = f->element * f->packed_count;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_body(const wlc_field_t *f, const void *p,
                                  size_t *n, bool validated) {
  *n = 0U;
  switch (f->kind) {
    case WLC_BOOL: {
      bool v = *(const bool *)p;
      if (v != false && v != true) return WL_CODEC_ERR_INVALID_VALUE;
      *n = 1U;
      return WL_CODEC_OK;
    }
    case WLC_U8: *n = wlc_vsize(*(const uint8_t *)p); return WL_CODEC_OK;
    case WLC_U16: *n = wlc_vsize(*(const uint16_t *)p); return WL_CODEC_OK;
    case WLC_U32: *n = wlc_vsize(*(const uint32_t *)p); return WL_CODEC_OK;
    case WLC_U64: *n = wlc_vsize(*(const uint64_t *)p); return WL_CODEC_OK;
    case WLC_I8: *n = wlc_vsize(wlc_z32(*(const int8_t *)p)); return WL_CODEC_OK;
    case WLC_I16: *n = wlc_vsize(wlc_z32(*(const int16_t *)p)); return WL_CODEC_OK;
    case WLC_I32:
    case WLC_ENUM: *n = wlc_vsize(wlc_z32(*(const int32_t *)p)); return WL_CODEC_OK;
    case WLC_I64: *n = wlc_vsize(wlc_z64(*(const int64_t *)p)); return WL_CODEC_OK;
    case WLC_F32:
    case WLC_FLOAT32: *n = 4U; return WL_CODEC_OK;
    case WLC_F64:
    case WLC_FLOAT64: *n = 8U; return WL_CODEC_OK;
    case WLC_BYTES: {
      const wl_codec_bytes_t *v = p;
      if (f->max_length != 0U && v->length > (size_t)f->max_length)
        return WL_CODEC_ERR_INVALID_VALUE;
      if (v->length != 0U && v->data == NULL) return WL_CODEC_ERR_INVALID_VALUE;
      if (wlc_add(n, wlc_vsize(v->length)) != WL_CODEC_OK)
        return WL_CODEC_ERR_OVERFLOW;
      return wlc_add(n, v->length);
    }
    case WLC_STRING: {
      const wl_codec_string_t *v = p;
      if (f->max_length != 0U && v->length > (size_t)f->max_length)
        return WL_CODEC_ERR_INVALID_VALUE;
      if (v->length != 0U && v->data == NULL) return WL_CODEC_ERR_INVALID_VALUE;
      if (!validated && !wlc_utf8((const uint8_t *)v->data, v->length)) return WL_CODEC_ERR_UTF8;
      if (wlc_add(n, wlc_vsize(v->length)) != WL_CODEC_OK)
        return WL_CODEC_ERR_OVERFLOW;
      return wlc_add(n, v->length);
    }
    case WLC_MESSAGE: {
      size_t child;
      wl_codec_status_t s = wlc_measure_impl(f->nested, p, &child, validated);
      if (s != WL_CODEC_OK) return s;
      *n = wlc_vsize(child);
      return wlc_add(n, child);
    }
    default: return WL_CODEC_ERR_INVALID_VALUE;
  }
}
static wl_codec_status_t wlc_measure_impl(const wlc_desc_t *d, const void *value,
                                     size_t *out, bool validated) {
  size_t n = 0U;
  if (d == NULL || value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  for (size_t i = 0U; i < d->count; ++i) {
    const wlc_field_t *f = &d->fields[i];
    const uint8_t *base = value;
    if (f->card == WLC_PACKED) {
      size_t bytes;
      wl_codec_status_t s;
      if (!*(const bool *)(base + f->has)) {
        if (f->required != 0U) return WL_CODEC_ERR_MISSING_REQUIRED_FIELD;
        continue;
      }
      if ((s = wlc_packed_bytes(f, &bytes)) != WL_CODEC_OK) return s;
      if ((s = wlc_add(&n, f->key_size)) != WL_CODEC_OK ||
          (s = wlc_add(&n, wlc_vsize(bytes))) != WL_CODEC_OK ||
          (s = wlc_add(&n, bytes)) != WL_CODEC_OK) return s;
      continue;
    }
    size_t count = 1U;
    if (f->card == WLC_OPTIONAL) {
      if (!*(const bool *)(base + f->has)) {
        if (f->required != 0U) return WL_CODEC_ERR_MISSING_REQUIRED_FIELD;
        continue;
      }
    } else {
      count = *(const size_t *)(base + f->count);
      if ((count != 0U && *(void *const *)(base + f->value) == NULL) ||
          count > *(const size_t *)(base + f->capacity))
        return WL_CODEC_ERR_INVALID_VALUE;
    }
    for (size_t j = 0U; j < count; ++j) {
      size_t body;
      const void *p = f->card == WLC_REPEATED
                          ? *(const uint8_t *const *)(base + f->value) + j * f->element
                          : base + f->value;
      wl_codec_status_t s = wlc_body(f, p, &body, validated);
      if (s != WL_CODEC_OK) return s;
      if ((s = wlc_add(&n, f->key_size)) != WL_CODEC_OK ||
          (s = wlc_add(&n, body)) != WL_CODEC_OK) return s;
    }
  }
  *out = n;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_measure(const wlc_desc_t *d, const void *value, size_t *out) {
  return wlc_measure_impl(d, value, out, false);
}
static wl_codec_status_t wlc_measure_validated(const wlc_desc_t *d, const void *value, size_t *out) {
  return wlc_measure_impl(d, value, out, true);
}
static void wlc_clear(const wlc_desc_t *d, void *value) {
  uint8_t *base = value;
  for (size_t i = 0U; i < d->count; ++i) {
    const wlc_field_t *f = &d->fields[i];
    void *p = base + f->value;
    if (f->card == WLC_REPEATED) {
      *(size_t *)(base + f->count) = 0U;
      continue;
    }
    *(bool *)(base + f->has) = false;
    if (f->card == WLC_PACKED) {
      memset(p, 0, f->element * f->packed_count);
      continue;
    }
    if (f->kind == WLC_MESSAGE) { wlc_clear(f->nested, p); continue; }
    if (f->kind == WLC_STRING) {
      *(wl_codec_string_t *)p =
          (wl_codec_string_t){f->string_default, (size_t)f->unsigned_default};
      continue;
    }
    if (f->kind == WLC_BYTES) {
      *(wl_codec_bytes_t *)p = (wl_codec_bytes_t){NULL, 0U};
      continue;
    }
    if (f->kind == WLC_FLOAT32 || f->kind == WLC_FLOAT64) {
      memset(p, 0, f->element);
      continue;
    }
    if (f->kind == WLC_BOOL) *(bool *)p = f->unsigned_default != 0U;
    else if (f->kind == WLC_I8) *(int8_t *)p = (int8_t)f->signed_default;
    else if (f->kind == WLC_I16) *(int16_t *)p = (int16_t)f->signed_default;
    else if (f->kind == WLC_I32 || f->kind == WLC_ENUM)
      *(int32_t *)p = (int32_t)f->signed_default;
    else if (f->kind == WLC_I64) *(int64_t *)p = f->signed_default;
    else if (f->kind == WLC_U8) *(uint8_t *)p = (uint8_t)f->unsigned_default;
    else if (f->kind == WLC_U16) *(uint16_t *)p = (uint16_t)f->unsigned_default;
    else if (f->kind == WLC_U32 || f->kind == WLC_F32)
      *(uint32_t *)p = (uint32_t)f->unsigned_default;
    else if (f->kind == WLC_U64 || f->kind == WLC_F64)
      *(uint64_t *)p = f->unsigned_default;
  }
}
static inline void wlc_put32(uint8_t **p, uint32_t v) {
  *(*p)++ = (uint8_t)(v >> 24U);
  *(*p)++ = (uint8_t)(v >> 16U);
  *(*p)++ = (uint8_t)(v >> 8U);
  *(*p)++ = (uint8_t)v;
}
static inline void wlc_put64(uint8_t **p, uint64_t v) {
  wlc_put32(p, (uint32_t)(v >> 32U));
  wlc_put32(p, (uint32_t)v);
}
static inline void wlc_copy_span(uint8_t **out, const void *data, size_t length) {
  if (length != 0U) { memcpy(*out, data, length); *out += length; }
}
static inline void wlc_put_key(uint8_t **out, const wlc_field_t *f) {
  *(*out)++ = f->key[0];
  if (f->key_size > 1U) {
    *(*out)++ = f->key[1];
    if (f->key_size > 2U) *(*out)++ = f->key[2];
  }
}

static wl_codec_status_t wlc_emit_fixed(uint8_t kind, const void *value,
                                        uint8_t **out) {
  if (kind == WLC_F32) wlc_put32(out, *(const uint32_t *)value);
  else if (kind == WLC_F64) wlc_put64(out, *(const uint64_t *)value);
  else if (kind == WLC_FLOAT32) {
    uint32_t bits32;
    memcpy(&bits32, value, sizeof(bits32));
    wlc_put32(out, bits32);
  } else if (kind == WLC_FLOAT64) {
    uint64_t bits;
    memcpy(&bits, value, sizeof(bits));
    wlc_put64(out, bits);
  } else return WL_CODEC_ERR_INVALID_VALUE;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_emit_value(const wlc_field_t *f, const void *p,
                                        uint8_t **out) {
  switch (f->kind) {
    case WLC_BOOL: wlc_putv(out, *(const bool *)p); return WL_CODEC_OK;
    case WLC_U8: wlc_putv(out, *(const uint8_t *)p); return WL_CODEC_OK;
    case WLC_U16: wlc_putv(out, *(const uint16_t *)p); return WL_CODEC_OK;
    case WLC_U32: wlc_putv(out, *(const uint32_t *)p); return WL_CODEC_OK;
    case WLC_U64: wlc_putv(out, *(const uint64_t *)p); return WL_CODEC_OK;
    case WLC_I8: wlc_putv(out, wlc_z32(*(const int8_t *)p)); return WL_CODEC_OK;
    case WLC_I16: wlc_putv(out, wlc_z32(*(const int16_t *)p)); return WL_CODEC_OK;
    case WLC_I32:
    case WLC_ENUM: wlc_putv(out, wlc_z32(*(const int32_t *)p)); return WL_CODEC_OK;
    case WLC_I64: wlc_putv(out, wlc_z64(*(const int64_t *)p)); return WL_CODEC_OK;
    case WLC_F32:
    case WLC_F64:
    case WLC_FLOAT32:
    case WLC_FLOAT64: return wlc_emit_fixed(f->kind, p, out);
    case WLC_BYTES: {
      const wl_codec_bytes_t *v = p;
      wlc_putv(out, v->length);
      wlc_copy_span(out, v->data, v->length);
      return WL_CODEC_OK;
    }
    case WLC_STRING: {
      const wl_codec_string_t *v = p;
      wlc_putv(out, v->length);
      wlc_copy_span(out, v->data, v->length);
      return WL_CODEC_OK;
    }
    case WLC_MESSAGE: {
      size_t child;
      wl_codec_status_t s = wlc_measure(f->nested, p, &child);
      if (s != WL_CODEC_OK) return s;
      wlc_putv(out, child);
      return wlc_emit_fields(f->nested, p, out);
    }
    default: return WL_CODEC_ERR_INVALID_VALUE;
  }
}
static wl_codec_status_t wlc_emit_packed(const wlc_field_t *f, const void *p,
                                         uint8_t **out) {
  size_t bytes;
  wl_codec_status_t s = wlc_packed_bytes(f, &bytes);
  if (s != WL_CODEC_OK) return s;
  wlc_putv(out, bytes);
  if (f->kind == WLC_F32 || f->kind == WLC_FLOAT32) {
    for (size_t j = 0U; j < f->packed_count; ++j) {
      uint32_t bits;
      memcpy(&bits, (const uint8_t *)p + j * f->element, sizeof(bits));
      wlc_put32(out, bits);
    }
  } else if (f->kind == WLC_F64 || f->kind == WLC_FLOAT64) {
    for (size_t j = 0U; j < f->packed_count; ++j) {
      uint64_t bits;
      memcpy(&bits, (const uint8_t *)p + j * f->element, sizeof(bits));
      wlc_put64(out, bits);
    }
  } else {
    return WL_CODEC_ERR_INVALID_VALUE;
  }
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_emit_fields(const wlc_desc_t *d, const void *value,
                                         uint8_t **out) {
  for (size_t i = 0U; i < d->count; ++i) {
    const wlc_field_t *f = &d->fields[i];
    const uint8_t *base = value;
    if (f->card == WLC_PACKED) {
      wl_codec_status_t s;
      if (!*(const bool *)(base + f->has)) continue;
      wlc_put_key(out, f);
      if ((s = wlc_emit_packed(f, base + f->value, out)) != WL_CODEC_OK) return s;
      continue;
    }
    size_t count = f->card == WLC_OPTIONAL
                       ? (*(const bool *)(base + f->has) ? 1U : 0U)
                       : *(const size_t *)(base + f->count);
    for (size_t j = 0U; j < count; ++j) {
      const void *p = f->card == WLC_REPEATED
                          ? *(const uint8_t *const *)(base + f->value) + j * f->element
                          : base + f->value;
      wl_codec_status_t s;
      wlc_put_key(out, f);
      if ((s = wlc_emit_value(f, p, out)) != WL_CODEC_OK) return s;
    }
  }
  return WL_CODEC_OK;
}

static inline wl_codec_status_t wlc_encode(const wlc_desc_t *d, const void *value,
                                    uint8_t *out, size_t cap, size_t *length) {
  size_t n;
  wl_codec_status_t s = wlc_measure(d, value, &n);
  if (s != WL_CODEC_OK || length == NULL || (n != 0U && out == NULL))
    return s == WL_CODEC_OK ? WL_CODEC_ERR_INVALID_VALUE : s;
  if (cap < n) return WL_CODEC_ERR_CAPACITY;
  uint8_t *p = out;
  if ((s = wlc_emit_fields(d, value, &p)) != WL_CODEC_OK) return s;
  *length = n;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_skip(uint8_t wire, const uint8_t *in, size_t n,
                                  size_t *at) {
  uint64_t length;
  wl_codec_status_t s;
  if (wire == 0U) return wlc_getv(in, n, at, &length);
  if (wire == 1U) {
    if (n - *at < 8U) return WL_CODEC_ERR_MALFORMED;
    *at += 8U;
    return WL_CODEC_OK;
  }
  if (wire == 5U) {
    if (n - *at < 4U) return WL_CODEC_ERR_MALFORMED;
    *at += 4U;
    return WL_CODEC_OK;
  }
  if (wire != 2U) return WL_CODEC_ERR_MALFORMED;
  if ((s = wlc_getv(in, n, at, &length)) != WL_CODEC_OK) return s;
  if (length > n - *at) return WL_CODEC_ERR_MALFORMED;
  *at += (size_t)length;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_read_fixed(uint8_t kind, const uint8_t *in,
                                        size_t n, size_t *at, void *out) {
  size_t bytes = (kind == WLC_F32 || kind == WLC_FLOAT32) ? 4U : 8U;
  if (n - *at < bytes) return WL_CODEC_ERR_MALFORMED;
  uint64_t bits = 0U;
  for (size_t i = 0U; i < bytes; ++i) bits = (bits << 8U) | in[(*at)++];
  if (kind == WLC_F32) *(uint32_t *)out = (uint32_t)bits;
  else if (kind == WLC_F64) *(uint64_t *)out = bits;
  else if (kind == WLC_FLOAT32) {
    uint32_t bits32 = (uint32_t)bits;
    memcpy(out, &bits32, sizeof(bits32));
  } else if (kind == WLC_FLOAT64) memcpy(out, &bits, sizeof(bits));
  else return WL_CODEC_ERR_INVALID_VALUE;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_read_value(const wlc_field_t *f, const uint8_t *in,
                                        size_t n, size_t *at, void *out) {
  uint64_t v;
  wl_codec_status_t s;
  if (f->kind == WLC_F32 || f->kind == WLC_F64 ||
      f->kind == WLC_FLOAT32 || f->kind == WLC_FLOAT64)
    return wlc_read_fixed(f->kind, in, n, at, out);
  if (f->kind == WLC_BYTES || f->kind == WLC_STRING || f->kind == WLC_MESSAGE) {
    if ((s = wlc_getv(in, n, at, &v)) != WL_CODEC_OK) return s;
    if ((f->kind == WLC_BYTES || f->kind == WLC_STRING) &&
        f->max_length != 0U && v > (uint64_t)f->max_length)
      return WL_CODEC_ERR_INVALID_VALUE;
    if (v > n - *at) return WL_CODEC_ERR_MALFORMED;
    size_t bytes = (size_t)v;
    if (f->kind == WLC_BYTES)
      *(wl_codec_bytes_t *)out = (wl_codec_bytes_t){in + *at, bytes};
    else if (f->kind == WLC_STRING) {
      if (!wlc_utf8(in + *at, bytes)) return WL_CODEC_ERR_UTF8;
      *(wl_codec_string_t *)out =
          (wl_codec_string_t){(const char *)(in + *at), bytes};
    } else {
      s = wlc_decode(f->nested, in + *at, bytes, out);
      if (s != WL_CODEC_OK) return s;
    }
    *at += bytes;
    return WL_CODEC_OK;
  }
  if ((s = wlc_getv(in, n, at, &v)) != WL_CODEC_OK) return s;
  if (f->kind == WLC_BOOL) {
    if (v > 1U) return WL_CODEC_ERR_INVALID_VALUE;
    *(bool *)out = v != 0U;
  } else if (f->kind == WLC_U8) {
    if (v > UINT8_MAX) return WL_CODEC_ERR_OVERFLOW;
    *(uint8_t *)out = (uint8_t)v;
  } else if (f->kind == WLC_U16) {
    if (v > UINT16_MAX) return WL_CODEC_ERR_OVERFLOW;
    *(uint16_t *)out = (uint16_t)v;
  } else if (f->kind == WLC_U32) {
    if (v > UINT32_MAX) return WL_CODEC_ERR_OVERFLOW;
    *(uint32_t *)out = (uint32_t)v;
  } else if (f->kind == WLC_U64) *(uint64_t *)out = v;
  else if (f->kind == WLC_I8) {
    if (v > UINT8_MAX) return WL_CODEC_ERR_OVERFLOW;
    *(int8_t *)out = (int8_t)wlc_uz32((uint32_t)v);
  } else if (f->kind == WLC_I16) {
    if (v > UINT16_MAX) return WL_CODEC_ERR_OVERFLOW;
    *(int16_t *)out = (int16_t)wlc_uz32((uint32_t)v);
  } else if (f->kind == WLC_I32 || f->kind == WLC_ENUM) {
    if (v > UINT32_MAX) return WL_CODEC_ERR_OVERFLOW;
    *(int32_t *)out = wlc_uz32((uint32_t)v);
  } else if (f->kind == WLC_I64) *(int64_t *)out = wlc_uz64(v);
  else return WL_CODEC_ERR_INVALID_VALUE;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_read_packed(const wlc_field_t *f,
                                         const uint8_t *in, size_t n,
                                         size_t *at, void *out) {
  uint64_t encoded_bytes;
  size_t expected_bytes;
  wl_codec_status_t s;
  if ((s = wlc_getv(in, n, at, &encoded_bytes)) != WL_CODEC_OK) return s;
  if ((s = wlc_packed_bytes(f, &expected_bytes)) != WL_CODEC_OK) return s;
  if (encoded_bytes != expected_bytes || expected_bytes > n - *at)
    return WL_CODEC_ERR_MALFORMED;
  if (f->kind == WLC_F32 || f->kind == WLC_FLOAT32) {
    for (size_t j = 0U; j < f->packed_count; ++j) {
      uint32_t bits = ((uint32_t)in[*at] << 24U) |
                      ((uint32_t)in[*at + 1U] << 16U) |
                      ((uint32_t)in[*at + 2U] << 8U) |
                      (uint32_t)in[*at + 3U];
      memcpy((uint8_t *)out + j * f->element, &bits, sizeof(bits));
      *at += 4U;
    }
  } else if (f->kind == WLC_F64 || f->kind == WLC_FLOAT64) {
    for (size_t j = 0U; j < f->packed_count; ++j) {
      uint64_t bits = 0U;
      for (size_t i = 0U; i < 8U; ++i) bits = (bits << 8U) | in[(*at)++];
      memcpy((uint8_t *)out + j * f->element, &bits, sizeof(bits));
    }
  } else {
    return WL_CODEC_ERR_INVALID_VALUE;
  }
  return WL_CODEC_OK;
}
static const wlc_field_t *wlc_find_field(const wlc_desc_t *d, uint16_t number) {
  if (d->lookup == WLC_LOOKUP_DENSE) {
    size_t index = (size_t)number - (size_t)d->fields[0].number;
    return index < d->count ? &d->fields[index] : NULL;
  }
  if (d->lookup == WLC_LOOKUP_BINARY) {
    size_t lo = 0U, hi = d->count;
    while (lo < hi) {
      size_t mid = lo + (hi - lo) / 2U;
      uint16_t candidate = d->fields[mid].number;
      if (candidate == number) return &d->fields[mid];
      if (candidate < number) lo = mid + 1U;
      else hi = mid;
    }
    return NULL;
  }
  for (size_t i = 0U; i < d->count; ++i)
    if (d->fields[i].number == number) return &d->fields[i];
  return NULL;
}
static wl_codec_status_t wlc_decode(const wlc_desc_t *d, const uint8_t *in,
                                    size_t n, void *out) {
  if (d == NULL || out == NULL || (n != 0U && in == NULL))
    return WL_CODEC_ERR_INVALID_VALUE;
  wlc_clear(d, out);
  // A hint only: missing, unknown or reordered fields still use exact lookup.
  // Each recursive decode owns its cursor; empty schemas avoid NULL arithmetic.
  const wlc_field_t *next = d->fields;
  const wlc_field_t *end = d->count == 0U ? next : next + d->count;
  for (size_t at = 0U; at < n;) {
    uint64_t key;
    wl_codec_status_t s = wlc_getv(in, n, &at, &key);
    if (s != WL_CODEC_OK) return s;
    uint64_t raw_number = key >> 3U;
    uint8_t wire = (uint8_t)(key & 7U);
    if (raw_number == 0U || raw_number > 65535U ||
        (wire != 0U && wire != 1U && wire != 2U && wire != 5U))
      return WL_CODEC_ERR_MALFORMED;
    uint16_t number = (uint16_t)raw_number;
    const wlc_field_t *f = next != end && next->number == number
                               ? next : wlc_find_field(d, number);
    if (f == NULL) {
      if ((s = wlc_skip(wire, in, n, &at)) != WL_CODEC_OK) return s;
      continue;
    }
    if (wire != wlc_wire(f)) return WL_CODEC_ERR_WIRE_TYPE;
    next = f->card == WLC_REPEATED ? f : f + 1;
    uint8_t *base = out;
    if (f->card == WLC_PACKED) {
      if (*(bool *)(base + f->has)) return WL_CODEC_ERR_DUPLICATE_FIELD;
      if ((s = wlc_read_packed(f, in, n, &at, base + f->value)) != WL_CODEC_OK)
        return s;
      *(bool *)(base + f->has) = true;
      continue;
    }
    void *p;
    if (f->card == WLC_OPTIONAL) {
      if (*(bool *)(base + f->has)) return WL_CODEC_ERR_DUPLICATE_FIELD;
      p = base + f->value;
    } else {
      size_t count = *(size_t *)(base + f->count);
      if (count >= *(size_t *)(base + f->capacity)) return WL_CODEC_ERR_CAPACITY;
      void *storage = *(void **)(base + f->value);
      if (storage == NULL) return WL_CODEC_ERR_INVALID_VALUE;
      p = (uint8_t *)storage + count * f->element;
    }
    if ((s = wlc_read_value(f, in, n, &at, p)) != WL_CODEC_OK) return s;
    if (f->card == WLC_OPTIONAL) *(bool *)(base + f->has) = true;
    else ++*(size_t *)(base + f->count);
  }
  for (size_t i = 0U; i < d->count; ++i) {
    const wlc_field_t *f = &d->fields[i];
    if (f->required != 0U && !*(const bool *)((uint8_t *)out + f->has))
      return WL_CODEC_ERR_MISSING_REQUIRED_FIELD;
  }
  return WL_CODEC_OK;
}
/* Generator-private canonical sink. Only successful decode results enter here.
 * The shared emitter orders known fields, drops unknowns, and preserves presence.
 * Scalar bytes use the ordinary endian/varint writers; no raw-frame hashing. */
typedef struct {
  uint64_t hash;
  size_t length;
  wl_codec_status_t status;
} wlc_hash_state_t;
static wl_codec_status_t wlc_hash_emit_fields(const wlc_desc_t *, const void *, wlc_hash_state_t *);
static inline void wlc_hash_copy_span(wlc_hash_state_t *out, const void *data, size_t length) {
  const uint8_t *bytes = data;
  if (out->status != WL_CODEC_OK) return;
  out->status = wlc_add(&out->length, length);
  if (out->status != WL_CODEC_OK) return;
  uint64_t hash = out->hash;
  for (size_t i = 0U; i < length; ++i)
    hash = (hash ^ bytes[i]) * UINT64_C(0x100000001b3);
  out->hash = hash;
}
static inline void wlc_hash_putv(wlc_hash_state_t *out, uint64_t value) {
  uint8_t bytes[10], *cursor = bytes;
  wlc_putv(&cursor, value);
  wlc_hash_copy_span(out, bytes, (size_t)(cursor - bytes));
}
static inline void wlc_hash_put_key(wlc_hash_state_t *out, const wlc_field_t *f) {
  wlc_hash_copy_span(out, f->key, f->key_size);
}
static inline void wlc_hash_put32(wlc_hash_state_t *out, uint32_t value) {
  uint8_t bytes[4], *cursor = bytes;
  wlc_put32(&cursor, value);
  wlc_hash_copy_span(out, bytes, sizeof(bytes));
}
static inline void wlc_hash_put64(wlc_hash_state_t *out, uint64_t value) {
  uint8_t bytes[8], *cursor = bytes;
  wlc_put64(&cursor, value);
  wlc_hash_copy_span(out, bytes, sizeof(bytes));
}

static wl_codec_status_t wlc_hash_emit_fixed(uint8_t kind, const void *value,
                                        wlc_hash_state_t *out) {
  if (kind == WLC_F32) wlc_hash_put32(out, *(const uint32_t *)value);
  else if (kind == WLC_F64) wlc_hash_put64(out, *(const uint64_t *)value);
  else if (kind == WLC_FLOAT32) {
    uint32_t bits32;
    memcpy(&bits32, value, sizeof(bits32));
    wlc_hash_put32(out, bits32);
  } else if (kind == WLC_FLOAT64) {
    uint64_t bits;
    memcpy(&bits, value, sizeof(bits));
    wlc_hash_put64(out, bits);
  } else return WL_CODEC_ERR_INVALID_VALUE;
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_hash_emit_value(const wlc_field_t *f, const void *p,
                                        wlc_hash_state_t *out) {
  switch (f->kind) {
    case WLC_BOOL: wlc_hash_putv(out, *(const bool *)p); return WL_CODEC_OK;
    case WLC_U8: wlc_hash_putv(out, *(const uint8_t *)p); return WL_CODEC_OK;
    case WLC_U16: wlc_hash_putv(out, *(const uint16_t *)p); return WL_CODEC_OK;
    case WLC_U32: wlc_hash_putv(out, *(const uint32_t *)p); return WL_CODEC_OK;
    case WLC_U64: wlc_hash_putv(out, *(const uint64_t *)p); return WL_CODEC_OK;
    case WLC_I8: wlc_hash_putv(out, wlc_z32(*(const int8_t *)p)); return WL_CODEC_OK;
    case WLC_I16: wlc_hash_putv(out, wlc_z32(*(const int16_t *)p)); return WL_CODEC_OK;
    case WLC_I32:
    case WLC_ENUM: wlc_hash_putv(out, wlc_z32(*(const int32_t *)p)); return WL_CODEC_OK;
    case WLC_I64: wlc_hash_putv(out, wlc_z64(*(const int64_t *)p)); return WL_CODEC_OK;
    case WLC_F32:
    case WLC_F64:
    case WLC_FLOAT32:
    case WLC_FLOAT64: return wlc_hash_emit_fixed(f->kind, p, out);
    case WLC_BYTES: {
      const wl_codec_bytes_t *v = p;
      wlc_hash_putv(out, v->length);
      wlc_hash_copy_span(out, v->data, v->length);
      return WL_CODEC_OK;
    }
    case WLC_STRING: {
      const wl_codec_string_t *v = p;
      wlc_hash_putv(out, v->length);
      wlc_hash_copy_span(out, v->data, v->length);
      return WL_CODEC_OK;
    }
    case WLC_MESSAGE: {
      size_t child;
      wl_codec_status_t s = wlc_measure_validated(f->nested, p, &child);
      if (s != WL_CODEC_OK) return s;
      wlc_hash_putv(out, child);
      return wlc_hash_emit_fields(f->nested, p, out);
    }
    default: return WL_CODEC_ERR_INVALID_VALUE;
  }
}
static wl_codec_status_t wlc_hash_emit_packed(const wlc_field_t *f, const void *p,
                                         wlc_hash_state_t *out) {
  size_t bytes;
  wl_codec_status_t s = wlc_packed_bytes(f, &bytes);
  if (s != WL_CODEC_OK) return s;
  wlc_hash_putv(out, bytes);
  if (f->kind == WLC_F32 || f->kind == WLC_FLOAT32) {
    for (size_t j = 0U; j < f->packed_count; ++j) {
      uint32_t bits;
      memcpy(&bits, (const uint8_t *)p + j * f->element, sizeof(bits));
      wlc_hash_put32(out, bits);
    }
  } else if (f->kind == WLC_F64 || f->kind == WLC_FLOAT64) {
    for (size_t j = 0U; j < f->packed_count; ++j) {
      uint64_t bits;
      memcpy(&bits, (const uint8_t *)p + j * f->element, sizeof(bits));
      wlc_hash_put64(out, bits);
    }
  } else {
    return WL_CODEC_ERR_INVALID_VALUE;
  }
  return WL_CODEC_OK;
}
static wl_codec_status_t wlc_hash_emit_fields(const wlc_desc_t *d, const void *value,
                                         wlc_hash_state_t *out) {
  for (size_t i = 0U; i < d->count; ++i) {
    const wlc_field_t *f = &d->fields[i];
    const uint8_t *base = value;
    if (f->card == WLC_PACKED) {
      wl_codec_status_t s;
      if (!*(const bool *)(base + f->has)) continue;
      wlc_hash_put_key(out, f);
      if ((s = wlc_hash_emit_packed(f, base + f->value, out)) != WL_CODEC_OK) return s;
      continue;
    }
    size_t count = f->card == WLC_OPTIONAL
                       ? (*(const bool *)(base + f->has) ? 1U : 0U)
                       : *(const size_t *)(base + f->count);
    for (size_t j = 0U; j < count; ++j) {
      const void *p = f->card == WLC_REPEATED
                          ? *(const uint8_t *const *)(base + f->value) + j * f->element
                          : base + f->value;
      wl_codec_status_t s;
      wlc_hash_put_key(out, f);
      if ((s = wlc_hash_emit_value(f, p, out)) != WL_CODEC_OK) return s;
    }
  }
  return WL_CODEC_OK;
}
static const wlc_desc_t stream_header_desc;
static const wlc_desc_t arm_command_desc;
static const wlc_desc_t hand_command_desc;
static const wlc_desc_t arm_state_desc;
static const wlc_desc_t hand_state_desc;
static const wlc_desc_t fault_snapshot_desc;
static const wlc_desc_t diagnostics_desc;
static const wlc_desc_t fault_event_desc;
static const wlc_desc_t operation_request_desc;
static const wlc_desc_t operation_result_desc;
static const wlc_desc_t configure_request_desc;
static const wlc_desc_t configure_result_desc;
static const wlc_desc_t peer_arm_state_desc;

static const wlc_field_t stream_header_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(stream_header_t, schema_revision), offsetof(stream_header_t, has_schema_revision), 0, 0, sizeof(uint16_t), 0U, 0U, { 8U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 2U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(stream_header_t, source_node), offsetof(stream_header_t, has_source_node), 0, 0, sizeof(uint16_t), 0U, 0U, { 16U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(stream_header_t, target_node), offsetof(stream_header_t, has_target_node), 0, 0, sizeof(uint16_t), 0U, 0U, { 24U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(stream_header_t, source_boot_id), offsetof(stream_header_t, has_source_boot_id), 0, 0, sizeof(uint64_t), 0U, 0U, { 33U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(stream_header_t, target_boot_id), offsetof(stream_header_t, has_target_boot_id), 0, 0, sizeof(uint64_t), 0U, 0U, { 41U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(stream_header_t, sequence), offsetof(stream_header_t, has_sequence), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(stream_header_t, tx_monotonic_us), offsetof(stream_header_t, has_tx_monotonic_us), 0, 0, sizeof(uint64_t), 0U, 0U, { 57U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t stream_header_desc = { stream_header_fields, sizeof(stream_header_fields) / sizeof(stream_header_fields[0]), WLC_LOOKUP_LINEAR };

static const wlc_field_t arm_command_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(arm_command_t, header), offsetof(arm_command_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(arm_command_t, control_lease), offsetof(arm_command_t, has_control_lease), 0, 0, sizeof(uint64_t), 0U, 0U, { 17U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_command_t, policy_revision), offsetof(arm_command_t, has_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_command_t, valid_for_us), offsetof(arm_command_t, has_valid_for_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 37U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_command_t, joint_mask), offsetof(arm_command_t, has_joint_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 45U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(arm_command_t, mit), offsetof(arm_command_t, has_mit), 0, 0, sizeof(float), 30U, 0U, { 50U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t arm_command_desc = { arm_command_fields, sizeof(arm_command_fields) / sizeof(arm_command_fields[0]), WLC_LOOKUP_LINEAR };

static const wlc_field_t hand_command_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(hand_command_t, header), offsetof(hand_command_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(hand_command_t, control_lease), offsetof(hand_command_t, has_control_lease), 0, 0, sizeof(uint64_t), 0U, 0U, { 17U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(hand_command_t, policy_revision), offsetof(hand_command_t, has_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(hand_command_t, valid_for_us), offsetof(hand_command_t, has_valid_for_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 37U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(hand_command_t, target_q_rad), offsetof(hand_command_t, has_target_q_rad), 0, 0, sizeof(float), 6U, 0U, { 42U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(hand_command_t, duration_ms), offsetof(hand_command_t, has_duration_ms), 0, 0, sizeof(uint16_t), 0U, 0U, { 48U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t hand_command_desc = { hand_command_fields, sizeof(hand_command_fields) / sizeof(hand_command_fields[0]), WLC_LOOKUP_LINEAR };

static const wlc_field_t arm_state_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(arm_state_t, header), offsetof(arm_state_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, last_applied_command_sequence), offsetof(arm_state_t, has_last_applied_command_sequence), 0, 0, sizeof(uint32_t), 0U, 0U, { 21U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, command_age_us), offsetof(arm_state_t, has_command_age_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, policy_revision), offsetof(arm_state_t, has_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 37U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(arm_state_t, control_owner), offsetof(arm_state_t, has_control_owner), 0, 0, sizeof(uint8_t), 0U, 0U, { 40U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(arm_state_t, run_state), offsetof(arm_state_t, has_run_state), 0, 0, sizeof(uint8_t), 0U, 0U, { 48U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(arm_state_t, compensation_mode), offsetof(arm_state_t, has_compensation_mode), 0, 0, sizeof(uint8_t), 0U, 0U, { 56U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, online_mask), offsetof(arm_state_t, has_online_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 69U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, valid_q_mask), offsetof(arm_state_t, has_valid_q_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 77U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, valid_dq_mask), offsetof(arm_state_t, has_valid_dq_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 85U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(arm_state_t, valid_tau_mask), offsetof(arm_state_t, has_valid_tau_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 93U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(arm_state_t, feedback_age_us), offsetof(arm_state_t, has_feedback_age_us), 0, 0, sizeof(uint32_t), 6U, 0U, { 98U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(arm_state_t, q_rad), offsetof(arm_state_t, has_q_rad), 0, 0, sizeof(float), 6U, 0U, { 106U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 14U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(arm_state_t, dq_rad_s), offsetof(arm_state_t, has_dq_rad_s), 0, 0, sizeof(float), 6U, 0U, { 114U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 15U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(arm_state_t, tau_measured_nm), offsetof(arm_state_t, has_tau_measured_nm), 0, 0, sizeof(float), 6U, 0U, { 122U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 16U, WLC_PACKED, WLC_F32, 1, 2U, 2U, offsetof(arm_state_t, fault_code_raw), offsetof(arm_state_t, has_fault_code_raw), 0, 0, sizeof(uint32_t), 6U, 0U, { 130U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 17U, WLC_PACKED, WLC_F32, 1, 2U, 2U, offsetof(arm_state_t, motor_mode_raw), offsetof(arm_state_t, has_motor_mode_raw), 0, 0, sizeof(uint32_t), 6U, 0U, { 138U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 18U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(arm_state_t, motor_fault_valid_mask), offsetof(arm_state_t, has_motor_fault_valid_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 149U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 19U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(arm_state_t, last_state_probe_sequence), offsetof(arm_state_t, has_last_state_probe_sequence), 0, 0, sizeof(uint32_t), 0U, 0U, { 157U, 1U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t arm_state_desc = { arm_state_fields, sizeof(arm_state_fields) / sizeof(arm_state_fields[0]), WLC_LOOKUP_DENSE };

static const wlc_field_t hand_state_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(hand_state_t, header), offsetof(hand_state_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(hand_state_t, last_applied_command_sequence), offsetof(hand_state_t, has_last_applied_command_sequence), 0, 0, sizeof(uint32_t), 0U, 0U, { 21U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(hand_state_t, command_age_us), offsetof(hand_state_t, has_command_age_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_BOOL, 1, 0U, 1U, offsetof(hand_state_t, online), offsetof(hand_state_t, has_online), 0, 0, sizeof(bool), 0U, 0U, { 32U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(hand_state_t, valid_fields), offsetof(hand_state_t, has_valid_fields), 0, 0, sizeof(uint32_t), 0U, 0U, { 45U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(hand_state_t, field_age_us), offsetof(hand_state_t, has_field_age_us), 0, 0, sizeof(uint32_t), 5U, 0U, { 50U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(hand_state_t, motor_position_percent), offsetof(hand_state_t, has_motor_position_percent), 0, 0, sizeof(float), 6U, 0U, { 58U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(hand_state_t, joint_q_rad), offsetof(hand_state_t, has_joint_q_rad), 0, 0, sizeof(float), 11U, 0U, { 66U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(hand_state_t, motor_dq_rad_s), offsetof(hand_state_t, has_motor_dq_rad_s), 0, 0, sizeof(float), 6U, 0U, { 74U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(hand_state_t, motor_current_ma), offsetof(hand_state_t, has_motor_current_ma), 0, 0, sizeof(float), 6U, 0U, { 82U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(hand_state_t, fault_code_raw), offsetof(hand_state_t, has_fault_code_raw), 0, 0, sizeof(uint32_t), 6U, 0U, { 90U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(hand_state_t, mode_raw), offsetof(hand_state_t, has_mode_raw), 0, 0, sizeof(uint16_t), 0U, 0U, { 96U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(hand_state_t, fault_valid_mask), offsetof(hand_state_t, has_fault_valid_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 109U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 14U, WLC_OPTIONAL, WLC_BOOL, 1, 0U, 1U, offsetof(hand_state_t, tactile_capable), offsetof(hand_state_t, has_tactile_capable), 0, 0, sizeof(bool), 0U, 0U, { 112U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 15U, WLC_OPTIONAL, WLC_BOOL, 1, 0U, 1U, offsetof(hand_state_t, tactile_valid), offsetof(hand_state_t, has_tactile_valid), 0, 0, sizeof(bool), 0U, 0U, { 120U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t hand_state_desc = { hand_state_fields, sizeof(hand_state_fields) / sizeof(hand_state_fields[0]), WLC_LOOKUP_DENSE };

static const wlc_field_t fault_snapshot_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(fault_snapshot_t, header), offsetof(fault_snapshot_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, policy_revision), offsetof(fault_snapshot_t, has_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 21U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, mcu_fault_active), offsetof(fault_snapshot_t, has_mcu_fault_active), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, mcu_fault_latched), offsetof(fault_snapshot_t, has_mcu_fault_latched), 0, 0, sizeof(uint32_t), 0U, 0U, { 37U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, mcu_fault_valid_mask), offsetof(fault_snapshot_t, has_mcu_fault_valid_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 45U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(fault_snapshot_t, local_action), offsetof(fault_snapshot_t, has_local_action), 0, 0, sizeof(uint8_t), 0U, 0U, { 48U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(fault_snapshot_t, arm_fault_raw), offsetof(fault_snapshot_t, has_arm_fault_raw), 0, 0, sizeof(uint32_t), 6U, 0U, { 58U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, arm_fault_valid_mask), offsetof(fault_snapshot_t, has_arm_fault_valid_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 69U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(fault_snapshot_t, hand_fault_raw), offsetof(fault_snapshot_t, has_hand_fault_raw), 0, 0, sizeof(uint32_t), 6U, 0U, { 74U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, hand_fault_valid_mask), offsetof(fault_snapshot_t, has_hand_fault_valid_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 85U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(fault_snapshot_t, latest_event_id), offsetof(fault_snapshot_t, has_latest_event_id), 0, 0, sizeof(uint64_t), 0U, 0U, { 89U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(fault_snapshot_t, oldest_retained_event_id), offsetof(fault_snapshot_t, has_oldest_retained_event_id), 0, 0, sizeof(uint64_t), 0U, 0U, { 97U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_snapshot_t, event_overflow_count), offsetof(fault_snapshot_t, has_event_overflow_count), 0, 0, sizeof(uint32_t), 0U, 0U, { 109U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t fault_snapshot_desc = { fault_snapshot_fields, sizeof(fault_snapshot_fields) / sizeof(fault_snapshot_fields[0]), WLC_LOOKUP_DENSE };

static const wlc_field_t diagnostics_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(diagnostics_t, header), offsetof(diagnostics_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_tx_completed), offsetof(diagnostics_t, has_can_tx_completed), 0, 0, sizeof(uint32_t), 0U, 0U, { 21U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_tx_failed), offsetof(diagnostics_t, has_can_tx_failed), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_rx_valid), offsetof(diagnostics_t, has_can_rx_valid), 0, 0, sizeof(uint32_t), 0U, 0U, { 37U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_rx_unknown), offsetof(diagnostics_t, has_can_rx_unknown), 0, 0, sizeof(uint32_t), 0U, 0U, { 45U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_rx_ambiguous), offsetof(diagnostics_t, has_can_rx_ambiguous), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_rx_malformed), offsetof(diagnostics_t, has_can_rx_malformed), 0, 0, sizeof(uint32_t), 0U, 0U, { 61U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_bus_state), offsetof(diagnostics_t, has_can_bus_state), 0, 0, sizeof(uint32_t), 0U, 0U, { 69U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_tx_error_count), offsetof(diagnostics_t, has_can_tx_error_count), 0, 0, sizeof(uint32_t), 0U, 0U, { 77U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, can_rx_error_count), offsetof(diagnostics_t, has_can_rx_error_count), 0, 0, sizeof(uint32_t), 0U, 0U, { 85U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, udp_rx_valid), offsetof(diagnostics_t, has_udp_rx_valid), 0, 0, sizeof(uint32_t), 0U, 0U, { 93U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, udp_rx_bad), offsetof(diagnostics_t, has_udp_rx_bad), 0, 0, sizeof(uint32_t), 0U, 0U, { 101U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, udp_rx_old), offsetof(diagnostics_t, has_udp_rx_old), 0, 0, sizeof(uint32_t), 0U, 0U, { 109U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 14U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, udp_tx_drop), offsetof(diagnostics_t, has_udp_tx_drop), 0, 0, sizeof(uint32_t), 0U, 0U, { 117U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 15U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(diagnostics_t, control_period_mean_us), offsetof(diagnostics_t, has_control_period_mean_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 125U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 16U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, control_period_max_us), offsetof(diagnostics_t, has_control_period_max_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 133U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 17U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, control_jitter_abs_max_us), offsetof(diagnostics_t, has_control_jitter_abs_max_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 141U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 18U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, command_coalesced), offsetof(diagnostics_t, has_command_coalesced), 0, 0, sizeof(uint32_t), 0U, 0U, { 149U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 19U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, counter_valid_mask), offsetof(diagnostics_t, has_counter_valid_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 157U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 20U, WLC_PACKED, WLC_F32, 1, 2U, 2U, offsetof(diagnostics_t, motor_feedback_count), offsetof(diagnostics_t, has_motor_feedback_count), 0, 0, sizeof(uint32_t), 6U, 0U, { 162U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 21U, WLC_PACKED, WLC_F32, 1, 2U, 2U, offsetof(diagnostics_t, hand_field_feedback_count), offsetof(diagnostics_t, has_hand_field_feedback_count), 0, 0, sizeof(uint32_t), 5U, 0U, { 170U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 22U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, state_probe_received), offsetof(diagnostics_t, has_state_probe_received), 0, 0, sizeof(uint32_t), 0U, 0U, { 181U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 23U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, last_state_probe_sequence), offsetof(diagnostics_t, has_last_state_probe_sequence), 0, 0, sizeof(uint32_t), 0U, 0U, { 189U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 24U, WLC_OPTIONAL, WLC_U8, 1, 0U, 2U, offsetof(diagnostics_t, hand_state_query_mode), offsetof(diagnostics_t, has_hand_state_query_mode), 0, 0, sizeof(uint8_t), 0U, 0U, { 192U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 25U, WLC_OPTIONAL, WLC_U8, 1, 0U, 2U, offsetof(diagnostics_t, hand_mode_guard_phase), offsetof(diagnostics_t, has_hand_mode_guard_phase), 0, 0, sizeof(uint8_t), 0U, 0U, { 200U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 26U, WLC_OPTIONAL, WLC_I32, 1, 0U, 2U, offsetof(diagnostics_t, hand_mode_guard_status), offsetof(diagnostics_t, has_hand_mode_guard_status), 0, 0, sizeof(int32_t), 0U, 0U, { 208U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 27U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_mode_guard_elapsed_us), offsetof(diagnostics_t, has_hand_mode_guard_elapsed_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 221U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 28U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_mode_guard_token), offsetof(diagnostics_t, has_hand_mode_guard_token), 0, 0, sizeof(uint32_t), 0U, 0U, { 229U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 29U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_control_tx), offsetof(diagnostics_t, has_hand_control_tx), 0, 0, sizeof(uint32_t), 0U, 0U, { 237U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 30U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_control_ack), offsetof(diagnostics_t, has_hand_control_ack), 0, 0, sizeof(uint32_t), 0U, 0U, { 245U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 31U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_query_timeouts), offsetof(diagnostics_t, has_hand_query_timeouts), 0, 0, sizeof(uint32_t), 0U, 0U, { 253U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 32U, WLC_PACKED, WLC_F32, 1, 2U, 2U, offsetof(diagnostics_t, motor_control_tx), offsetof(diagnostics_t, has_motor_control_tx), 0, 0, sizeof(uint32_t), 6U, 0U, { 130U, 2U, 0U }, 0, 0ULL, NULL, NULL },
  { 33U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_full_query_tx), offsetof(diagnostics_t, has_hand_full_query_tx), 0, 0, sizeof(uint32_t), 0U, 0U, { 141U, 2U, 0U }, 0, 0ULL, NULL, NULL },
  { 34U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_control_negative_ack), offsetof(diagnostics_t, has_hand_control_negative_ack), 0, 0, sizeof(uint32_t), 0U, 0U, { 149U, 2U, 0U }, 0, 0ULL, NULL, NULL },
  { 35U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(diagnostics_t, hand_control_ack_latest_status), offsetof(diagnostics_t, has_hand_control_ack_latest_status), 0, 0, sizeof(uint32_t), 0U, 0U, { 157U, 2U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t diagnostics_desc = { diagnostics_fields, sizeof(diagnostics_fields) / sizeof(diagnostics_fields[0]), WLC_LOOKUP_DENSE };

static const wlc_field_t fault_event_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(fault_event_t, header), offsetof(fault_event_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(fault_event_t, event_id), offsetof(fault_event_t, has_event_id), 0, 0, sizeof(uint64_t), 0U, 0U, { 17U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(fault_event_t, device_class), offsetof(fault_event_t, has_device_class), 0, 0, sizeof(uint8_t), 0U, 0U, { 24U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(fault_event_t, device_id), offsetof(fault_event_t, has_device_id), 0, 0, sizeof(uint16_t), 0U, 0U, { 32U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(fault_event_t, device_channel), offsetof(fault_event_t, has_device_channel), 0, 0, sizeof(uint8_t), 0U, 0U, { 40U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_event_t, raw_code), offsetof(fault_event_t, has_raw_code), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_event_t, normalized_code), offsetof(fault_event_t, has_normalized_code), 0, 0, sizeof(uint32_t), 0U, 0U, { 61U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(fault_event_t, severity), offsetof(fault_event_t, has_severity), 0, 0, sizeof(uint8_t), 0U, 0U, { 64U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_OPTIONAL, WLC_BOOL, 1, 0U, 1U, offsetof(fault_event_t, active), offsetof(fault_event_t, has_active), 0, 0, sizeof(bool), 0U, 0U, { 72U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(fault_event_t, local_action), offsetof(fault_event_t, has_local_action), 0, 0, sizeof(uint8_t), 0U, 0U, { 80U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_event_t, policy_revision), offsetof(fault_event_t, has_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 93U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(fault_event_t, first_seen_us), offsetof(fault_event_t, has_first_seen_us), 0, 0, sizeof(uint64_t), 0U, 0U, { 97U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(fault_event_t, last_seen_us), offsetof(fault_event_t, has_last_seen_us), 0, 0, sizeof(uint64_t), 0U, 0U, { 105U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 14U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(fault_event_t, occurrences), offsetof(fault_event_t, has_occurrences), 0, 0, sizeof(uint32_t), 0U, 0U, { 117U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t fault_event_desc = { fault_event_fields, sizeof(fault_event_fields) / sizeof(fault_event_fields[0]), WLC_LOOKUP_DENSE };

static const wlc_field_t operation_request_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(operation_request_t, header), offsetof(operation_request_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_U32, 1, 0U, 1U, offsetof(operation_request_t, operation_id), offsetof(operation_request_t, has_operation_id), 0, 0, sizeof(uint32_t), 0U, 0U, { 16U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(operation_request_t, operation), offsetof(operation_request_t, has_operation), 0, 0, sizeof(uint16_t), 0U, 0U, { 24U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(operation_request_t, control_lease), offsetof(operation_request_t, has_control_lease), 0, 0, sizeof(uint64_t), 0U, 0U, { 33U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(operation_request_t, expected_policy_revision), offsetof(operation_request_t, has_expected_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 45U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(operation_request_t, target_mask), offsetof(operation_request_t, has_target_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(operation_request_t, fault_event_id), offsetof(operation_request_t, has_fault_event_id), 0, 0, sizeof(uint64_t), 0U, 0U, { 57U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t operation_request_desc = { operation_request_fields, sizeof(operation_request_fields) / sizeof(operation_request_fields[0]), WLC_LOOKUP_LINEAR };

static const wlc_field_t operation_result_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(operation_result_t, header), offsetof(operation_result_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_U32, 1, 0U, 1U, offsetof(operation_result_t, operation_id), offsetof(operation_result_t, has_operation_id), 0, 0, sizeof(uint32_t), 0U, 0U, { 16U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_I32, 1, 0U, 1U, offsetof(operation_result_t, status), offsetof(operation_result_t, has_status), 0, 0, sizeof(int32_t), 0U, 0U, { 24U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(operation_result_t, reason), offsetof(operation_result_t, has_reason), 0, 0, sizeof(uint16_t), 0U, 0U, { 32U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(operation_result_t, observed_run_state), offsetof(operation_result_t, has_observed_run_state), 0, 0, sizeof(uint8_t), 0U, 0U, { 40U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(operation_result_t, applied_policy_revision), offsetof(operation_result_t, has_applied_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(operation_result_t, control_lease), offsetof(operation_result_t, has_control_lease), 0, 0, sizeof(uint64_t), 0U, 0U, { 57U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t operation_result_desc = { operation_result_fields, sizeof(operation_result_fields) / sizeof(operation_result_fields[0]), WLC_LOOKUP_LINEAR };

static const wlc_field_t configure_request_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(configure_request_t, header), offsetof(configure_request_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_U32, 1, 0U, 1U, offsetof(configure_request_t, operation_id), offsetof(configure_request_t, has_operation_id), 0, 0, sizeof(uint32_t), 0U, 0U, { 16U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_request_t, expected_policy_revision), offsetof(configure_request_t, has_expected_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_request_t, new_policy_revision), offsetof(configure_request_t, has_new_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 37U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(configure_request_t, gravity_owner), offsetof(configure_request_t, has_gravity_owner), 0, 0, sizeof(uint8_t), 0U, 0U, { 40U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(configure_request_t, friction_owner), offsetof(configure_request_t, has_friction_owner), 0, 0, sizeof(uint8_t), 0U, 0U, { 48U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(configure_request_t, disconnect_action), offsetof(configure_request_t, has_disconnect_action), 0, 0, sizeof(uint8_t), 0U, 0U, { 56U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(configure_request_t, teleop_mode), offsetof(configure_request_t, has_teleop_mode), 0, 0, sizeof(uint8_t), 0U, 0U, { 64U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_request_t, arm_command_timeout_us), offsetof(configure_request_t, has_arm_command_timeout_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 77U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_request_t, hand_command_timeout_us), offsetof(configure_request_t, has_hand_command_timeout_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 85U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_request_t, arm_feedback_timeout_us), offsetof(configure_request_t, has_arm_feedback_timeout_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 93U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_request_t, hand_feedback_timeout_us), offsetof(configure_request_t, has_hand_feedback_timeout_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 101U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(configure_request_t, hand_mode), offsetof(configure_request_t, has_hand_mode), 0, 0, sizeof(uint8_t), 0U, 0U, { 104U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 14U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(configure_request_t, hand_soft_current_ma), offsetof(configure_request_t, has_hand_soft_current_ma), 0, 0, sizeof(uint32_t), 6U, 0U, { 114U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 15U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(configure_request_t, hand_current_semantics), offsetof(configure_request_t, has_hand_current_semantics), 0, 0, sizeof(uint8_t), 0U, 0U, { 120U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 16U, WLC_OPTIONAL, WLC_BOOL, 1, 0U, 2U, offsetof(configure_request_t, local_fallback_enabled), offsetof(configure_request_t, has_local_fallback_enabled), 0, 0, sizeof(bool), 0U, 0U, { 128U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 17U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, model_revision), offsetof(configure_request_t, has_model_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 141U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 18U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, host_managed_fault_mask), offsetof(configure_request_t, has_host_managed_fault_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 149U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 19U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, hard_protection_profile_id), offsetof(configure_request_t, has_hard_protection_profile_id), 0, 0, sizeof(uint32_t), 0U, 0U, { 157U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 20U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, max_command_valid_for_us), offsetof(configure_request_t, has_max_command_valid_for_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 165U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 23U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 2U, offsetof(configure_request_t, force_feedback_gain), offsetof(configure_request_t, has_force_feedback_gain), 0, 0, sizeof(float), 6U, 0U, { 186U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 25U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, arm_state_period_us), offsetof(configure_request_t, has_arm_state_period_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 205U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 26U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, hand_state_period_us), offsetof(configure_request_t, has_hand_state_period_us), 0, 0, sizeof(uint32_t), 0U, 0U, { 213U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 27U, WLC_OPTIONAL, WLC_F32, 1, 5U, 2U, offsetof(configure_request_t, hand_control_feedforward_ma), offsetof(configure_request_t, has_hand_control_feedforward_ma), 0, 0, sizeof(uint32_t), 0U, 0U, { 221U, 1U, 0U }, 0, 0ULL, NULL, NULL },
  { 28U, WLC_PACKED, WLC_F32, 1, 2U, 2U, offsetof(configure_request_t, hand_position_current_limit_ma), offsetof(configure_request_t, has_hand_position_current_limit_ma), 0, 0, sizeof(uint32_t), 6U, 0U, { 226U, 1U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t configure_request_desc = { configure_request_fields, sizeof(configure_request_fields) / sizeof(configure_request_fields[0]), WLC_LOOKUP_BINARY };

static const wlc_field_t configure_result_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(configure_result_t, header), offsetof(configure_result_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_U32, 1, 0U, 1U, offsetof(configure_result_t, operation_id), offsetof(configure_result_t, has_operation_id), 0, 0, sizeof(uint32_t), 0U, 0U, { 16U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_I32, 1, 0U, 1U, offsetof(configure_result_t, status), offsetof(configure_result_t, has_status), 0, 0, sizeof(int32_t), 0U, 0U, { 24U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_U16, 1, 0U, 1U, offsetof(configure_result_t, reason), offsetof(configure_result_t, has_reason), 0, 0, sizeof(uint16_t), 0U, 0U, { 32U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_result_t, applied_policy_revision), offsetof(configure_result_t, has_applied_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 45U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_result_t, hard_protection_mask), offsetof(configure_result_t, has_hard_protection_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(configure_result_t, capabilities), offsetof(configure_result_t, has_capabilities), 0, 0, sizeof(uint32_t), 0U, 0U, { 61U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t configure_result_desc = { configure_result_fields, sizeof(configure_result_fields) / sizeof(configure_result_fields[0]), WLC_LOOKUP_LINEAR };

static const wlc_field_t peer_arm_state_fields[] = {
  { 1U, WLC_OPTIONAL, WLC_MESSAGE, 1, 2U, 1U, offsetof(peer_arm_state_t, header), offsetof(peer_arm_state_t, has_header), 0, 0, sizeof(stream_header_t), 0U, 0U, { 10U, 0U, 0U }, 0, 0ULL, NULL, &stream_header_desc },
  { 2U, WLC_OPTIONAL, WLC_F64, 1, 1U, 1U, offsetof(peer_arm_state_t, pairing_session), offsetof(peer_arm_state_t, has_pairing_session), 0, 0, sizeof(uint64_t), 0U, 0U, { 17U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 3U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(peer_arm_state_t, policy_revision), offsetof(peer_arm_state_t, has_policy_revision), 0, 0, sizeof(uint32_t), 0U, 0U, { 29U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 4U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(peer_arm_state_t, role), offsetof(peer_arm_state_t, has_role), 0, 0, sizeof(uint8_t), 0U, 0U, { 32U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 5U, WLC_OPTIONAL, WLC_U8, 1, 0U, 1U, offsetof(peer_arm_state_t, teleop_mode), offsetof(peer_arm_state_t, has_teleop_mode), 0, 0, sizeof(uint8_t), 0U, 0U, { 40U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 6U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(peer_arm_state_t, valid_q_mask), offsetof(peer_arm_state_t, has_valid_q_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 53U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 7U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(peer_arm_state_t, valid_dq_mask), offsetof(peer_arm_state_t, has_valid_dq_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 61U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 8U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(peer_arm_state_t, valid_residual_tau_mask), offsetof(peer_arm_state_t, has_valid_residual_tau_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 69U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 9U, WLC_PACKED, WLC_F32, 1, 2U, 1U, offsetof(peer_arm_state_t, feedback_age_us), offsetof(peer_arm_state_t, has_feedback_age_us), 0, 0, sizeof(uint32_t), 6U, 0U, { 74U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 10U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(peer_arm_state_t, q_rad), offsetof(peer_arm_state_t, has_q_rad), 0, 0, sizeof(float), 6U, 0U, { 82U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 11U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(peer_arm_state_t, dq_rad_s), offsetof(peer_arm_state_t, has_dq_rad_s), 0, 0, sizeof(float), 6U, 0U, { 90U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 12U, WLC_PACKED, WLC_FLOAT32, 1, 2U, 1U, offsetof(peer_arm_state_t, residual_tau_nm), offsetof(peer_arm_state_t, has_residual_tau_nm), 0, 0, sizeof(float), 6U, 0U, { 98U, 0U, 0U }, 0, 0ULL, NULL, NULL },
  { 13U, WLC_OPTIONAL, WLC_F32, 1, 5U, 1U, offsetof(peer_arm_state_t, critical_fault_mask), offsetof(peer_arm_state_t, has_critical_fault_mask), 0, 0, sizeof(uint32_t), 0U, 0U, { 109U, 0U, 0U }, 0, 0ULL, NULL, NULL },
};
static const wlc_desc_t peer_arm_state_desc = { peer_arm_state_fields, sizeof(peer_arm_state_fields) / sizeof(peer_arm_state_fields[0]), WLC_LOOKUP_DENSE };

void stream_header_clear(stream_header_t *value) { if (value != NULL) wlc_clear(&stream_header_desc, value); }
size_t stream_header_encoded_size(const stream_header_t *value) { size_t size; return wlc_measure(&stream_header_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t stream_header_encode(const stream_header_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&stream_header_desc, value, out, cap, length); }
wl_codec_status_t stream_header_decode(const uint8_t *input, size_t length, stream_header_t *out) { return wlc_decode(&stream_header_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t stream_header_wlc_detail_fingerprint(const stream_header_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&stream_header_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void arm_command_clear(arm_command_t *value) { if (value != NULL) wlc_clear(&arm_command_desc, value); }
size_t arm_command_encoded_size(const arm_command_t *value) { size_t size; return wlc_measure(&arm_command_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t arm_command_encode(const arm_command_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&arm_command_desc, value, out, cap, length); }
wl_codec_status_t arm_command_decode(const uint8_t *input, size_t length, arm_command_t *out) { return wlc_decode(&arm_command_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t arm_command_wlc_detail_fingerprint(const arm_command_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&arm_command_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void hand_command_clear(hand_command_t *value) { if (value != NULL) wlc_clear(&hand_command_desc, value); }
size_t hand_command_encoded_size(const hand_command_t *value) { size_t size; return wlc_measure(&hand_command_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t hand_command_encode(const hand_command_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&hand_command_desc, value, out, cap, length); }
wl_codec_status_t hand_command_decode(const uint8_t *input, size_t length, hand_command_t *out) { return wlc_decode(&hand_command_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t hand_command_wlc_detail_fingerprint(const hand_command_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&hand_command_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void arm_state_clear(arm_state_t *value) { if (value != NULL) wlc_clear(&arm_state_desc, value); }
size_t arm_state_encoded_size(const arm_state_t *value) { size_t size; return wlc_measure(&arm_state_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t arm_state_encode(const arm_state_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&arm_state_desc, value, out, cap, length); }
wl_codec_status_t arm_state_decode(const uint8_t *input, size_t length, arm_state_t *out) { return wlc_decode(&arm_state_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t arm_state_wlc_detail_fingerprint(const arm_state_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&arm_state_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void hand_state_clear(hand_state_t *value) { if (value != NULL) wlc_clear(&hand_state_desc, value); }
size_t hand_state_encoded_size(const hand_state_t *value) { size_t size; return wlc_measure(&hand_state_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t hand_state_encode(const hand_state_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&hand_state_desc, value, out, cap, length); }
wl_codec_status_t hand_state_decode(const uint8_t *input, size_t length, hand_state_t *out) { return wlc_decode(&hand_state_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t hand_state_wlc_detail_fingerprint(const hand_state_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&hand_state_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void fault_snapshot_clear(fault_snapshot_t *value) { if (value != NULL) wlc_clear(&fault_snapshot_desc, value); }
size_t fault_snapshot_encoded_size(const fault_snapshot_t *value) { size_t size; return wlc_measure(&fault_snapshot_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t fault_snapshot_encode(const fault_snapshot_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&fault_snapshot_desc, value, out, cap, length); }
wl_codec_status_t fault_snapshot_decode(const uint8_t *input, size_t length, fault_snapshot_t *out) { return wlc_decode(&fault_snapshot_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t fault_snapshot_wlc_detail_fingerprint(const fault_snapshot_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&fault_snapshot_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void diagnostics_clear(diagnostics_t *value) { if (value != NULL) wlc_clear(&diagnostics_desc, value); }
size_t diagnostics_encoded_size(const diagnostics_t *value) { size_t size; return wlc_measure(&diagnostics_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t diagnostics_encode(const diagnostics_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&diagnostics_desc, value, out, cap, length); }
wl_codec_status_t diagnostics_decode(const uint8_t *input, size_t length, diagnostics_t *out) { return wlc_decode(&diagnostics_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t diagnostics_wlc_detail_fingerprint(const diagnostics_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&diagnostics_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void fault_event_clear(fault_event_t *value) { if (value != NULL) wlc_clear(&fault_event_desc, value); }
size_t fault_event_encoded_size(const fault_event_t *value) { size_t size; return wlc_measure(&fault_event_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t fault_event_encode(const fault_event_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&fault_event_desc, value, out, cap, length); }
wl_codec_status_t fault_event_decode(const uint8_t *input, size_t length, fault_event_t *out) { return wlc_decode(&fault_event_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t fault_event_wlc_detail_fingerprint(const fault_event_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&fault_event_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void operation_request_clear(operation_request_t *value) { if (value != NULL) wlc_clear(&operation_request_desc, value); }
size_t operation_request_encoded_size(const operation_request_t *value) { size_t size; return wlc_measure(&operation_request_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t operation_request_encode(const operation_request_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&operation_request_desc, value, out, cap, length); }
wl_codec_status_t operation_request_decode(const uint8_t *input, size_t length, operation_request_t *out) { return wlc_decode(&operation_request_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t operation_request_wlc_detail_fingerprint(const operation_request_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&operation_request_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void operation_result_clear(operation_result_t *value) { if (value != NULL) wlc_clear(&operation_result_desc, value); }
size_t operation_result_encoded_size(const operation_result_t *value) { size_t size; return wlc_measure(&operation_result_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t operation_result_encode(const operation_result_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&operation_result_desc, value, out, cap, length); }
wl_codec_status_t operation_result_decode(const uint8_t *input, size_t length, operation_result_t *out) { return wlc_decode(&operation_result_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t operation_result_wlc_detail_fingerprint(const operation_result_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&operation_result_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void configure_request_clear(configure_request_t *value) { if (value != NULL) wlc_clear(&configure_request_desc, value); }
size_t configure_request_encoded_size(const configure_request_t *value) { size_t size; return wlc_measure(&configure_request_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t configure_request_encode(const configure_request_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&configure_request_desc, value, out, cap, length); }
wl_codec_status_t configure_request_decode(const uint8_t *input, size_t length, configure_request_t *out) { return wlc_decode(&configure_request_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t configure_request_wlc_detail_fingerprint(const configure_request_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&configure_request_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void configure_result_clear(configure_result_t *value) { if (value != NULL) wlc_clear(&configure_result_desc, value); }
size_t configure_result_encoded_size(const configure_result_t *value) { size_t size; return wlc_measure(&configure_result_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t configure_result_encode(const configure_result_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&configure_result_desc, value, out, cap, length); }
wl_codec_status_t configure_result_decode(const uint8_t *input, size_t length, configure_result_t *out) { return wlc_decode(&configure_result_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t configure_result_wlc_detail_fingerprint(const configure_result_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&configure_result_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

void peer_arm_state_clear(peer_arm_state_t *value) { if (value != NULL) wlc_clear(&peer_arm_state_desc, value); }
size_t peer_arm_state_encoded_size(const peer_arm_state_t *value) { size_t size; return wlc_measure(&peer_arm_state_desc, value, &size) == WL_CODEC_OK ? size : SIZE_MAX; }
wl_codec_status_t peer_arm_state_encode(const peer_arm_state_t *value, uint8_t *out, size_t cap, size_t *length) { return wlc_encode(&peer_arm_state_desc, value, out, cap, length); }
wl_codec_status_t peer_arm_state_decode(const uint8_t *input, size_t length, peer_arm_state_t *out) { return wlc_decode(&peer_arm_state_desc, input, length, out); }

/* Generator-private: value must be the unmodified result of successful decode.
 * hash supplies the caller domain seed; codec has no RPC identity policy. */
wl_codec_status_t peer_arm_state_wlc_detail_fingerprint(const peer_arm_state_t *value, uint64_t *hash, size_t *length) {
  wlc_hash_state_t state = {*hash, 0U, WL_CODEC_OK};
  wl_codec_status_t status = wlc_hash_emit_fields(&peer_arm_state_desc, value, &state);
  if (status != WL_CODEC_OK) return status;
  if (state.status != WL_CODEC_OK) return state.status;
  *hash = state.hash;
  *length = state.length;
  return WL_CODEC_OK;
}

static void stream_header_value_defaults(stream_header_value_t *out) {
  (void)out;
  out->schema_revision = 0;
  out->source_node = 0;
  out->target_node = 0;
  out->source_boot_id = UINT64_C(0);
  out->target_boot_id = UINT64_C(0);
  out->sequence = UINT32_C(0);
  out->tx_monotonic_us = UINT64_C(0);
}

static void stream_header_value_copy_fields(const stream_header_t *view, stream_header_value_t *out) {
  out->has_schema_revision = view->has_schema_revision;
  out->schema_revision = view->has_schema_revision ? view->schema_revision : 0;
  out->has_source_node = view->has_source_node;
  out->source_node = view->has_source_node ? view->source_node : 0;
  out->has_target_node = view->has_target_node;
  out->target_node = view->has_target_node ? view->target_node : 0;
  out->has_source_boot_id = view->has_source_boot_id;
  out->source_boot_id = view->has_source_boot_id ? view->source_boot_id : UINT64_C(0);
  out->has_target_boot_id = view->has_target_boot_id;
  out->target_boot_id = view->has_target_boot_id ? view->target_boot_id : UINT64_C(0);
  out->has_sequence = view->has_sequence;
  out->sequence = view->has_sequence ? view->sequence : UINT32_C(0);
  out->has_tx_monotonic_us = view->has_tx_monotonic_us;
  out->tx_monotonic_us = view->has_tx_monotonic_us ? view->tx_monotonic_us : UINT64_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void stream_header_wlc_detail_value_copy(const stream_header_t *view, stream_header_value_t *out) {
  memset(out, 0, sizeof(*out));
  stream_header_value_copy_fields(view, out);
}

void stream_header_value_clear(stream_header_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  stream_header_value_defaults(value);
}

wl_codec_status_t stream_header_value_from_view(const stream_header_t *view, stream_header_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&stream_header_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  stream_header_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t stream_header_value_borrow(const stream_header_value_t *value, stream_header_t *out) {
  stream_header_clear(out);
  out->has_schema_revision = value->has_schema_revision;
  if (value->has_schema_revision) {
    out->schema_revision = value->schema_revision;
  }
  out->has_source_node = value->has_source_node;
  if (value->has_source_node) {
    out->source_node = value->source_node;
  }
  out->has_target_node = value->has_target_node;
  if (value->has_target_node) {
    out->target_node = value->target_node;
  }
  out->has_source_boot_id = value->has_source_boot_id;
  if (value->has_source_boot_id) {
    out->source_boot_id = value->source_boot_id;
  }
  out->has_target_boot_id = value->has_target_boot_id;
  if (value->has_target_boot_id) {
    out->target_boot_id = value->target_boot_id;
  }
  out->has_sequence = value->has_sequence;
  if (value->has_sequence) {
    out->sequence = value->sequence;
  }
  out->has_tx_monotonic_us = value->has_tx_monotonic_us;
  if (value->has_tx_monotonic_us) {
    out->tx_monotonic_us = value->tx_monotonic_us;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t stream_header_value_to_view(const stream_header_value_t *value, stream_header_t *out) {
  stream_header_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = stream_header_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&stream_header_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t stream_header_value_encoded_size(const stream_header_value_t *value) {
  stream_header_t view;
  if (value == NULL || stream_header_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return stream_header_encoded_size(&view);
}

wl_codec_status_t stream_header_value_encode(const stream_header_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  stream_header_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = stream_header_value_borrow(value, &view);
  return status == WL_CODEC_OK ? stream_header_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t stream_header_value_decode(const uint8_t *input, size_t length, stream_header_value_t *out) {
  stream_header_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = stream_header_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  stream_header_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void arm_command_value_defaults(arm_command_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->control_lease = UINT64_C(0);
  out->policy_revision = UINT32_C(0);
  out->valid_for_us = UINT32_C(0);
  out->joint_mask = UINT32_C(0);
}

static void arm_command_value_copy_fields(const arm_command_t *view, arm_command_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_control_lease = view->has_control_lease;
  out->control_lease = view->has_control_lease ? view->control_lease : UINT64_C(0);
  out->has_policy_revision = view->has_policy_revision;
  out->policy_revision = view->has_policy_revision ? view->policy_revision : UINT32_C(0);
  out->has_valid_for_us = view->has_valid_for_us;
  out->valid_for_us = view->has_valid_for_us ? view->valid_for_us : UINT32_C(0);
  out->has_joint_mask = view->has_joint_mask;
  out->joint_mask = view->has_joint_mask ? view->joint_mask : UINT32_C(0);
  out->has_mit = view->has_mit;
  if (view->has_mit) memcpy(out->mit, view->mit, sizeof(out->mit));
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void arm_command_wlc_detail_value_copy(const arm_command_t *view, arm_command_value_t *out) {
  memset(out, 0, sizeof(*out));
  arm_command_value_copy_fields(view, out);
}

void arm_command_value_clear(arm_command_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  arm_command_value_defaults(value);
}

wl_codec_status_t arm_command_value_from_view(const arm_command_t *view, arm_command_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&arm_command_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  arm_command_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t arm_command_value_borrow(const arm_command_value_t *value, arm_command_t *out) {
  arm_command_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_control_lease = value->has_control_lease;
  if (value->has_control_lease) {
    out->control_lease = value->control_lease;
  }
  out->has_policy_revision = value->has_policy_revision;
  if (value->has_policy_revision) {
    out->policy_revision = value->policy_revision;
  }
  out->has_valid_for_us = value->has_valid_for_us;
  if (value->has_valid_for_us) {
    out->valid_for_us = value->valid_for_us;
  }
  out->has_joint_mask = value->has_joint_mask;
  if (value->has_joint_mask) {
    out->joint_mask = value->joint_mask;
  }
  out->has_mit = value->has_mit;
  if (value->has_mit) {
    memcpy(out->mit, value->mit, sizeof(out->mit));
  }
  return WL_CODEC_OK;
}

wl_codec_status_t arm_command_value_to_view(const arm_command_value_t *value, arm_command_t *out) {
  arm_command_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = arm_command_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&arm_command_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t arm_command_value_encoded_size(const arm_command_value_t *value) {
  arm_command_t view;
  if (value == NULL || arm_command_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return arm_command_encoded_size(&view);
}

wl_codec_status_t arm_command_value_encode(const arm_command_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  arm_command_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = arm_command_value_borrow(value, &view);
  return status == WL_CODEC_OK ? arm_command_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t arm_command_value_decode(const uint8_t *input, size_t length, arm_command_value_t *out) {
  arm_command_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = arm_command_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  arm_command_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void hand_command_value_defaults(hand_command_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->control_lease = UINT64_C(0);
  out->policy_revision = UINT32_C(0);
  out->valid_for_us = UINT32_C(0);
  out->duration_ms = 0;
}

static void hand_command_value_copy_fields(const hand_command_t *view, hand_command_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_control_lease = view->has_control_lease;
  out->control_lease = view->has_control_lease ? view->control_lease : UINT64_C(0);
  out->has_policy_revision = view->has_policy_revision;
  out->policy_revision = view->has_policy_revision ? view->policy_revision : UINT32_C(0);
  out->has_valid_for_us = view->has_valid_for_us;
  out->valid_for_us = view->has_valid_for_us ? view->valid_for_us : UINT32_C(0);
  out->has_target_q_rad = view->has_target_q_rad;
  if (view->has_target_q_rad) memcpy(out->target_q_rad, view->target_q_rad, sizeof(out->target_q_rad));
  out->has_duration_ms = view->has_duration_ms;
  out->duration_ms = view->has_duration_ms ? view->duration_ms : 0;
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void hand_command_wlc_detail_value_copy(const hand_command_t *view, hand_command_value_t *out) {
  memset(out, 0, sizeof(*out));
  hand_command_value_copy_fields(view, out);
}

void hand_command_value_clear(hand_command_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  hand_command_value_defaults(value);
}

wl_codec_status_t hand_command_value_from_view(const hand_command_t *view, hand_command_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&hand_command_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  hand_command_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t hand_command_value_borrow(const hand_command_value_t *value, hand_command_t *out) {
  hand_command_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_control_lease = value->has_control_lease;
  if (value->has_control_lease) {
    out->control_lease = value->control_lease;
  }
  out->has_policy_revision = value->has_policy_revision;
  if (value->has_policy_revision) {
    out->policy_revision = value->policy_revision;
  }
  out->has_valid_for_us = value->has_valid_for_us;
  if (value->has_valid_for_us) {
    out->valid_for_us = value->valid_for_us;
  }
  out->has_target_q_rad = value->has_target_q_rad;
  if (value->has_target_q_rad) {
    memcpy(out->target_q_rad, value->target_q_rad, sizeof(out->target_q_rad));
  }
  out->has_duration_ms = value->has_duration_ms;
  if (value->has_duration_ms) {
    out->duration_ms = value->duration_ms;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t hand_command_value_to_view(const hand_command_value_t *value, hand_command_t *out) {
  hand_command_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = hand_command_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&hand_command_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t hand_command_value_encoded_size(const hand_command_value_t *value) {
  hand_command_t view;
  if (value == NULL || hand_command_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return hand_command_encoded_size(&view);
}

wl_codec_status_t hand_command_value_encode(const hand_command_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  hand_command_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = hand_command_value_borrow(value, &view);
  return status == WL_CODEC_OK ? hand_command_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t hand_command_value_decode(const uint8_t *input, size_t length, hand_command_value_t *out) {
  hand_command_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = hand_command_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  hand_command_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void arm_state_value_defaults(arm_state_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->last_applied_command_sequence = UINT32_C(0);
  out->command_age_us = UINT32_C(0);
  out->policy_revision = UINT32_C(0);
  out->control_owner = 0;
  out->run_state = 0;
  out->compensation_mode = 0;
  out->online_mask = UINT32_C(0);
  out->valid_q_mask = UINT32_C(0);
  out->valid_dq_mask = UINT32_C(0);
  out->valid_tau_mask = UINT32_C(0);
  out->motor_fault_valid_mask = UINT32_C(0);
  out->last_state_probe_sequence = UINT32_C(0);
}

static void arm_state_value_copy_fields(const arm_state_t *view, arm_state_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_last_applied_command_sequence = view->has_last_applied_command_sequence;
  out->last_applied_command_sequence = view->has_last_applied_command_sequence ? view->last_applied_command_sequence : UINT32_C(0);
  out->has_command_age_us = view->has_command_age_us;
  out->command_age_us = view->has_command_age_us ? view->command_age_us : UINT32_C(0);
  out->has_policy_revision = view->has_policy_revision;
  out->policy_revision = view->has_policy_revision ? view->policy_revision : UINT32_C(0);
  out->has_control_owner = view->has_control_owner;
  out->control_owner = view->has_control_owner ? view->control_owner : 0;
  out->has_run_state = view->has_run_state;
  out->run_state = view->has_run_state ? view->run_state : 0;
  out->has_compensation_mode = view->has_compensation_mode;
  out->compensation_mode = view->has_compensation_mode ? view->compensation_mode : 0;
  out->has_online_mask = view->has_online_mask;
  out->online_mask = view->has_online_mask ? view->online_mask : UINT32_C(0);
  out->has_valid_q_mask = view->has_valid_q_mask;
  out->valid_q_mask = view->has_valid_q_mask ? view->valid_q_mask : UINT32_C(0);
  out->has_valid_dq_mask = view->has_valid_dq_mask;
  out->valid_dq_mask = view->has_valid_dq_mask ? view->valid_dq_mask : UINT32_C(0);
  out->has_valid_tau_mask = view->has_valid_tau_mask;
  out->valid_tau_mask = view->has_valid_tau_mask ? view->valid_tau_mask : UINT32_C(0);
  out->has_feedback_age_us = view->has_feedback_age_us;
  if (view->has_feedback_age_us) memcpy(out->feedback_age_us, view->feedback_age_us, sizeof(out->feedback_age_us));
  out->has_q_rad = view->has_q_rad;
  if (view->has_q_rad) memcpy(out->q_rad, view->q_rad, sizeof(out->q_rad));
  out->has_dq_rad_s = view->has_dq_rad_s;
  if (view->has_dq_rad_s) memcpy(out->dq_rad_s, view->dq_rad_s, sizeof(out->dq_rad_s));
  out->has_tau_measured_nm = view->has_tau_measured_nm;
  if (view->has_tau_measured_nm) memcpy(out->tau_measured_nm, view->tau_measured_nm, sizeof(out->tau_measured_nm));
  out->has_fault_code_raw = view->has_fault_code_raw;
  if (view->has_fault_code_raw) memcpy(out->fault_code_raw, view->fault_code_raw, sizeof(out->fault_code_raw));
  out->has_motor_mode_raw = view->has_motor_mode_raw;
  if (view->has_motor_mode_raw) memcpy(out->motor_mode_raw, view->motor_mode_raw, sizeof(out->motor_mode_raw));
  out->has_motor_fault_valid_mask = view->has_motor_fault_valid_mask;
  out->motor_fault_valid_mask = view->has_motor_fault_valid_mask ? view->motor_fault_valid_mask : UINT32_C(0);
  out->has_last_state_probe_sequence = view->has_last_state_probe_sequence;
  out->last_state_probe_sequence = view->has_last_state_probe_sequence ? view->last_state_probe_sequence : UINT32_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void arm_state_wlc_detail_value_copy(const arm_state_t *view, arm_state_value_t *out) {
  memset(out, 0, sizeof(*out));
  arm_state_value_copy_fields(view, out);
}

void arm_state_value_clear(arm_state_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  arm_state_value_defaults(value);
}

wl_codec_status_t arm_state_value_from_view(const arm_state_t *view, arm_state_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&arm_state_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  arm_state_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t arm_state_value_borrow(const arm_state_value_t *value, arm_state_t *out) {
  arm_state_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_last_applied_command_sequence = value->has_last_applied_command_sequence;
  if (value->has_last_applied_command_sequence) {
    out->last_applied_command_sequence = value->last_applied_command_sequence;
  }
  out->has_command_age_us = value->has_command_age_us;
  if (value->has_command_age_us) {
    out->command_age_us = value->command_age_us;
  }
  out->has_policy_revision = value->has_policy_revision;
  if (value->has_policy_revision) {
    out->policy_revision = value->policy_revision;
  }
  out->has_control_owner = value->has_control_owner;
  if (value->has_control_owner) {
    out->control_owner = value->control_owner;
  }
  out->has_run_state = value->has_run_state;
  if (value->has_run_state) {
    out->run_state = value->run_state;
  }
  out->has_compensation_mode = value->has_compensation_mode;
  if (value->has_compensation_mode) {
    out->compensation_mode = value->compensation_mode;
  }
  out->has_online_mask = value->has_online_mask;
  if (value->has_online_mask) {
    out->online_mask = value->online_mask;
  }
  out->has_valid_q_mask = value->has_valid_q_mask;
  if (value->has_valid_q_mask) {
    out->valid_q_mask = value->valid_q_mask;
  }
  out->has_valid_dq_mask = value->has_valid_dq_mask;
  if (value->has_valid_dq_mask) {
    out->valid_dq_mask = value->valid_dq_mask;
  }
  out->has_valid_tau_mask = value->has_valid_tau_mask;
  if (value->has_valid_tau_mask) {
    out->valid_tau_mask = value->valid_tau_mask;
  }
  out->has_feedback_age_us = value->has_feedback_age_us;
  if (value->has_feedback_age_us) {
    memcpy(out->feedback_age_us, value->feedback_age_us, sizeof(out->feedback_age_us));
  }
  out->has_q_rad = value->has_q_rad;
  if (value->has_q_rad) {
    memcpy(out->q_rad, value->q_rad, sizeof(out->q_rad));
  }
  out->has_dq_rad_s = value->has_dq_rad_s;
  if (value->has_dq_rad_s) {
    memcpy(out->dq_rad_s, value->dq_rad_s, sizeof(out->dq_rad_s));
  }
  out->has_tau_measured_nm = value->has_tau_measured_nm;
  if (value->has_tau_measured_nm) {
    memcpy(out->tau_measured_nm, value->tau_measured_nm, sizeof(out->tau_measured_nm));
  }
  out->has_fault_code_raw = value->has_fault_code_raw;
  if (value->has_fault_code_raw) {
    memcpy(out->fault_code_raw, value->fault_code_raw, sizeof(out->fault_code_raw));
  }
  out->has_motor_mode_raw = value->has_motor_mode_raw;
  if (value->has_motor_mode_raw) {
    memcpy(out->motor_mode_raw, value->motor_mode_raw, sizeof(out->motor_mode_raw));
  }
  out->has_motor_fault_valid_mask = value->has_motor_fault_valid_mask;
  if (value->has_motor_fault_valid_mask) {
    out->motor_fault_valid_mask = value->motor_fault_valid_mask;
  }
  out->has_last_state_probe_sequence = value->has_last_state_probe_sequence;
  if (value->has_last_state_probe_sequence) {
    out->last_state_probe_sequence = value->last_state_probe_sequence;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t arm_state_value_to_view(const arm_state_value_t *value, arm_state_t *out) {
  arm_state_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = arm_state_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&arm_state_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t arm_state_value_encoded_size(const arm_state_value_t *value) {
  arm_state_t view;
  if (value == NULL || arm_state_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return arm_state_encoded_size(&view);
}

wl_codec_status_t arm_state_value_encode(const arm_state_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  arm_state_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = arm_state_value_borrow(value, &view);
  return status == WL_CODEC_OK ? arm_state_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t arm_state_value_decode(const uint8_t *input, size_t length, arm_state_value_t *out) {
  arm_state_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = arm_state_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  arm_state_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void hand_state_value_defaults(hand_state_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->last_applied_command_sequence = UINT32_C(0);
  out->command_age_us = UINT32_C(0);
  out->online = 0;
  out->valid_fields = UINT32_C(0);
  out->mode_raw = 0;
  out->fault_valid_mask = UINT32_C(0);
  out->tactile_capable = 0;
  out->tactile_valid = 0;
}

static void hand_state_value_copy_fields(const hand_state_t *view, hand_state_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_last_applied_command_sequence = view->has_last_applied_command_sequence;
  out->last_applied_command_sequence = view->has_last_applied_command_sequence ? view->last_applied_command_sequence : UINT32_C(0);
  out->has_command_age_us = view->has_command_age_us;
  out->command_age_us = view->has_command_age_us ? view->command_age_us : UINT32_C(0);
  out->has_online = view->has_online;
  out->online = view->has_online ? view->online : 0;
  out->has_valid_fields = view->has_valid_fields;
  out->valid_fields = view->has_valid_fields ? view->valid_fields : UINT32_C(0);
  out->has_field_age_us = view->has_field_age_us;
  if (view->has_field_age_us) memcpy(out->field_age_us, view->field_age_us, sizeof(out->field_age_us));
  out->has_motor_position_percent = view->has_motor_position_percent;
  if (view->has_motor_position_percent) memcpy(out->motor_position_percent, view->motor_position_percent, sizeof(out->motor_position_percent));
  out->has_joint_q_rad = view->has_joint_q_rad;
  if (view->has_joint_q_rad) memcpy(out->joint_q_rad, view->joint_q_rad, sizeof(out->joint_q_rad));
  out->has_motor_dq_rad_s = view->has_motor_dq_rad_s;
  if (view->has_motor_dq_rad_s) memcpy(out->motor_dq_rad_s, view->motor_dq_rad_s, sizeof(out->motor_dq_rad_s));
  out->has_motor_current_ma = view->has_motor_current_ma;
  if (view->has_motor_current_ma) memcpy(out->motor_current_ma, view->motor_current_ma, sizeof(out->motor_current_ma));
  out->has_fault_code_raw = view->has_fault_code_raw;
  if (view->has_fault_code_raw) memcpy(out->fault_code_raw, view->fault_code_raw, sizeof(out->fault_code_raw));
  out->has_mode_raw = view->has_mode_raw;
  out->mode_raw = view->has_mode_raw ? view->mode_raw : 0;
  out->has_fault_valid_mask = view->has_fault_valid_mask;
  out->fault_valid_mask = view->has_fault_valid_mask ? view->fault_valid_mask : UINT32_C(0);
  out->has_tactile_capable = view->has_tactile_capable;
  out->tactile_capable = view->has_tactile_capable ? view->tactile_capable : 0;
  out->has_tactile_valid = view->has_tactile_valid;
  out->tactile_valid = view->has_tactile_valid ? view->tactile_valid : 0;
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void hand_state_wlc_detail_value_copy(const hand_state_t *view, hand_state_value_t *out) {
  memset(out, 0, sizeof(*out));
  hand_state_value_copy_fields(view, out);
}

void hand_state_value_clear(hand_state_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  hand_state_value_defaults(value);
}

wl_codec_status_t hand_state_value_from_view(const hand_state_t *view, hand_state_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&hand_state_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  hand_state_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t hand_state_value_borrow(const hand_state_value_t *value, hand_state_t *out) {
  hand_state_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_last_applied_command_sequence = value->has_last_applied_command_sequence;
  if (value->has_last_applied_command_sequence) {
    out->last_applied_command_sequence = value->last_applied_command_sequence;
  }
  out->has_command_age_us = value->has_command_age_us;
  if (value->has_command_age_us) {
    out->command_age_us = value->command_age_us;
  }
  out->has_online = value->has_online;
  if (value->has_online) {
    out->online = value->online;
  }
  out->has_valid_fields = value->has_valid_fields;
  if (value->has_valid_fields) {
    out->valid_fields = value->valid_fields;
  }
  out->has_field_age_us = value->has_field_age_us;
  if (value->has_field_age_us) {
    memcpy(out->field_age_us, value->field_age_us, sizeof(out->field_age_us));
  }
  out->has_motor_position_percent = value->has_motor_position_percent;
  if (value->has_motor_position_percent) {
    memcpy(out->motor_position_percent, value->motor_position_percent, sizeof(out->motor_position_percent));
  }
  out->has_joint_q_rad = value->has_joint_q_rad;
  if (value->has_joint_q_rad) {
    memcpy(out->joint_q_rad, value->joint_q_rad, sizeof(out->joint_q_rad));
  }
  out->has_motor_dq_rad_s = value->has_motor_dq_rad_s;
  if (value->has_motor_dq_rad_s) {
    memcpy(out->motor_dq_rad_s, value->motor_dq_rad_s, sizeof(out->motor_dq_rad_s));
  }
  out->has_motor_current_ma = value->has_motor_current_ma;
  if (value->has_motor_current_ma) {
    memcpy(out->motor_current_ma, value->motor_current_ma, sizeof(out->motor_current_ma));
  }
  out->has_fault_code_raw = value->has_fault_code_raw;
  if (value->has_fault_code_raw) {
    memcpy(out->fault_code_raw, value->fault_code_raw, sizeof(out->fault_code_raw));
  }
  out->has_mode_raw = value->has_mode_raw;
  if (value->has_mode_raw) {
    out->mode_raw = value->mode_raw;
  }
  out->has_fault_valid_mask = value->has_fault_valid_mask;
  if (value->has_fault_valid_mask) {
    out->fault_valid_mask = value->fault_valid_mask;
  }
  out->has_tactile_capable = value->has_tactile_capable;
  if (value->has_tactile_capable) {
    out->tactile_capable = value->tactile_capable;
  }
  out->has_tactile_valid = value->has_tactile_valid;
  if (value->has_tactile_valid) {
    out->tactile_valid = value->tactile_valid;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t hand_state_value_to_view(const hand_state_value_t *value, hand_state_t *out) {
  hand_state_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = hand_state_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&hand_state_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t hand_state_value_encoded_size(const hand_state_value_t *value) {
  hand_state_t view;
  if (value == NULL || hand_state_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return hand_state_encoded_size(&view);
}

wl_codec_status_t hand_state_value_encode(const hand_state_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  hand_state_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = hand_state_value_borrow(value, &view);
  return status == WL_CODEC_OK ? hand_state_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t hand_state_value_decode(const uint8_t *input, size_t length, hand_state_value_t *out) {
  hand_state_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = hand_state_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  hand_state_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void fault_snapshot_value_defaults(fault_snapshot_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->policy_revision = UINT32_C(0);
  out->mcu_fault_active = UINT32_C(0);
  out->mcu_fault_latched = UINT32_C(0);
  out->mcu_fault_valid_mask = UINT32_C(0);
  out->local_action = 0;
  out->arm_fault_valid_mask = UINT32_C(0);
  out->hand_fault_valid_mask = UINT32_C(0);
  out->latest_event_id = UINT64_C(0);
  out->oldest_retained_event_id = UINT64_C(0);
  out->event_overflow_count = UINT32_C(0);
}

static void fault_snapshot_value_copy_fields(const fault_snapshot_t *view, fault_snapshot_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_policy_revision = view->has_policy_revision;
  out->policy_revision = view->has_policy_revision ? view->policy_revision : UINT32_C(0);
  out->has_mcu_fault_active = view->has_mcu_fault_active;
  out->mcu_fault_active = view->has_mcu_fault_active ? view->mcu_fault_active : UINT32_C(0);
  out->has_mcu_fault_latched = view->has_mcu_fault_latched;
  out->mcu_fault_latched = view->has_mcu_fault_latched ? view->mcu_fault_latched : UINT32_C(0);
  out->has_mcu_fault_valid_mask = view->has_mcu_fault_valid_mask;
  out->mcu_fault_valid_mask = view->has_mcu_fault_valid_mask ? view->mcu_fault_valid_mask : UINT32_C(0);
  out->has_local_action = view->has_local_action;
  out->local_action = view->has_local_action ? view->local_action : 0;
  out->has_arm_fault_raw = view->has_arm_fault_raw;
  if (view->has_arm_fault_raw) memcpy(out->arm_fault_raw, view->arm_fault_raw, sizeof(out->arm_fault_raw));
  out->has_arm_fault_valid_mask = view->has_arm_fault_valid_mask;
  out->arm_fault_valid_mask = view->has_arm_fault_valid_mask ? view->arm_fault_valid_mask : UINT32_C(0);
  out->has_hand_fault_raw = view->has_hand_fault_raw;
  if (view->has_hand_fault_raw) memcpy(out->hand_fault_raw, view->hand_fault_raw, sizeof(out->hand_fault_raw));
  out->has_hand_fault_valid_mask = view->has_hand_fault_valid_mask;
  out->hand_fault_valid_mask = view->has_hand_fault_valid_mask ? view->hand_fault_valid_mask : UINT32_C(0);
  out->has_latest_event_id = view->has_latest_event_id;
  out->latest_event_id = view->has_latest_event_id ? view->latest_event_id : UINT64_C(0);
  out->has_oldest_retained_event_id = view->has_oldest_retained_event_id;
  out->oldest_retained_event_id = view->has_oldest_retained_event_id ? view->oldest_retained_event_id : UINT64_C(0);
  out->has_event_overflow_count = view->has_event_overflow_count;
  out->event_overflow_count = view->has_event_overflow_count ? view->event_overflow_count : UINT32_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void fault_snapshot_wlc_detail_value_copy(const fault_snapshot_t *view, fault_snapshot_value_t *out) {
  memset(out, 0, sizeof(*out));
  fault_snapshot_value_copy_fields(view, out);
}

void fault_snapshot_value_clear(fault_snapshot_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  fault_snapshot_value_defaults(value);
}

wl_codec_status_t fault_snapshot_value_from_view(const fault_snapshot_t *view, fault_snapshot_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&fault_snapshot_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  fault_snapshot_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t fault_snapshot_value_borrow(const fault_snapshot_value_t *value, fault_snapshot_t *out) {
  fault_snapshot_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_policy_revision = value->has_policy_revision;
  if (value->has_policy_revision) {
    out->policy_revision = value->policy_revision;
  }
  out->has_mcu_fault_active = value->has_mcu_fault_active;
  if (value->has_mcu_fault_active) {
    out->mcu_fault_active = value->mcu_fault_active;
  }
  out->has_mcu_fault_latched = value->has_mcu_fault_latched;
  if (value->has_mcu_fault_latched) {
    out->mcu_fault_latched = value->mcu_fault_latched;
  }
  out->has_mcu_fault_valid_mask = value->has_mcu_fault_valid_mask;
  if (value->has_mcu_fault_valid_mask) {
    out->mcu_fault_valid_mask = value->mcu_fault_valid_mask;
  }
  out->has_local_action = value->has_local_action;
  if (value->has_local_action) {
    out->local_action = value->local_action;
  }
  out->has_arm_fault_raw = value->has_arm_fault_raw;
  if (value->has_arm_fault_raw) {
    memcpy(out->arm_fault_raw, value->arm_fault_raw, sizeof(out->arm_fault_raw));
  }
  out->has_arm_fault_valid_mask = value->has_arm_fault_valid_mask;
  if (value->has_arm_fault_valid_mask) {
    out->arm_fault_valid_mask = value->arm_fault_valid_mask;
  }
  out->has_hand_fault_raw = value->has_hand_fault_raw;
  if (value->has_hand_fault_raw) {
    memcpy(out->hand_fault_raw, value->hand_fault_raw, sizeof(out->hand_fault_raw));
  }
  out->has_hand_fault_valid_mask = value->has_hand_fault_valid_mask;
  if (value->has_hand_fault_valid_mask) {
    out->hand_fault_valid_mask = value->hand_fault_valid_mask;
  }
  out->has_latest_event_id = value->has_latest_event_id;
  if (value->has_latest_event_id) {
    out->latest_event_id = value->latest_event_id;
  }
  out->has_oldest_retained_event_id = value->has_oldest_retained_event_id;
  if (value->has_oldest_retained_event_id) {
    out->oldest_retained_event_id = value->oldest_retained_event_id;
  }
  out->has_event_overflow_count = value->has_event_overflow_count;
  if (value->has_event_overflow_count) {
    out->event_overflow_count = value->event_overflow_count;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t fault_snapshot_value_to_view(const fault_snapshot_value_t *value, fault_snapshot_t *out) {
  fault_snapshot_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = fault_snapshot_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&fault_snapshot_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t fault_snapshot_value_encoded_size(const fault_snapshot_value_t *value) {
  fault_snapshot_t view;
  if (value == NULL || fault_snapshot_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return fault_snapshot_encoded_size(&view);
}

wl_codec_status_t fault_snapshot_value_encode(const fault_snapshot_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  fault_snapshot_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = fault_snapshot_value_borrow(value, &view);
  return status == WL_CODEC_OK ? fault_snapshot_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t fault_snapshot_value_decode(const uint8_t *input, size_t length, fault_snapshot_value_t *out) {
  fault_snapshot_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = fault_snapshot_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  fault_snapshot_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void diagnostics_value_defaults(diagnostics_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->can_tx_completed = UINT32_C(0);
  out->can_tx_failed = UINT32_C(0);
  out->can_rx_valid = UINT32_C(0);
  out->can_rx_unknown = UINT32_C(0);
  out->can_rx_ambiguous = UINT32_C(0);
  out->can_rx_malformed = UINT32_C(0);
  out->can_bus_state = UINT32_C(0);
  out->can_tx_error_count = UINT32_C(0);
  out->can_rx_error_count = UINT32_C(0);
  out->udp_rx_valid = UINT32_C(0);
  out->udp_rx_bad = UINT32_C(0);
  out->udp_rx_old = UINT32_C(0);
  out->udp_tx_drop = UINT32_C(0);
  out->control_period_mean_us = UINT32_C(0);
  out->control_period_max_us = UINT32_C(0);
  out->control_jitter_abs_max_us = UINT32_C(0);
  out->command_coalesced = UINT32_C(0);
  out->counter_valid_mask = UINT32_C(0);
  out->state_probe_received = UINT32_C(0);
  out->last_state_probe_sequence = UINT32_C(0);
  out->hand_state_query_mode = 0;
  out->hand_mode_guard_phase = 0;
  out->hand_mode_guard_status = 0;
  out->hand_mode_guard_elapsed_us = UINT32_C(0);
  out->hand_mode_guard_token = UINT32_C(0);
  out->hand_control_tx = UINT32_C(0);
  out->hand_control_ack = UINT32_C(0);
  out->hand_query_timeouts = UINT32_C(0);
  out->hand_full_query_tx = UINT32_C(0);
  out->hand_control_negative_ack = UINT32_C(0);
  out->hand_control_ack_latest_status = UINT32_C(0);
}

static void diagnostics_value_copy_fields(const diagnostics_t *view, diagnostics_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_can_tx_completed = view->has_can_tx_completed;
  out->can_tx_completed = view->has_can_tx_completed ? view->can_tx_completed : UINT32_C(0);
  out->has_can_tx_failed = view->has_can_tx_failed;
  out->can_tx_failed = view->has_can_tx_failed ? view->can_tx_failed : UINT32_C(0);
  out->has_can_rx_valid = view->has_can_rx_valid;
  out->can_rx_valid = view->has_can_rx_valid ? view->can_rx_valid : UINT32_C(0);
  out->has_can_rx_unknown = view->has_can_rx_unknown;
  out->can_rx_unknown = view->has_can_rx_unknown ? view->can_rx_unknown : UINT32_C(0);
  out->has_can_rx_ambiguous = view->has_can_rx_ambiguous;
  out->can_rx_ambiguous = view->has_can_rx_ambiguous ? view->can_rx_ambiguous : UINT32_C(0);
  out->has_can_rx_malformed = view->has_can_rx_malformed;
  out->can_rx_malformed = view->has_can_rx_malformed ? view->can_rx_malformed : UINT32_C(0);
  out->has_can_bus_state = view->has_can_bus_state;
  out->can_bus_state = view->has_can_bus_state ? view->can_bus_state : UINT32_C(0);
  out->has_can_tx_error_count = view->has_can_tx_error_count;
  out->can_tx_error_count = view->has_can_tx_error_count ? view->can_tx_error_count : UINT32_C(0);
  out->has_can_rx_error_count = view->has_can_rx_error_count;
  out->can_rx_error_count = view->has_can_rx_error_count ? view->can_rx_error_count : UINT32_C(0);
  out->has_udp_rx_valid = view->has_udp_rx_valid;
  out->udp_rx_valid = view->has_udp_rx_valid ? view->udp_rx_valid : UINT32_C(0);
  out->has_udp_rx_bad = view->has_udp_rx_bad;
  out->udp_rx_bad = view->has_udp_rx_bad ? view->udp_rx_bad : UINT32_C(0);
  out->has_udp_rx_old = view->has_udp_rx_old;
  out->udp_rx_old = view->has_udp_rx_old ? view->udp_rx_old : UINT32_C(0);
  out->has_udp_tx_drop = view->has_udp_tx_drop;
  out->udp_tx_drop = view->has_udp_tx_drop ? view->udp_tx_drop : UINT32_C(0);
  out->has_control_period_mean_us = view->has_control_period_mean_us;
  out->control_period_mean_us = view->has_control_period_mean_us ? view->control_period_mean_us : UINT32_C(0);
  out->has_control_period_max_us = view->has_control_period_max_us;
  out->control_period_max_us = view->has_control_period_max_us ? view->control_period_max_us : UINT32_C(0);
  out->has_control_jitter_abs_max_us = view->has_control_jitter_abs_max_us;
  out->control_jitter_abs_max_us = view->has_control_jitter_abs_max_us ? view->control_jitter_abs_max_us : UINT32_C(0);
  out->has_command_coalesced = view->has_command_coalesced;
  out->command_coalesced = view->has_command_coalesced ? view->command_coalesced : UINT32_C(0);
  out->has_counter_valid_mask = view->has_counter_valid_mask;
  out->counter_valid_mask = view->has_counter_valid_mask ? view->counter_valid_mask : UINT32_C(0);
  out->has_motor_feedback_count = view->has_motor_feedback_count;
  if (view->has_motor_feedback_count) memcpy(out->motor_feedback_count, view->motor_feedback_count, sizeof(out->motor_feedback_count));
  out->has_hand_field_feedback_count = view->has_hand_field_feedback_count;
  if (view->has_hand_field_feedback_count) memcpy(out->hand_field_feedback_count, view->hand_field_feedback_count, sizeof(out->hand_field_feedback_count));
  out->has_state_probe_received = view->has_state_probe_received;
  out->state_probe_received = view->has_state_probe_received ? view->state_probe_received : UINT32_C(0);
  out->has_last_state_probe_sequence = view->has_last_state_probe_sequence;
  out->last_state_probe_sequence = view->has_last_state_probe_sequence ? view->last_state_probe_sequence : UINT32_C(0);
  out->has_hand_state_query_mode = view->has_hand_state_query_mode;
  out->hand_state_query_mode = view->has_hand_state_query_mode ? view->hand_state_query_mode : 0;
  out->has_hand_mode_guard_phase = view->has_hand_mode_guard_phase;
  out->hand_mode_guard_phase = view->has_hand_mode_guard_phase ? view->hand_mode_guard_phase : 0;
  out->has_hand_mode_guard_status = view->has_hand_mode_guard_status;
  out->hand_mode_guard_status = view->has_hand_mode_guard_status ? view->hand_mode_guard_status : 0;
  out->has_hand_mode_guard_elapsed_us = view->has_hand_mode_guard_elapsed_us;
  out->hand_mode_guard_elapsed_us = view->has_hand_mode_guard_elapsed_us ? view->hand_mode_guard_elapsed_us : UINT32_C(0);
  out->has_hand_mode_guard_token = view->has_hand_mode_guard_token;
  out->hand_mode_guard_token = view->has_hand_mode_guard_token ? view->hand_mode_guard_token : UINT32_C(0);
  out->has_hand_control_tx = view->has_hand_control_tx;
  out->hand_control_tx = view->has_hand_control_tx ? view->hand_control_tx : UINT32_C(0);
  out->has_hand_control_ack = view->has_hand_control_ack;
  out->hand_control_ack = view->has_hand_control_ack ? view->hand_control_ack : UINT32_C(0);
  out->has_hand_query_timeouts = view->has_hand_query_timeouts;
  out->hand_query_timeouts = view->has_hand_query_timeouts ? view->hand_query_timeouts : UINT32_C(0);
  out->has_motor_control_tx = view->has_motor_control_tx;
  if (view->has_motor_control_tx) memcpy(out->motor_control_tx, view->motor_control_tx, sizeof(out->motor_control_tx));
  out->has_hand_full_query_tx = view->has_hand_full_query_tx;
  out->hand_full_query_tx = view->has_hand_full_query_tx ? view->hand_full_query_tx : UINT32_C(0);
  out->has_hand_control_negative_ack = view->has_hand_control_negative_ack;
  out->hand_control_negative_ack = view->has_hand_control_negative_ack ? view->hand_control_negative_ack : UINT32_C(0);
  out->has_hand_control_ack_latest_status = view->has_hand_control_ack_latest_status;
  out->hand_control_ack_latest_status = view->has_hand_control_ack_latest_status ? view->hand_control_ack_latest_status : UINT32_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void diagnostics_wlc_detail_value_copy(const diagnostics_t *view, diagnostics_value_t *out) {
  memset(out, 0, sizeof(*out));
  diagnostics_value_copy_fields(view, out);
}

void diagnostics_value_clear(diagnostics_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  diagnostics_value_defaults(value);
}

wl_codec_status_t diagnostics_value_from_view(const diagnostics_t *view, diagnostics_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&diagnostics_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  diagnostics_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t diagnostics_value_borrow(const diagnostics_value_t *value, diagnostics_t *out) {
  diagnostics_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_can_tx_completed = value->has_can_tx_completed;
  if (value->has_can_tx_completed) {
    out->can_tx_completed = value->can_tx_completed;
  }
  out->has_can_tx_failed = value->has_can_tx_failed;
  if (value->has_can_tx_failed) {
    out->can_tx_failed = value->can_tx_failed;
  }
  out->has_can_rx_valid = value->has_can_rx_valid;
  if (value->has_can_rx_valid) {
    out->can_rx_valid = value->can_rx_valid;
  }
  out->has_can_rx_unknown = value->has_can_rx_unknown;
  if (value->has_can_rx_unknown) {
    out->can_rx_unknown = value->can_rx_unknown;
  }
  out->has_can_rx_ambiguous = value->has_can_rx_ambiguous;
  if (value->has_can_rx_ambiguous) {
    out->can_rx_ambiguous = value->can_rx_ambiguous;
  }
  out->has_can_rx_malformed = value->has_can_rx_malformed;
  if (value->has_can_rx_malformed) {
    out->can_rx_malformed = value->can_rx_malformed;
  }
  out->has_can_bus_state = value->has_can_bus_state;
  if (value->has_can_bus_state) {
    out->can_bus_state = value->can_bus_state;
  }
  out->has_can_tx_error_count = value->has_can_tx_error_count;
  if (value->has_can_tx_error_count) {
    out->can_tx_error_count = value->can_tx_error_count;
  }
  out->has_can_rx_error_count = value->has_can_rx_error_count;
  if (value->has_can_rx_error_count) {
    out->can_rx_error_count = value->can_rx_error_count;
  }
  out->has_udp_rx_valid = value->has_udp_rx_valid;
  if (value->has_udp_rx_valid) {
    out->udp_rx_valid = value->udp_rx_valid;
  }
  out->has_udp_rx_bad = value->has_udp_rx_bad;
  if (value->has_udp_rx_bad) {
    out->udp_rx_bad = value->udp_rx_bad;
  }
  out->has_udp_rx_old = value->has_udp_rx_old;
  if (value->has_udp_rx_old) {
    out->udp_rx_old = value->udp_rx_old;
  }
  out->has_udp_tx_drop = value->has_udp_tx_drop;
  if (value->has_udp_tx_drop) {
    out->udp_tx_drop = value->udp_tx_drop;
  }
  out->has_control_period_mean_us = value->has_control_period_mean_us;
  if (value->has_control_period_mean_us) {
    out->control_period_mean_us = value->control_period_mean_us;
  }
  out->has_control_period_max_us = value->has_control_period_max_us;
  if (value->has_control_period_max_us) {
    out->control_period_max_us = value->control_period_max_us;
  }
  out->has_control_jitter_abs_max_us = value->has_control_jitter_abs_max_us;
  if (value->has_control_jitter_abs_max_us) {
    out->control_jitter_abs_max_us = value->control_jitter_abs_max_us;
  }
  out->has_command_coalesced = value->has_command_coalesced;
  if (value->has_command_coalesced) {
    out->command_coalesced = value->command_coalesced;
  }
  out->has_counter_valid_mask = value->has_counter_valid_mask;
  if (value->has_counter_valid_mask) {
    out->counter_valid_mask = value->counter_valid_mask;
  }
  out->has_motor_feedback_count = value->has_motor_feedback_count;
  if (value->has_motor_feedback_count) {
    memcpy(out->motor_feedback_count, value->motor_feedback_count, sizeof(out->motor_feedback_count));
  }
  out->has_hand_field_feedback_count = value->has_hand_field_feedback_count;
  if (value->has_hand_field_feedback_count) {
    memcpy(out->hand_field_feedback_count, value->hand_field_feedback_count, sizeof(out->hand_field_feedback_count));
  }
  out->has_state_probe_received = value->has_state_probe_received;
  if (value->has_state_probe_received) {
    out->state_probe_received = value->state_probe_received;
  }
  out->has_last_state_probe_sequence = value->has_last_state_probe_sequence;
  if (value->has_last_state_probe_sequence) {
    out->last_state_probe_sequence = value->last_state_probe_sequence;
  }
  out->has_hand_state_query_mode = value->has_hand_state_query_mode;
  if (value->has_hand_state_query_mode) {
    out->hand_state_query_mode = value->hand_state_query_mode;
  }
  out->has_hand_mode_guard_phase = value->has_hand_mode_guard_phase;
  if (value->has_hand_mode_guard_phase) {
    out->hand_mode_guard_phase = value->hand_mode_guard_phase;
  }
  out->has_hand_mode_guard_status = value->has_hand_mode_guard_status;
  if (value->has_hand_mode_guard_status) {
    out->hand_mode_guard_status = value->hand_mode_guard_status;
  }
  out->has_hand_mode_guard_elapsed_us = value->has_hand_mode_guard_elapsed_us;
  if (value->has_hand_mode_guard_elapsed_us) {
    out->hand_mode_guard_elapsed_us = value->hand_mode_guard_elapsed_us;
  }
  out->has_hand_mode_guard_token = value->has_hand_mode_guard_token;
  if (value->has_hand_mode_guard_token) {
    out->hand_mode_guard_token = value->hand_mode_guard_token;
  }
  out->has_hand_control_tx = value->has_hand_control_tx;
  if (value->has_hand_control_tx) {
    out->hand_control_tx = value->hand_control_tx;
  }
  out->has_hand_control_ack = value->has_hand_control_ack;
  if (value->has_hand_control_ack) {
    out->hand_control_ack = value->hand_control_ack;
  }
  out->has_hand_query_timeouts = value->has_hand_query_timeouts;
  if (value->has_hand_query_timeouts) {
    out->hand_query_timeouts = value->hand_query_timeouts;
  }
  out->has_motor_control_tx = value->has_motor_control_tx;
  if (value->has_motor_control_tx) {
    memcpy(out->motor_control_tx, value->motor_control_tx, sizeof(out->motor_control_tx));
  }
  out->has_hand_full_query_tx = value->has_hand_full_query_tx;
  if (value->has_hand_full_query_tx) {
    out->hand_full_query_tx = value->hand_full_query_tx;
  }
  out->has_hand_control_negative_ack = value->has_hand_control_negative_ack;
  if (value->has_hand_control_negative_ack) {
    out->hand_control_negative_ack = value->hand_control_negative_ack;
  }
  out->has_hand_control_ack_latest_status = value->has_hand_control_ack_latest_status;
  if (value->has_hand_control_ack_latest_status) {
    out->hand_control_ack_latest_status = value->hand_control_ack_latest_status;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t diagnostics_value_to_view(const diagnostics_value_t *value, diagnostics_t *out) {
  diagnostics_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = diagnostics_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&diagnostics_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t diagnostics_value_encoded_size(const diagnostics_value_t *value) {
  diagnostics_t view;
  if (value == NULL || diagnostics_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return diagnostics_encoded_size(&view);
}

wl_codec_status_t diagnostics_value_encode(const diagnostics_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  diagnostics_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = diagnostics_value_borrow(value, &view);
  return status == WL_CODEC_OK ? diagnostics_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t diagnostics_value_decode(const uint8_t *input, size_t length, diagnostics_value_t *out) {
  diagnostics_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = diagnostics_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  diagnostics_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void fault_event_value_defaults(fault_event_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->event_id = UINT64_C(0);
  out->device_class = 0;
  out->device_id = 0;
  out->device_channel = 0;
  out->raw_code = UINT32_C(0);
  out->normalized_code = UINT32_C(0);
  out->severity = 0;
  out->active = 0;
  out->local_action = 0;
  out->policy_revision = UINT32_C(0);
  out->first_seen_us = UINT64_C(0);
  out->last_seen_us = UINT64_C(0);
  out->occurrences = UINT32_C(0);
}

static void fault_event_value_copy_fields(const fault_event_t *view, fault_event_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_event_id = view->has_event_id;
  out->event_id = view->has_event_id ? view->event_id : UINT64_C(0);
  out->has_device_class = view->has_device_class;
  out->device_class = view->has_device_class ? view->device_class : 0;
  out->has_device_id = view->has_device_id;
  out->device_id = view->has_device_id ? view->device_id : 0;
  out->has_device_channel = view->has_device_channel;
  out->device_channel = view->has_device_channel ? view->device_channel : 0;
  out->has_raw_code = view->has_raw_code;
  out->raw_code = view->has_raw_code ? view->raw_code : UINT32_C(0);
  out->has_normalized_code = view->has_normalized_code;
  out->normalized_code = view->has_normalized_code ? view->normalized_code : UINT32_C(0);
  out->has_severity = view->has_severity;
  out->severity = view->has_severity ? view->severity : 0;
  out->has_active = view->has_active;
  out->active = view->has_active ? view->active : 0;
  out->has_local_action = view->has_local_action;
  out->local_action = view->has_local_action ? view->local_action : 0;
  out->has_policy_revision = view->has_policy_revision;
  out->policy_revision = view->has_policy_revision ? view->policy_revision : UINT32_C(0);
  out->has_first_seen_us = view->has_first_seen_us;
  out->first_seen_us = view->has_first_seen_us ? view->first_seen_us : UINT64_C(0);
  out->has_last_seen_us = view->has_last_seen_us;
  out->last_seen_us = view->has_last_seen_us ? view->last_seen_us : UINT64_C(0);
  out->has_occurrences = view->has_occurrences;
  out->occurrences = view->has_occurrences ? view->occurrences : UINT32_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void fault_event_wlc_detail_value_copy(const fault_event_t *view, fault_event_value_t *out) {
  memset(out, 0, sizeof(*out));
  fault_event_value_copy_fields(view, out);
}

void fault_event_value_clear(fault_event_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  fault_event_value_defaults(value);
}

wl_codec_status_t fault_event_value_from_view(const fault_event_t *view, fault_event_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&fault_event_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  fault_event_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t fault_event_value_borrow(const fault_event_value_t *value, fault_event_t *out) {
  fault_event_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_event_id = value->has_event_id;
  if (value->has_event_id) {
    out->event_id = value->event_id;
  }
  out->has_device_class = value->has_device_class;
  if (value->has_device_class) {
    out->device_class = value->device_class;
  }
  out->has_device_id = value->has_device_id;
  if (value->has_device_id) {
    out->device_id = value->device_id;
  }
  out->has_device_channel = value->has_device_channel;
  if (value->has_device_channel) {
    out->device_channel = value->device_channel;
  }
  out->has_raw_code = value->has_raw_code;
  if (value->has_raw_code) {
    out->raw_code = value->raw_code;
  }
  out->has_normalized_code = value->has_normalized_code;
  if (value->has_normalized_code) {
    out->normalized_code = value->normalized_code;
  }
  out->has_severity = value->has_severity;
  if (value->has_severity) {
    out->severity = value->severity;
  }
  out->has_active = value->has_active;
  if (value->has_active) {
    out->active = value->active;
  }
  out->has_local_action = value->has_local_action;
  if (value->has_local_action) {
    out->local_action = value->local_action;
  }
  out->has_policy_revision = value->has_policy_revision;
  if (value->has_policy_revision) {
    out->policy_revision = value->policy_revision;
  }
  out->has_first_seen_us = value->has_first_seen_us;
  if (value->has_first_seen_us) {
    out->first_seen_us = value->first_seen_us;
  }
  out->has_last_seen_us = value->has_last_seen_us;
  if (value->has_last_seen_us) {
    out->last_seen_us = value->last_seen_us;
  }
  out->has_occurrences = value->has_occurrences;
  if (value->has_occurrences) {
    out->occurrences = value->occurrences;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t fault_event_value_to_view(const fault_event_value_t *value, fault_event_t *out) {
  fault_event_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = fault_event_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&fault_event_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t fault_event_value_encoded_size(const fault_event_value_t *value) {
  fault_event_t view;
  if (value == NULL || fault_event_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return fault_event_encoded_size(&view);
}

wl_codec_status_t fault_event_value_encode(const fault_event_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  fault_event_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = fault_event_value_borrow(value, &view);
  return status == WL_CODEC_OK ? fault_event_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t fault_event_value_decode(const uint8_t *input, size_t length, fault_event_value_t *out) {
  fault_event_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = fault_event_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  fault_event_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void operation_request_value_defaults(operation_request_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->operation_id = UINT32_C(0);
  out->operation = 0;
  out->control_lease = UINT64_C(0);
  out->expected_policy_revision = UINT32_C(0);
  out->target_mask = UINT32_C(0);
  out->fault_event_id = UINT64_C(0);
}

static void operation_request_value_copy_fields(const operation_request_t *view, operation_request_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_operation_id = view->has_operation_id;
  out->operation_id = view->has_operation_id ? view->operation_id : UINT32_C(0);
  out->has_operation = view->has_operation;
  out->operation = view->has_operation ? view->operation : 0;
  out->has_control_lease = view->has_control_lease;
  out->control_lease = view->has_control_lease ? view->control_lease : UINT64_C(0);
  out->has_expected_policy_revision = view->has_expected_policy_revision;
  out->expected_policy_revision = view->has_expected_policy_revision ? view->expected_policy_revision : UINT32_C(0);
  out->has_target_mask = view->has_target_mask;
  out->target_mask = view->has_target_mask ? view->target_mask : UINT32_C(0);
  out->has_fault_event_id = view->has_fault_event_id;
  out->fault_event_id = view->has_fault_event_id ? view->fault_event_id : UINT64_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void operation_request_wlc_detail_value_copy(const operation_request_t *view, operation_request_value_t *out) {
  memset(out, 0, sizeof(*out));
  operation_request_value_copy_fields(view, out);
}

void operation_request_value_clear(operation_request_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  operation_request_value_defaults(value);
}

wl_codec_status_t operation_request_value_from_view(const operation_request_t *view, operation_request_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&operation_request_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  operation_request_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t operation_request_value_borrow(const operation_request_value_t *value, operation_request_t *out) {
  operation_request_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_operation_id = value->has_operation_id;
  if (value->has_operation_id) {
    out->operation_id = value->operation_id;
  }
  out->has_operation = value->has_operation;
  if (value->has_operation) {
    out->operation = value->operation;
  }
  out->has_control_lease = value->has_control_lease;
  if (value->has_control_lease) {
    out->control_lease = value->control_lease;
  }
  out->has_expected_policy_revision = value->has_expected_policy_revision;
  if (value->has_expected_policy_revision) {
    out->expected_policy_revision = value->expected_policy_revision;
  }
  out->has_target_mask = value->has_target_mask;
  if (value->has_target_mask) {
    out->target_mask = value->target_mask;
  }
  out->has_fault_event_id = value->has_fault_event_id;
  if (value->has_fault_event_id) {
    out->fault_event_id = value->fault_event_id;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t operation_request_value_to_view(const operation_request_value_t *value, operation_request_t *out) {
  operation_request_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = operation_request_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&operation_request_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t operation_request_value_encoded_size(const operation_request_value_t *value) {
  operation_request_t view;
  if (value == NULL || operation_request_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return operation_request_encoded_size(&view);
}

wl_codec_status_t operation_request_value_encode(const operation_request_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  operation_request_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = operation_request_value_borrow(value, &view);
  return status == WL_CODEC_OK ? operation_request_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t operation_request_value_decode(const uint8_t *input, size_t length, operation_request_value_t *out) {
  operation_request_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = operation_request_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  operation_request_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void operation_result_value_defaults(operation_result_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->operation_id = UINT32_C(0);
  out->status = 0;
  out->reason = 0;
  out->observed_run_state = 0;
  out->applied_policy_revision = UINT32_C(0);
  out->control_lease = UINT64_C(0);
}

static void operation_result_value_copy_fields(const operation_result_t *view, operation_result_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_operation_id = view->has_operation_id;
  out->operation_id = view->has_operation_id ? view->operation_id : UINT32_C(0);
  out->has_status = view->has_status;
  out->status = view->has_status ? view->status : 0;
  out->has_reason = view->has_reason;
  out->reason = view->has_reason ? view->reason : 0;
  out->has_observed_run_state = view->has_observed_run_state;
  out->observed_run_state = view->has_observed_run_state ? view->observed_run_state : 0;
  out->has_applied_policy_revision = view->has_applied_policy_revision;
  out->applied_policy_revision = view->has_applied_policy_revision ? view->applied_policy_revision : UINT32_C(0);
  out->has_control_lease = view->has_control_lease;
  out->control_lease = view->has_control_lease ? view->control_lease : UINT64_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void operation_result_wlc_detail_value_copy(const operation_result_t *view, operation_result_value_t *out) {
  memset(out, 0, sizeof(*out));
  operation_result_value_copy_fields(view, out);
}

void operation_result_value_clear(operation_result_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  operation_result_value_defaults(value);
}

wl_codec_status_t operation_result_value_from_view(const operation_result_t *view, operation_result_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&operation_result_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  operation_result_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t operation_result_value_borrow(const operation_result_value_t *value, operation_result_t *out) {
  operation_result_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_operation_id = value->has_operation_id;
  if (value->has_operation_id) {
    out->operation_id = value->operation_id;
  }
  out->has_status = value->has_status;
  if (value->has_status) {
    out->status = value->status;
  }
  out->has_reason = value->has_reason;
  if (value->has_reason) {
    out->reason = value->reason;
  }
  out->has_observed_run_state = value->has_observed_run_state;
  if (value->has_observed_run_state) {
    out->observed_run_state = value->observed_run_state;
  }
  out->has_applied_policy_revision = value->has_applied_policy_revision;
  if (value->has_applied_policy_revision) {
    out->applied_policy_revision = value->applied_policy_revision;
  }
  out->has_control_lease = value->has_control_lease;
  if (value->has_control_lease) {
    out->control_lease = value->control_lease;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t operation_result_value_to_view(const operation_result_value_t *value, operation_result_t *out) {
  operation_result_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = operation_result_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&operation_result_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t operation_result_value_encoded_size(const operation_result_value_t *value) {
  operation_result_t view;
  if (value == NULL || operation_result_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return operation_result_encoded_size(&view);
}

wl_codec_status_t operation_result_value_encode(const operation_result_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  operation_result_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = operation_result_value_borrow(value, &view);
  return status == WL_CODEC_OK ? operation_result_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t operation_result_value_decode(const uint8_t *input, size_t length, operation_result_value_t *out) {
  operation_result_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = operation_result_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  operation_result_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void configure_request_value_defaults(configure_request_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->operation_id = UINT32_C(0);
  out->expected_policy_revision = UINT32_C(0);
  out->new_policy_revision = UINT32_C(0);
  out->gravity_owner = 0;
  out->friction_owner = 0;
  out->disconnect_action = 0;
  out->teleop_mode = 0;
  out->arm_command_timeout_us = UINT32_C(0);
  out->hand_command_timeout_us = UINT32_C(0);
  out->arm_feedback_timeout_us = UINT32_C(0);
  out->hand_feedback_timeout_us = UINT32_C(0);
  out->hand_mode = 0;
  out->hand_current_semantics = 0;
  out->local_fallback_enabled = 0;
  out->model_revision = UINT32_C(0);
  out->host_managed_fault_mask = UINT32_C(0);
  out->hard_protection_profile_id = UINT32_C(0);
  out->max_command_valid_for_us = UINT32_C(0);
  out->arm_state_period_us = UINT32_C(0);
  out->hand_state_period_us = UINT32_C(0);
  out->hand_control_feedforward_ma = UINT32_C(0);
}

static void configure_request_value_copy_fields(const configure_request_t *view, configure_request_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_operation_id = view->has_operation_id;
  out->operation_id = view->has_operation_id ? view->operation_id : UINT32_C(0);
  out->has_expected_policy_revision = view->has_expected_policy_revision;
  out->expected_policy_revision = view->has_expected_policy_revision ? view->expected_policy_revision : UINT32_C(0);
  out->has_new_policy_revision = view->has_new_policy_revision;
  out->new_policy_revision = view->has_new_policy_revision ? view->new_policy_revision : UINT32_C(0);
  out->has_gravity_owner = view->has_gravity_owner;
  out->gravity_owner = view->has_gravity_owner ? view->gravity_owner : 0;
  out->has_friction_owner = view->has_friction_owner;
  out->friction_owner = view->has_friction_owner ? view->friction_owner : 0;
  out->has_disconnect_action = view->has_disconnect_action;
  out->disconnect_action = view->has_disconnect_action ? view->disconnect_action : 0;
  out->has_teleop_mode = view->has_teleop_mode;
  out->teleop_mode = view->has_teleop_mode ? view->teleop_mode : 0;
  out->has_arm_command_timeout_us = view->has_arm_command_timeout_us;
  out->arm_command_timeout_us = view->has_arm_command_timeout_us ? view->arm_command_timeout_us : UINT32_C(0);
  out->has_hand_command_timeout_us = view->has_hand_command_timeout_us;
  out->hand_command_timeout_us = view->has_hand_command_timeout_us ? view->hand_command_timeout_us : UINT32_C(0);
  out->has_arm_feedback_timeout_us = view->has_arm_feedback_timeout_us;
  out->arm_feedback_timeout_us = view->has_arm_feedback_timeout_us ? view->arm_feedback_timeout_us : UINT32_C(0);
  out->has_hand_feedback_timeout_us = view->has_hand_feedback_timeout_us;
  out->hand_feedback_timeout_us = view->has_hand_feedback_timeout_us ? view->hand_feedback_timeout_us : UINT32_C(0);
  out->has_hand_mode = view->has_hand_mode;
  out->hand_mode = view->has_hand_mode ? view->hand_mode : 0;
  out->has_hand_soft_current_ma = view->has_hand_soft_current_ma;
  if (view->has_hand_soft_current_ma) memcpy(out->hand_soft_current_ma, view->hand_soft_current_ma, sizeof(out->hand_soft_current_ma));
  out->has_hand_current_semantics = view->has_hand_current_semantics;
  out->hand_current_semantics = view->has_hand_current_semantics ? view->hand_current_semantics : 0;
  out->has_local_fallback_enabled = view->has_local_fallback_enabled;
  out->local_fallback_enabled = view->has_local_fallback_enabled ? view->local_fallback_enabled : 0;
  out->has_model_revision = view->has_model_revision;
  out->model_revision = view->has_model_revision ? view->model_revision : UINT32_C(0);
  out->has_host_managed_fault_mask = view->has_host_managed_fault_mask;
  out->host_managed_fault_mask = view->has_host_managed_fault_mask ? view->host_managed_fault_mask : UINT32_C(0);
  out->has_hard_protection_profile_id = view->has_hard_protection_profile_id;
  out->hard_protection_profile_id = view->has_hard_protection_profile_id ? view->hard_protection_profile_id : UINT32_C(0);
  out->has_max_command_valid_for_us = view->has_max_command_valid_for_us;
  out->max_command_valid_for_us = view->has_max_command_valid_for_us ? view->max_command_valid_for_us : UINT32_C(0);
  out->has_force_feedback_gain = view->has_force_feedback_gain;
  if (view->has_force_feedback_gain) memcpy(out->force_feedback_gain, view->force_feedback_gain, sizeof(out->force_feedback_gain));
  out->has_arm_state_period_us = view->has_arm_state_period_us;
  out->arm_state_period_us = view->has_arm_state_period_us ? view->arm_state_period_us : UINT32_C(0);
  out->has_hand_state_period_us = view->has_hand_state_period_us;
  out->hand_state_period_us = view->has_hand_state_period_us ? view->hand_state_period_us : UINT32_C(0);
  out->has_hand_control_feedforward_ma = view->has_hand_control_feedforward_ma;
  out->hand_control_feedforward_ma = view->has_hand_control_feedforward_ma ? view->hand_control_feedforward_ma : UINT32_C(0);
  out->has_hand_position_current_limit_ma = view->has_hand_position_current_limit_ma;
  if (view->has_hand_position_current_limit_ma) memcpy(out->hand_position_current_limit_ma, view->hand_position_current_limit_ma, sizeof(out->hand_position_current_limit_ma));
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void configure_request_wlc_detail_value_copy(const configure_request_t *view, configure_request_value_t *out) {
  memset(out, 0, sizeof(*out));
  configure_request_value_copy_fields(view, out);
}

void configure_request_value_clear(configure_request_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  configure_request_value_defaults(value);
}

wl_codec_status_t configure_request_value_from_view(const configure_request_t *view, configure_request_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&configure_request_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  configure_request_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t configure_request_value_borrow(const configure_request_value_t *value, configure_request_t *out) {
  configure_request_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_operation_id = value->has_operation_id;
  if (value->has_operation_id) {
    out->operation_id = value->operation_id;
  }
  out->has_expected_policy_revision = value->has_expected_policy_revision;
  if (value->has_expected_policy_revision) {
    out->expected_policy_revision = value->expected_policy_revision;
  }
  out->has_new_policy_revision = value->has_new_policy_revision;
  if (value->has_new_policy_revision) {
    out->new_policy_revision = value->new_policy_revision;
  }
  out->has_gravity_owner = value->has_gravity_owner;
  if (value->has_gravity_owner) {
    out->gravity_owner = value->gravity_owner;
  }
  out->has_friction_owner = value->has_friction_owner;
  if (value->has_friction_owner) {
    out->friction_owner = value->friction_owner;
  }
  out->has_disconnect_action = value->has_disconnect_action;
  if (value->has_disconnect_action) {
    out->disconnect_action = value->disconnect_action;
  }
  out->has_teleop_mode = value->has_teleop_mode;
  if (value->has_teleop_mode) {
    out->teleop_mode = value->teleop_mode;
  }
  out->has_arm_command_timeout_us = value->has_arm_command_timeout_us;
  if (value->has_arm_command_timeout_us) {
    out->arm_command_timeout_us = value->arm_command_timeout_us;
  }
  out->has_hand_command_timeout_us = value->has_hand_command_timeout_us;
  if (value->has_hand_command_timeout_us) {
    out->hand_command_timeout_us = value->hand_command_timeout_us;
  }
  out->has_arm_feedback_timeout_us = value->has_arm_feedback_timeout_us;
  if (value->has_arm_feedback_timeout_us) {
    out->arm_feedback_timeout_us = value->arm_feedback_timeout_us;
  }
  out->has_hand_feedback_timeout_us = value->has_hand_feedback_timeout_us;
  if (value->has_hand_feedback_timeout_us) {
    out->hand_feedback_timeout_us = value->hand_feedback_timeout_us;
  }
  out->has_hand_mode = value->has_hand_mode;
  if (value->has_hand_mode) {
    out->hand_mode = value->hand_mode;
  }
  out->has_hand_soft_current_ma = value->has_hand_soft_current_ma;
  if (value->has_hand_soft_current_ma) {
    memcpy(out->hand_soft_current_ma, value->hand_soft_current_ma, sizeof(out->hand_soft_current_ma));
  }
  out->has_hand_current_semantics = value->has_hand_current_semantics;
  if (value->has_hand_current_semantics) {
    out->hand_current_semantics = value->hand_current_semantics;
  }
  out->has_local_fallback_enabled = value->has_local_fallback_enabled;
  if (value->has_local_fallback_enabled) {
    out->local_fallback_enabled = value->local_fallback_enabled;
  }
  out->has_model_revision = value->has_model_revision;
  if (value->has_model_revision) {
    out->model_revision = value->model_revision;
  }
  out->has_host_managed_fault_mask = value->has_host_managed_fault_mask;
  if (value->has_host_managed_fault_mask) {
    out->host_managed_fault_mask = value->host_managed_fault_mask;
  }
  out->has_hard_protection_profile_id = value->has_hard_protection_profile_id;
  if (value->has_hard_protection_profile_id) {
    out->hard_protection_profile_id = value->hard_protection_profile_id;
  }
  out->has_max_command_valid_for_us = value->has_max_command_valid_for_us;
  if (value->has_max_command_valid_for_us) {
    out->max_command_valid_for_us = value->max_command_valid_for_us;
  }
  out->has_force_feedback_gain = value->has_force_feedback_gain;
  if (value->has_force_feedback_gain) {
    memcpy(out->force_feedback_gain, value->force_feedback_gain, sizeof(out->force_feedback_gain));
  }
  out->has_arm_state_period_us = value->has_arm_state_period_us;
  if (value->has_arm_state_period_us) {
    out->arm_state_period_us = value->arm_state_period_us;
  }
  out->has_hand_state_period_us = value->has_hand_state_period_us;
  if (value->has_hand_state_period_us) {
    out->hand_state_period_us = value->hand_state_period_us;
  }
  out->has_hand_control_feedforward_ma = value->has_hand_control_feedforward_ma;
  if (value->has_hand_control_feedforward_ma) {
    out->hand_control_feedforward_ma = value->hand_control_feedforward_ma;
  }
  out->has_hand_position_current_limit_ma = value->has_hand_position_current_limit_ma;
  if (value->has_hand_position_current_limit_ma) {
    memcpy(out->hand_position_current_limit_ma, value->hand_position_current_limit_ma, sizeof(out->hand_position_current_limit_ma));
  }
  return WL_CODEC_OK;
}

wl_codec_status_t configure_request_value_to_view(const configure_request_value_t *value, configure_request_t *out) {
  configure_request_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = configure_request_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&configure_request_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t configure_request_value_encoded_size(const configure_request_value_t *value) {
  configure_request_t view;
  if (value == NULL || configure_request_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return configure_request_encoded_size(&view);
}

wl_codec_status_t configure_request_value_encode(const configure_request_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  configure_request_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = configure_request_value_borrow(value, &view);
  return status == WL_CODEC_OK ? configure_request_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t configure_request_value_decode(const uint8_t *input, size_t length, configure_request_value_t *out) {
  configure_request_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = configure_request_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  configure_request_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void configure_result_value_defaults(configure_result_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->operation_id = UINT32_C(0);
  out->status = 0;
  out->reason = 0;
  out->applied_policy_revision = UINT32_C(0);
  out->hard_protection_mask = UINT32_C(0);
  out->capabilities = UINT32_C(0);
}

static void configure_result_value_copy_fields(const configure_result_t *view, configure_result_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_operation_id = view->has_operation_id;
  out->operation_id = view->has_operation_id ? view->operation_id : UINT32_C(0);
  out->has_status = view->has_status;
  out->status = view->has_status ? view->status : 0;
  out->has_reason = view->has_reason;
  out->reason = view->has_reason ? view->reason : 0;
  out->has_applied_policy_revision = view->has_applied_policy_revision;
  out->applied_policy_revision = view->has_applied_policy_revision ? view->applied_policy_revision : UINT32_C(0);
  out->has_hard_protection_mask = view->has_hard_protection_mask;
  out->hard_protection_mask = view->has_hard_protection_mask ? view->hard_protection_mask : UINT32_C(0);
  out->has_capabilities = view->has_capabilities;
  out->capabilities = view->has_capabilities ? view->capabilities : UINT32_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void configure_result_wlc_detail_value_copy(const configure_result_t *view, configure_result_value_t *out) {
  memset(out, 0, sizeof(*out));
  configure_result_value_copy_fields(view, out);
}

void configure_result_value_clear(configure_result_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  configure_result_value_defaults(value);
}

wl_codec_status_t configure_result_value_from_view(const configure_result_t *view, configure_result_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&configure_result_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  configure_result_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t configure_result_value_borrow(const configure_result_value_t *value, configure_result_t *out) {
  configure_result_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_operation_id = value->has_operation_id;
  if (value->has_operation_id) {
    out->operation_id = value->operation_id;
  }
  out->has_status = value->has_status;
  if (value->has_status) {
    out->status = value->status;
  }
  out->has_reason = value->has_reason;
  if (value->has_reason) {
    out->reason = value->reason;
  }
  out->has_applied_policy_revision = value->has_applied_policy_revision;
  if (value->has_applied_policy_revision) {
    out->applied_policy_revision = value->applied_policy_revision;
  }
  out->has_hard_protection_mask = value->has_hard_protection_mask;
  if (value->has_hard_protection_mask) {
    out->hard_protection_mask = value->hard_protection_mask;
  }
  out->has_capabilities = value->has_capabilities;
  if (value->has_capabilities) {
    out->capabilities = value->capabilities;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t configure_result_value_to_view(const configure_result_value_t *value, configure_result_t *out) {
  configure_result_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = configure_result_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&configure_result_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t configure_result_value_encoded_size(const configure_result_value_t *value) {
  configure_result_t view;
  if (value == NULL || configure_result_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return configure_result_encoded_size(&view);
}

wl_codec_status_t configure_result_value_encode(const configure_result_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  configure_result_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = configure_result_value_borrow(value, &view);
  return status == WL_CODEC_OK ? configure_result_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t configure_result_value_decode(const uint8_t *input, size_t length, configure_result_value_t *out) {
  configure_result_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = configure_result_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  configure_result_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

static void peer_arm_state_value_defaults(peer_arm_state_value_t *out) {
  (void)out;
  stream_header_value_defaults(&out->header);
  out->pairing_session = UINT64_C(0);
  out->policy_revision = UINT32_C(0);
  out->role = 0;
  out->teleop_mode = 0;
  out->valid_q_mask = UINT32_C(0);
  out->valid_dq_mask = UINT32_C(0);
  out->valid_residual_tau_mask = UINT32_C(0);
  out->critical_fault_mask = UINT32_C(0);
}

static void peer_arm_state_value_copy_fields(const peer_arm_state_t *view, peer_arm_state_value_t *out) {
  out->has_header = view->has_header;
  if (view->has_header) stream_header_value_copy_fields(&view->header, &out->header);
  else stream_header_value_defaults(&out->header);
  out->has_pairing_session = view->has_pairing_session;
  out->pairing_session = view->has_pairing_session ? view->pairing_session : UINT64_C(0);
  out->has_policy_revision = view->has_policy_revision;
  out->policy_revision = view->has_policy_revision ? view->policy_revision : UINT32_C(0);
  out->has_role = view->has_role;
  out->role = view->has_role ? view->role : 0;
  out->has_teleop_mode = view->has_teleop_mode;
  out->teleop_mode = view->has_teleop_mode ? view->teleop_mode : 0;
  out->has_valid_q_mask = view->has_valid_q_mask;
  out->valid_q_mask = view->has_valid_q_mask ? view->valid_q_mask : UINT32_C(0);
  out->has_valid_dq_mask = view->has_valid_dq_mask;
  out->valid_dq_mask = view->has_valid_dq_mask ? view->valid_dq_mask : UINT32_C(0);
  out->has_valid_residual_tau_mask = view->has_valid_residual_tau_mask;
  out->valid_residual_tau_mask = view->has_valid_residual_tau_mask ? view->valid_residual_tau_mask : UINT32_C(0);
  out->has_feedback_age_us = view->has_feedback_age_us;
  if (view->has_feedback_age_us) memcpy(out->feedback_age_us, view->feedback_age_us, sizeof(out->feedback_age_us));
  out->has_q_rad = view->has_q_rad;
  if (view->has_q_rad) memcpy(out->q_rad, view->q_rad, sizeof(out->q_rad));
  out->has_dq_rad_s = view->has_dq_rad_s;
  if (view->has_dq_rad_s) memcpy(out->dq_rad_s, view->dq_rad_s, sizeof(out->dq_rad_s));
  out->has_residual_tau_nm = view->has_residual_tau_nm;
  if (view->has_residual_tau_nm) memcpy(out->residual_tau_nm, view->residual_tau_nm, sizeof(out->residual_tau_nm));
  out->has_critical_fault_mask = view->has_critical_fault_mask;
  out->critical_fault_mask = view->has_critical_fault_mask ? view->critical_fault_mask : UINT32_C(0);
}

/* Generator-private: input is an unmodified successful decode, or has been measured. */
void peer_arm_state_wlc_detail_value_copy(const peer_arm_state_t *view, peer_arm_state_value_t *out) {
  memset(out, 0, sizeof(*out));
  peer_arm_state_value_copy_fields(view, out);
}

void peer_arm_state_value_clear(peer_arm_state_value_t *value) {
  if (value == NULL) return;
  memset(value, 0, sizeof(*value));
  peer_arm_state_value_defaults(value);
}

wl_codec_status_t peer_arm_state_value_from_view(const peer_arm_state_t *view, peer_arm_state_value_t *out) {
  size_t size;
  wl_codec_status_t status;
  if (view == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = wlc_measure(&peer_arm_state_desc, view, &size);
  if (status != WL_CODEC_OK) return status;
  peer_arm_state_wlc_detail_value_copy(view, out);
  return WL_CODEC_OK;
}

/* Private conversion does not re-validate each nested subtree. */
static wl_codec_status_t peer_arm_state_value_borrow(const peer_arm_state_value_t *value, peer_arm_state_t *out) {
  peer_arm_state_clear(out);
  out->has_header = value->has_header;
  if (value->has_header) {
    wl_codec_status_t status = stream_header_value_borrow(&value->header, &out->header);
    if (status != WL_CODEC_OK) return status;
  }
  out->has_pairing_session = value->has_pairing_session;
  if (value->has_pairing_session) {
    out->pairing_session = value->pairing_session;
  }
  out->has_policy_revision = value->has_policy_revision;
  if (value->has_policy_revision) {
    out->policy_revision = value->policy_revision;
  }
  out->has_role = value->has_role;
  if (value->has_role) {
    out->role = value->role;
  }
  out->has_teleop_mode = value->has_teleop_mode;
  if (value->has_teleop_mode) {
    out->teleop_mode = value->teleop_mode;
  }
  out->has_valid_q_mask = value->has_valid_q_mask;
  if (value->has_valid_q_mask) {
    out->valid_q_mask = value->valid_q_mask;
  }
  out->has_valid_dq_mask = value->has_valid_dq_mask;
  if (value->has_valid_dq_mask) {
    out->valid_dq_mask = value->valid_dq_mask;
  }
  out->has_valid_residual_tau_mask = value->has_valid_residual_tau_mask;
  if (value->has_valid_residual_tau_mask) {
    out->valid_residual_tau_mask = value->valid_residual_tau_mask;
  }
  out->has_feedback_age_us = value->has_feedback_age_us;
  if (value->has_feedback_age_us) {
    memcpy(out->feedback_age_us, value->feedback_age_us, sizeof(out->feedback_age_us));
  }
  out->has_q_rad = value->has_q_rad;
  if (value->has_q_rad) {
    memcpy(out->q_rad, value->q_rad, sizeof(out->q_rad));
  }
  out->has_dq_rad_s = value->has_dq_rad_s;
  if (value->has_dq_rad_s) {
    memcpy(out->dq_rad_s, value->dq_rad_s, sizeof(out->dq_rad_s));
  }
  out->has_residual_tau_nm = value->has_residual_tau_nm;
  if (value->has_residual_tau_nm) {
    memcpy(out->residual_tau_nm, value->residual_tau_nm, sizeof(out->residual_tau_nm));
  }
  out->has_critical_fault_mask = value->has_critical_fault_mask;
  if (value->has_critical_fault_mask) {
    out->critical_fault_mask = value->critical_fault_mask;
  }
  return WL_CODEC_OK;
}

wl_codec_status_t peer_arm_state_value_to_view(const peer_arm_state_value_t *value, peer_arm_state_t *out) {
  peer_arm_state_t view;
  size_t size;
  wl_codec_status_t status;
  if (value == NULL || out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = peer_arm_state_value_borrow(value, &view);
  if (status == WL_CODEC_OK) status = wlc_measure(&peer_arm_state_desc, &view, &size);
  if (status != WL_CODEC_OK) return status;
  *out = view;
  return WL_CODEC_OK;
}

size_t peer_arm_state_value_encoded_size(const peer_arm_state_value_t *value) {
  peer_arm_state_t view;
  if (value == NULL || peer_arm_state_value_borrow(value, &view) != WL_CODEC_OK) return SIZE_MAX;
  return peer_arm_state_encoded_size(&view);
}

wl_codec_status_t peer_arm_state_value_encode(const peer_arm_state_value_t *value, uint8_t *out, size_t capacity, size_t *length) {
  peer_arm_state_t view;
  wl_codec_status_t status;
  if (value == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = peer_arm_state_value_borrow(value, &view);
  return status == WL_CODEC_OK ? peer_arm_state_encode(&view, out, capacity, length) : status;
}

wl_codec_status_t peer_arm_state_value_decode(const uint8_t *input, size_t length, peer_arm_state_value_t *out) {
  peer_arm_state_t view;
  wl_codec_status_t status;
  if (out == NULL) return WL_CODEC_ERR_INVALID_VALUE;
  status = peer_arm_state_decode(input, length, &view);
  if (status != WL_CODEC_OK) return status;
  peer_arm_state_wlc_detail_value_copy(&view, out);
  return WL_CODEC_OK;
}

