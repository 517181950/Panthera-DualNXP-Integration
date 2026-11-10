#ifndef INCLUDE_WIRELINK_SPAN_H_
#define INCLUDE_WIRELINK_SPAN_H_

#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct wl_span {
  uint8_t *data;
  size_t length;
} wl_span_t;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // INCLUDE_WIRELINK_SPAN_H_
