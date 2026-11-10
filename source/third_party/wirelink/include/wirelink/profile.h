/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_PROFILE_H_
#define INCLUDE_WIRELINK_PROFILE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Link-profile selectors shared by the link API and framing utilities. */
typedef int32_t wl_envelope_type_t;
enum {
  WL_ENVELOPE_COBS_STREAM = 0,
  WL_ENVELOPE_NATIVE_PACKET = 1,
  WL_ENVELOPE_BUS_LENGTH16 = 2,
};

typedef int32_t wl_integrity_t;
enum {
  WL_INTEGRITY_NONE = 0,
  WL_INTEGRITY_CRC16 = 1,
  WL_INTEGRITY_CRC32C = 2,
  WL_INTEGRITY_CRC32 = WL_INTEGRITY_CRC32C,
};

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_PROFILE_H_ */
