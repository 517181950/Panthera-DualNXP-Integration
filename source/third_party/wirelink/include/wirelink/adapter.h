/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_ADAPTER_H_
#define INCLUDE_WIRELINK_ADAPTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Transport-independent counters for telemetry, tests, and HIL acceptance. */
typedef struct wl_adapter_stats {
  uint64_t rx_units;
  uint64_t rx_bytes;
  uint64_t rx_backpressure;
  uint64_t tx_units;
  uint64_t tx_bytes;
  uint64_t tx_completions;
  uint64_t activity_notifications;
  uint64_t service_calls;
  uint64_t errors;
  uint8_t started;
  uint8_t rx_paused;
  uint8_t tx_active;
} wl_adapter_stats_t;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_ADAPTER_H_ */
