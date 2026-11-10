/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_BULK_H_
#define INCLUDE_WIRELINK_BULK_H_

#include <stddef.h>
#include <stdint.h>

#include "wirelink/link.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bulk runtime errors are deliberately disjoint from wl_err_t. */
typedef int32_t wl_bulk_err_t;
enum {
  WL_BULK_OK = 0,
  WL_BULK_ERR_INVALID_ARG = -2000,
  WL_BULK_ERR_NOT_INITIALIZED = -2001,
  WL_BULK_ERR_BUSY = -2002,
  WL_BULK_ERR_INVALID_STATE = -2003,
  WL_BULK_ERR_NOT_FOUND = -2004,
  WL_BULK_ERR_PROTOCOL = -2005,
  WL_BULK_ERR_TIMEOUT = -2006,
};

const char *wl_bulk_err_str(wl_bulk_err_t error);

typedef int32_t wl_bulk_phase_t;
enum {
  WL_BULK_PHASE_NONE = 0,
  WL_BULK_PHASE_BEGIN,
  WL_BULK_PHASE_CHUNK,
  WL_BULK_PHASE_END,
  WL_BULK_PHASE_ABORT,
};

/* Values carried by the application Status message. */
typedef int32_t wl_bulk_status_code_t;
enum {
  WL_BULK_STATUS_OK = 0,
  WL_BULK_STATUS_BUSY,
  WL_BULK_STATUS_OUT_OF_ORDER,
  WL_BULK_STATUS_CONFLICT,
  WL_BULK_STATUS_INVALID,
  WL_BULK_STATUS_WRITE_FAILED,
  WL_BULK_STATUS_INTEGRITY_FAILED,
  WL_BULK_STATUS_ABORTED,
  WL_BULK_STATUS_TIMED_OUT,
};

typedef struct wl_bulk_descriptor {
  /* Fresh and nonzero for each logical transfer while peer history may live. */
  uint32_t transfer_id;
  uint64_t total_length;
  uint32_t requested_chunk_size;
  uint32_t object_crc32c;
} wl_bulk_descriptor_t;

typedef struct wl_bulk_chunk {
  uint32_t transfer_id;
  uint64_t offset;
  const uint8_t *data;
  size_t length;
} wl_bulk_chunk_t;

typedef struct wl_bulk_status {
  uint32_t transfer_id;
  wl_bulk_phase_t phase;
  wl_bulk_status_code_t code;
  uint64_t next_offset;
  uint32_t accepted_chunk_size;
} wl_bulk_status_t;

typedef int32_t wl_bulk_sink_result_t;
enum {
  WL_BULK_SINK_OK = 0,
  /* The callback consumed no bytes and made no durable state change. */
  WL_BULK_SINK_BUSY,
  WL_BULK_SINK_WRITE_FAILED,
  WL_BULK_SINK_INTEGRITY_FAILED,
  WL_BULK_SINK_INVALID,
};

typedef struct wl_bulk_sink {
  void *user_data;
  /* Return the persisted prefix that may be resumed. */
  wl_bulk_sink_result_t (*begin)(void *user_data,
                                 const wl_bulk_descriptor_t *descriptor,
                                 uint64_t *out_resume_offset);
  /* data is borrowed and may not be retained after this callback returns. */
  wl_bulk_sink_result_t (*write)(void *user_data, uint32_t transfer_id,
                                 uint64_t offset, const uint8_t *data,
                                 size_t length);
  /* Verify the bytes actually stored and commit exactly once on success. */
  wl_bulk_sink_result_t (*finish)(void *user_data,
                                  const wl_bulk_descriptor_t *descriptor);
  void (*abort)(void *user_data, uint32_t transfer_id, int32_t reason);
} wl_bulk_sink_t;

typedef int32_t wl_bulk_receiver_state_t;
enum {
  WL_BULK_RECEIVER_IDLE = 0,
  WL_BULK_RECEIVER_RECEIVING,
  WL_BULK_RECEIVER_COMPLETED,
  WL_BULK_RECEIVER_FAILED,
  WL_BULK_RECEIVER_ABORTED,
};

#define WL_BULK_RECEIVER_STORAGE_SIZE 256U
typedef union wl_bulk_receiver {
  wl_max_align_t align;
  uint8_t private_bytes[WL_BULK_RECEIVER_STORAGE_SIZE];
} wl_bulk_receiver_t;

typedef struct wl_bulk_receiver_config {
  uint64_t max_object_length;
  uint32_t max_chunk_size;
  /* Nonzero power of two. The final chunk may be shorter. */
  uint32_t write_alignment;
  /* Zero disables inactivity expiry; otherwise it must be below 2^31 ms. */
  uint32_t idle_timeout_ms;
  wl_bulk_sink_t sink;
} wl_bulk_receiver_config_t;

typedef struct wl_bulk_receiver_status_view {
  wl_bulk_status_t status;
  uint32_t token;
} wl_bulk_receiver_status_view_t;

typedef struct wl_bulk_receiver_stats {
  uint32_t begins;
  uint32_t chunks;
  uint32_t bytes_written;
  uint32_t duplicate_messages;
  uint32_t busy_responses;
  uint32_t protocol_errors;
  uint32_t write_failures;
  uint32_t integrity_failures;
  uint32_t aborts;
  uint32_t timeouts;
} wl_bulk_receiver_stats_t;

wl_bulk_err_t wl_bulk_receiver_init(wl_bulk_receiver_t *receiver,
                                    const wl_bulk_receiver_config_t *config);

/*
 * reset is externally serialized and requires no acquired Status view. An
 * active sink is aborted before its receiver state is discarded.
 */
wl_bulk_err_t wl_bulk_receiver_reset(wl_bulk_receiver_t *receiver);

wl_bulk_err_t wl_bulk_receiver_on_begin(wl_bulk_receiver_t *receiver,
                                        const wl_bulk_descriptor_t *descriptor,
                                        wl_time_ms_t now_ms);
wl_bulk_err_t wl_bulk_receiver_on_chunk(wl_bulk_receiver_t *receiver,
                                        const wl_bulk_chunk_t *chunk,
                                        wl_time_ms_t now_ms);
wl_bulk_err_t wl_bulk_receiver_on_end(wl_bulk_receiver_t *receiver,
                                      uint32_t transfer_id,
                                      uint64_t total_length,
                                      uint32_t object_crc32c,
                                      wl_time_ms_t now_ms);
wl_bulk_err_t wl_bulk_receiver_on_abort(wl_bulk_receiver_t *receiver,
                                        uint32_t transfer_id, int32_t reason,
                                        wl_time_ms_t now_ms);

/*
 * Only one Status is retained. New input returns WL_BULK_ERR_BUSY until the
 * application publishes and releases it. Defer keeps the Status pending for a
 * later transport retry; release consumes it after local TX acceptance.
 */
wl_bulk_err_t
wl_bulk_receiver_status_acquire(wl_bulk_receiver_t *receiver,
                                wl_bulk_receiver_status_view_t *out_view);
wl_bulk_err_t
wl_bulk_receiver_status_defer(wl_bulk_receiver_t *receiver,
                              const wl_bulk_receiver_status_view_t *view);
wl_bulk_err_t
wl_bulk_receiver_status_release(wl_bulk_receiver_t *receiver,
                                const wl_bulk_receiver_status_view_t *view);

wl_bulk_err_t wl_bulk_receiver_poll(wl_bulk_receiver_t *receiver,
                                    wl_time_ms_t now_ms);

#define WL_BULK_NO_DEADLINE_MS UINT32_MAX
typedef struct wl_bulk_deadline_hint {
  uint32_t next_deadline_ms;
} wl_bulk_deadline_hint_t;

wl_bulk_err_t
wl_bulk_receiver_get_deadline_hint(const wl_bulk_receiver_t *receiver,
                                   wl_time_ms_t now_ms,
                                   wl_bulk_deadline_hint_t *out_hint);
wl_bulk_err_t wl_bulk_receiver_get_state(const wl_bulk_receiver_t *receiver,
                                         wl_bulk_receiver_state_t *out_state,
                                         uint64_t *out_next_offset);
wl_bulk_err_t wl_bulk_receiver_get_stats(const wl_bulk_receiver_t *receiver,
                                         wl_bulk_receiver_stats_t *out_stats);

typedef int32_t wl_bulk_sender_state_t;
enum {
  WL_BULK_SENDER_IDLE = 0,
  WL_BULK_SENDER_READY,
  WL_BULK_SENDER_WAIT_STATUS,
  WL_BULK_SENDER_COMPLETED,
  WL_BULK_SENDER_FAILED,
  WL_BULK_SENDER_ABORTED,
};

#define WL_BULK_SENDER_STORAGE_SIZE 256U
typedef union wl_bulk_sender {
  wl_max_align_t align;
  uint8_t private_bytes[WL_BULK_SENDER_STORAGE_SIZE];
} wl_bulk_sender_t;

typedef struct wl_bulk_sender_config {
  /* Both delays must be nonzero and below 2^31 ms. */
  uint32_t status_timeout_ms;
  uint32_t busy_retry_ms;
  uint16_t max_retries;
} wl_bulk_sender_config_t;

/*
 * Actions contain only stable metadata. For CHUNK, the caller reads length
 * bytes from its repeatable source at offset while encoding the generated
 * application message. The sender never retains that source span.
 */
typedef struct wl_bulk_sender_action {
  wl_bulk_phase_t phase;
  wl_bulk_descriptor_t descriptor;
  uint64_t offset;
  size_t length;
  int32_t abort_reason;
  uint32_t token;
} wl_bulk_sender_action_t;

typedef struct wl_bulk_sender_stats {
  uint32_t starts;
  uint32_t actions_submitted;
  uint32_t statuses_received;
  uint32_t retries;
  uint32_t busy_responses;
  uint32_t protocol_errors;
  uint32_t completed;
  uint32_t failed;
  uint32_t aborted;
} wl_bulk_sender_stats_t;

typedef struct wl_bulk_sender_result {
  wl_bulk_sender_state_t state;
  wl_bulk_status_code_t status;
  uint64_t next_offset;
  uint16_t retry_count;
} wl_bulk_sender_result_t;

wl_bulk_err_t wl_bulk_sender_init(wl_bulk_sender_t *sender,
                                  const wl_bulk_sender_config_t *config);
/*
 * Active senders and non-Abort failures return BUSY. Request Abort and reach a
 * terminal state first. A failed Abort has already exhausted bounded cleanup
 * retries and may be reset as an explicit force-abandon operation.
 */
wl_bulk_err_t wl_bulk_sender_reset(wl_bulk_sender_t *sender);
wl_bulk_err_t wl_bulk_sender_start(wl_bulk_sender_t *sender,
                                   const wl_bulk_descriptor_t *descriptor);
wl_bulk_err_t wl_bulk_sender_request_abort(wl_bulk_sender_t *sender,
                                           int32_t reason);

wl_bulk_err_t
wl_bulk_sender_action_acquire(wl_bulk_sender_t *sender,
                              wl_bulk_sender_action_t *out_action);
wl_bulk_err_t
wl_bulk_sender_action_defer(wl_bulk_sender_t *sender,
                            const wl_bulk_sender_action_t *action);
wl_bulk_err_t
wl_bulk_sender_action_submitted(wl_bulk_sender_t *sender,
                                const wl_bulk_sender_action_t *action,
                                wl_time_ms_t now_ms);

wl_bulk_err_t wl_bulk_sender_on_status(wl_bulk_sender_t *sender,
                                       const wl_bulk_status_t *status,
                                       wl_time_ms_t now_ms);
wl_bulk_err_t wl_bulk_sender_poll(wl_bulk_sender_t *sender,
                                  wl_time_ms_t now_ms);
wl_bulk_err_t
wl_bulk_sender_get_deadline_hint(const wl_bulk_sender_t *sender,
                                 wl_time_ms_t now_ms,
                                 wl_bulk_deadline_hint_t *out_hint);
wl_bulk_err_t wl_bulk_sender_get_result(const wl_bulk_sender_t *sender,
                                        wl_bulk_sender_result_t *out_result);
wl_bulk_err_t wl_bulk_sender_get_stats(const wl_bulk_sender_t *sender,
                                       wl_bulk_sender_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WIRELINK_BULK_H_ */
