/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WIRELINK_SRC_UNIT_RX_H_
#define WIRELINK_SRC_UNIT_RX_H_

#include "context.h"

wl_span_t wl_rx_unit_consumer_peek(wl_ctx_t *ctx);
int wl_rx_unit_consumer_has_data(const wl_ctx_t *ctx);
int wl_rx_unit_consumer_consume(wl_ctx_t *ctx);

#endif /* WIRELINK_SRC_UNIT_RX_H_ */
