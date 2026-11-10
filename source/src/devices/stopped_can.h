#pragma once
#include <stdbool.h>
#include <stdint.h>

enum stopped_can_action { STOPPED_CAN_QUIET, STOPPED_CAN_QUERY, STOPPED_CAN_STOP };
struct stopped_can {
    bool stopping;
    uint8_t sent_mask, confirmed_mask;
    uint64_t first_sent[6];
};
/* Stop for at least 20 ms, then require a fresh, explicit mode-zero reply.
 * Confirmed-off axes may remain quiet across drive power cycles. This never
 * applies to an active homing/hold/teleoperation output. */
void stopped_can_update(struct stopped_can *s, bool stopping);
enum stopped_can_action stopped_can_action(struct stopped_can *s, unsigned joint,
    uint64_t now, uint64_t mode_seen, uint32_t mode, bool requested_poll);
void stopped_can_sent(struct stopped_can *s, unsigned joint, uint64_t now);
