#include "stopped_can.h"
#include <string.h>

void stopped_can_update(struct stopped_can *s, bool stopping) {
    if (!stopping || !s->stopping) memset(s, 0, sizeof(*s));
    s->stopping = stopping;
}
enum stopped_can_action stopped_can_action(struct stopped_can *s, unsigned j,
    uint64_t now, uint64_t seen, uint32_t mode, bool requested_poll) {
    if (j >= 6 || !s->stopping) return STOPPED_CAN_QUIET;
    uint8_t bit = 1u << j;
    bool fresh = seen && now >= seen && now - seen < 10000;
    if (fresh && mode != 0) {
        s->confirmed_mask &= ~bit;
        s->sent_mask &= ~bit;
    }
    if (!(s->confirmed_mask & bit) && (s->sent_mask & bit) && fresh && mode == 0
            && seen >= s->first_sent[j] && seen - s->first_sent[j] >= 20000)
        s->confirmed_mask |= bit;
    if (!(s->confirmed_mask & bit)) return STOPPED_CAN_STOP;
    return requested_poll ? STOPPED_CAN_QUERY : STOPPED_CAN_QUIET;
}
void stopped_can_sent(struct stopped_can *s, unsigned j, uint64_t now) {
    if (!s->stopping || j >= 6) return;
    uint8_t bit = 1u << j;
    if (!(s->sent_mask & bit)) s->first_sent[j] = now;
    s->sent_mask |= bit;
}
