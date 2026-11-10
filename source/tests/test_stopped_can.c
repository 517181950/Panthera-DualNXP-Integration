#include "devices/stopped_can.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    struct stopped_can s={0};
    stopped_can_update(&s,true);
    /* Zero-initialized telemetry and old/stale mode-zero values cannot ack. */
    assert(stopped_can_action(&s,0,1000,0,0,false)==STOPPED_CAN_STOP);
    assert(stopped_can_action(&s,0,1000,999,0,false)==STOPPED_CAN_STOP);
    stopped_can_sent(&s,0,1000);
    assert(stopped_can_action(&s,0,2000,2000,0,false)==STOPPED_CAN_STOP);
    assert(stopped_can_action(&s,0,30000,900,0,false)==STOPPED_CAN_STOP);
    assert(stopped_can_action(&s,0,40000,21000,0,false)==STOPPED_CAN_STOP);
    assert(stopped_can_action(&s,0,21000,22000,0,false)==STOPPED_CAN_STOP);
    assert(stopped_can_action(&s,0,22000,21000,0,false)==STOPPED_CAN_QUIET);
    /* Confirmed stopped, no stream: remain quiet even during later power-off. */
    assert(stopped_can_action(&s,0,10000000,21000,0,false)==STOPPED_CAN_QUIET);
    /* Host preparation can request fresh read-only state without reasserting STOP. */
    assert(stopped_can_action(&s,0,10000000,21000,0,true)==STOPPED_CAN_QUERY);
    /* An unconfirmed second motor must still receive STOP. */
    assert(stopped_can_action(&s,1,22000,21000,0,false)==STOPPED_CAN_STOP);
    /* A newly observed nonzero mode immediately resumes STOP. */
    assert(stopped_can_action(&s,0,10000001,10000001,21,true)==STOPPED_CAN_STOP);
    assert(stopped_can_action(&s,0,10000002,10000002,0,false)==STOPPED_CAN_STOP);
    stopped_can_sent(&s,0,10000003);
    assert(stopped_can_action(&s,0,10021003,10021003,0,false)==STOPPED_CAN_QUIET);
    /* Leaving stop (e.g. homing/soft hold) clears confirmation; a later fault
     * cannot inherit an acknowledgement from before an enabled movement. */
    stopped_can_update(&s,false);
    assert(s.confirmed_mask==0 && s.sent_mask==0);
    stopped_can_update(&s,true);
    assert(stopped_can_action(&s,0,10021004,10021003,0,false)==STOPPED_CAN_STOP);
    for (unsigned j=0;j<6;j++) {
        stopped_can_sent(&s,j,20000000);
        assert(stopped_can_action(&s,j,20022000,20021000,0,false)==STOPPED_CAN_QUIET);
    }
    assert(s.confirmed_mask==63);
    puts("STOP acknowledgement, independent axes, quiet power cycle, poll restart and fresh fault checks passed");
}
