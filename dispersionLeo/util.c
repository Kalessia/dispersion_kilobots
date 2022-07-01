/* TODO
 */

#include <kilombo.h>
#ifdef SIMULATOR
#include <limits.h>
#else
#include <avr/io.h>
#endif

#include "util.h"



/* Helper function for setting motor speed smoothly
*/
void smooth_set_motors(uint8_t ccw, uint8_t cw) {
    // OCR2A = ccw;  OCR2B = cw;
#ifdef KILOBOT
    uint8_t l = 0, r = 0;
    if (ccw && !OCR2A) // we want left motor on, and it's off
        l = 0xff;
    if (cw && !OCR2B)  // we want right motor on, and it's off
        r = 0xff;
    if (l || r)        // at least one motor needs spin-up
    {
        set_motors(l, r);
        delay(15);
    }
#endif
    // spin-up is done, now we set the real value
    set_motors(ccw, cw);
}


void set_motion(motion_t new_motion) {
    switch(new_motion) {
        case STOP:
            smooth_set_motors(0,0);
            break;
        case FORWARD:
            smooth_set_motors(kilo_straight_left, kilo_straight_right);
            break;
        case LEFT:
            smooth_set_motors(kilo_turn_left, 0);
            break;
        case RIGHT:
            smooth_set_motors(0, kilo_turn_right);
            break;
    }
}

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}

message_t *message_tx() {
    return &mydata->msg;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
