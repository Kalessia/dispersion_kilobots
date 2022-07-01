
#ifndef UTIL_H_
#define UTIL_H_

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif
#include "dispersion.h"

// declare motion variable type
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

void smooth_set_motors(uint8_t ccw, uint8_t cw);

void set_motion(motion_t new_motion);

void rxbuffer_push(message_t *msg, distance_measurement_t *dist);

message_t *message_tx();


// ---------------------------------------------- {{{1
// Ring buffer operations. Taken from kilolib's ringbuffer.h
// but adapted for use with mydata->

// Ring buffer operations indexed with head, tail
// These waste one entry in the buffer, but are interrupt safe:
//   * head is changed only in popfront
//   * tail is changed only in pushback
//   * RB_popfront() is to be called AFTER the data in RB_front() has been used
//   * head and tail indices are uint8_t, which can be updated atomically
//     - still, the updates need to be atomic, especially in RB_popfront()

#define RB_init() {	\
    mydata->RXHead = 0; \
    mydata->RXTail = 0;\
}

#define RB_empty() (mydata->RXHead == mydata->RXTail)

#define RB_full()  ((mydata->RXHead+1)%RB_SIZE == mydata->RXTail)

#define RB_front() mydata->RXBuffer[mydata->RXHead]

#define RB_back() mydata->RXBuffer[mydata->RXTail]

#define RB_popfront() mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;


#define RB_pushback() {\
    mydata->RXTail = (mydata->RXTail+1)%RB_SIZE;\
    if (RB_empty())\
      { mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;	\
    /*printf("Full.\n"); */}			\
  }

/*
#define RB_pushback() {\
    mydata->RXTail = (mydata->RXTail+1)%RB_SIZE;\
    if (RB_empty())\
      { mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;	\
  }
*/

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
