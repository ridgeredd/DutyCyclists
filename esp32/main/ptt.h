#ifndef PTT_H
#define PTT_H

#include <inttypes.h>

#define PTT_INTERRUPT_ENABLE    1

// Initializes ptt in and output pins as well as internal timers
void ptt_init();

// Returns ptt level as updated by isr
uint8_t get_ptt();

// Gets current value via pin
uint8_t poll_ptt();

// Activates ptt and starts timer to turn it off
void key_ptt();

// Same as above but for longer periods of time for auto tx.
void key_ptt_long();

// Checks whether ptt has just been disabled since last call
uint8_t was_ptt_disabled();

#endif