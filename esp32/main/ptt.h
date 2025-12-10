#ifndef PTT_H
#define PTT_H

#include <inttypes.h>

// Initializes ptt in and output pins as well as internal timers
void ptt_init();

// Returns ptt level
uint8_t get_ptt();

// Activates ptt and starts timer to turn it off
void key_ptt();

// Checks whether ptt has just been disabled since last call
uint8_t was_ptt_disabled();

#endif