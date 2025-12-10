#ifndef AUTO_TX_H
#define AUTO_TX_H

#include <inttypes.h>

// Build and configure internal auto transmission timer
void auto_tx_init();

void reset_auto_tx_timer();

uint8_t was_auto_tx_flagged();

#endif