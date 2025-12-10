#ifndef TX_H
#define TX_H

extern const double TRANSMISSION_TIME_MS;
extern const double BIT_RATE;
extern const double BAUD_RATE;

#define DAC_AMPLITUDE_V         0.5    // Must be within [0, 3.3]
#define MIN_DAC_V               1.5    // band aid solution to dac voltage being clipped when ptt is high; 2 * dac amplitude + dac v must be < 5

void tx_init();

// Wake an asynch task to send gps
void tx_gps();

#endif