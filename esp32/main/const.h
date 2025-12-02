#ifndef CONST_H
#define CONST_H

#include <stdint.h>

//====================================================================================================================================
//                                                    NAMED VALUES     
//====================================================================================================================================

#define PI  3.141592653589793f

#define CALL_SIGN   0x69   // Unique id for this device
#define START_STOP  0x7E   // Starting and stopping byte of transmission

const char *TAG = "gps_tx";

//====================================================================================================================================
//                                                              GPIO
//====================================================================================================================================

#define PTT_INPUT_PIN           GPIO_NUM_4      // Pin monitoring PTT to identify edges    
#define PTT_OUTPUT_PIN          GPIO_NUM_5      // Pin driving PTT to key radio

#define PTT_ACTIVE_LVL          0               // active high (1) or low (0). Always use active low
#define PTT_INACTIVE_LVL        !PTT_ACTIVE_LVL

#if PTT_ACTIVE_LVL
    #define PTT_PRESS_INTR      EDGE_RISING     // Press is rising edge     
    #define PTT_RELEASE_INTR    EDGE_FALLING    // Release is falling edge
#else
    #define PTT_PRESS_INTR      EDGE_FALLING    // Press is falling edge
    #define PTT_RELEASE_INTR    EDGE_RISING     // Release is rising edge
#endif   

//====================================================================================================================================
//                                                            TIMING
//====================================================================================================================================

#define TIMER_DELAY_MS      10000   // Milliseconds before automatically resending signal
#define DEBOUNCE_MS         100     // Ignore isr requests within some recent amount of time to debounce signal
#define PTT_BUFFER_MS       10      // Amount of additional time to hold PTT high after transmission length
#define MIN_INTERVAL_MS     2 * TRANSMISSION_TIME_MS   // Don't allow transmissions within 2 * min interval of eachother 

//====================================================================================================================================
//                                                          TRANSMISSION
//====================================================================================================================================

#define AUTOMATIC_TRANSMISSION  0   // enable (1) disable (0) automatic transmission

#define TRANSMISSION_BYTES      PAYLOAD_BYTES + PARITY_BYTES + 3        // start + bytes sent + call sign + stop
#define PAYLOAD_BYTES           10                      
#define BUF_LEN                 256                // frames per DAC buffer. 2 * 256 because stereo

#define SYMBOL_MASK             (NSYMBOLS - 1u)

#define TRANSMISSION_TIME_MS    (1000 * SAMPLES_PER_SYMBOL * 8 * TRANSMISSION_BYTES) / (BITS_PER_SYMBOL * SAMPLE_RATE)

#define BAUD_RATE               SAMPLE_RATE / SAMPLES_PER_SYMBOL
#define BIT_RATE                BAUD_RATE * BITS_PER_SYMBOL

//====================================================================================================================================
//                                                            SYMBOLS
//====================================================================================================================================

#define NSYMBOLS                (1u << BITS_PER_SYMBOL)
#define BITS_PER_SYMBOL         1           // power of 2; will cause error if > 8 because using uint8
#define SAMPLES_PER_SYMBOL      100

static const float SYMBOL_FREQS[NSYMBOLS]   = { 1200.0f, 2200.0f }; // symbol frequencies in order of bit index

//====================================================================================================================================
//                                                              DAC
//====================================================================================================================================

#define DAC_AMPLITUDE_V     1.0    // Amplitude of wave. 2 * Amplitude + Min Voltage must be within [0, 3.3]
#define MIN_DAC_V           0.0    // Minimum voltage of wave output. Equivalent to DC Bias - DAC Amplitude
                                   // Band aid solution to dac voltage being clipped when ptt is high
#define SAMPLE_RATE         48000  // DAC sample rate

//====================================================================================================================================
//                                                         REED SOLOMON
//====================================================================================================================================

#define PARITY_BYTES            6    // These must sum to < 255

#define CODEWORD_BITS           4    // how many bits are combined for RS code alphabet
#define REDUNDANT_BITS          4    // how many bits are appended for error correction

//====================================================================================================================================

#endif