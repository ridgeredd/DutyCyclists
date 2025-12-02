#include "fsm.h"
#include "gpio.h"

typedef enum {
    BOOTUP = 0,
    IDLE,           // No transmission
                    // Not necessarily needed to have this state, but decreases chances of faulty gps transmission during voice transmission
    VOICE_TX,       // Voice transmission ended, PTT goes inactive. Disable the ISR
    GPS_TX,         // GPS transmission, key PTT active and wait on timer to transition back to idle
    NUM_STATES
} ESP_State;

ESP_State cur_state;

// initializes cur_state to bootup, waits for some amount of time, then transitions to idle
void state_init() {
    cur_state = BOOTUP;
    delay(1000);
    cur_state = IDLE;
    config_gpio();
    tx_init();
}

void idle() {
    

}

void 