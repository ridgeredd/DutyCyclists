#define AUTO_TX_MS               10000       // milliseconds before resending signal
#define AUTOMATIC_TRANSMISSION       0       // enables (1) or disables (0) auto transmission at regular interval

#include "auto_tx.h"
//#include <inttypes.h>
//#include "ptt.h"
//#include "fsm.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"

// TODO: consider using a hardware timer, not a freertos timer
static TimerHandle_t auto_tx_timer = NULL;         // automatically requests a gps send at regular intervals

static void auto_tx_timer_callback();

// TODO: set an internal flag that fsm checks
static uint8_t auto_tx;

//================================================ API ===============================================================

// Build and configure internal auto transmission timer
void auto_tx_init() {

    #if TIMER_DELAY_MS < MIN_INTERVAL_MS
        ESP_LOGI(TAG, "Ensure that timer delay is greater than minimum interval between tx and total transmission time");
    #endif

    auto_tx_timer = xTimerCreate(
        "gps_timer",
        pdMS_TO_TICKS(AUTO_TX_MS),
        pdTRUE, // repeats timer after expires
        NULL,
        auto_tx_timer_callback
    );

    #if AUTOMATIC_TRANSMISSION
    xTimerStart(auto_tx_timer, 0);
    #endif

}

void reset_auto_tx_timer() { 
    auto_tx = 0; 
    #if AUTOMATIC_TRANSMISSION
    xTimerReset(auto_tx_timer, 0); 
    #endif
}

uint8_t get_auto_tx() { 

    if( auto_tx == 1 ) { auto_tx = 0; return 1; }
    return 0;
}

//=====================================================================================================================

// Callback function which updates FSM
static void auto_tx_timer_callback() { /*FSM_flag_auto_tx();*/ auto_tx = 1; }
