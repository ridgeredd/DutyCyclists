#include "driver/gpio.h"
#include "soc/gpio_struct.h"

#include <stdio.h>
#include <inttypes.h>
#include "ptt.h"
//#include "fsm.h"
#include "tx.h" // For TRANSMISSION_TIME_MS

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "esp_log.h"

#define PTT_IN_ENABLE                 1             // Monitor (1) or don't monitor (0) PTT presses
#define PTT_INPUT_PIN                 GPIO_NUM_19
#define PTT_OUTPUT_PIN                GPIO_NUM_18
#define PTT_ACTIVE_LVL                0             // active high (1) or low (0)
#define PTT_INACTIVE_LVL              1 
#define PTT_DISABLE_TIMER_MS          3 * TRANSMISSION_TIME_MS
#define PTT_LONG_DISABLE_TIMER_MS     5 * TRANSMISSION_TIME_MS
#define DEBOUNCE_MS                   100        // ignore isr requests within some recent amount of time to debounce signal

static TimerHandle_t ptt_disable_timer = NULL; // Timer to reset ptt 
static TimerHandle_t ptt_long_disable_timer = NULL; // Timer to reset ptt 
static uint8_t ptt_level;
static uint8_t ptt_disabled;

static void config_ptt();
static void config_ptt_disable_timer();
static void disable_ptt_gpio();
static void enable_ptt_gpio();

//====================================== API ======================================================

void ptt_init() {

    config_ptt();
    config_ptt_disable_timer();
    ptt_disabled = 0;
}

uint8_t get_ptt() { /*return gpio_get_level(PTT_INPUT_PIN);*/ return ptt_level; }

uint8_t poll_ptt() { return gpio_get_level(PTT_INPUT_PIN); }

// Sets ptt active and disables interrupts on ptt input pin. Starts timer to automatically reset it
void key_ptt() {

    disable_ptt_gpio();
    gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL); // set ptt active
    ptt_level = PTT_INACTIVE_LVL; // ? does this matter ?
    xTimerStart(ptt_disable_timer, 0);  // start timer to reset ptt
}

void key_ptt_long() {
    disable_ptt_gpio();
    gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL); // set ptt active
    ptt_level = PTT_INACTIVE_LVL; // ? does this matter ?
    xTimerStart(ptt_long_disable_timer, 0);
}

// TODO: we could have better organization. maybe move all callback logic into tx.c so that this is just a driver class
// doesn't fully make sense to have is_tx_finished() in this class
uint8_t was_ptt_disabled() {

    if( ptt_disabled == 1 ) { ptt_disabled = 0; return 1; }
    return 0;
}

//========================================== PTT GPIO ===================================================

// Updates FSM when ptt pressed or released
static void IRAM_ATTR ptt_interrupt_handler() { ptt_level = gpio_get_level(PTT_INPUT_PIN); }

// Build and configure ptt pins and interrupt handlers
static void config_ptt() {

    #if PTT_IN_ENABLE
    gpio_config_t ptt_in_conf = {
        #if PTT_INTERRUPT_ENABLE
        .intr_type = GPIO_INTR_ANYEDGE,  // activates on any edge
        #else
        .intr_type = GPIO_INTR_DISABLE, // disabled
        #endif
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PTT_INPUT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,      // floats ptt. radio should also, but this works better
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&ptt_in_conf);
    #endif

    gpio_config_t ptt_out_conf = {
        .pin_bit_mask = 1ULL << PTT_OUTPUT_PIN,
        .mode = GPIO_MODE_OUTPUT_OD,                  // Open drain. "Pull Down"
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Do not pull up or down for output
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&ptt_out_conf);

    // TODO: move. Only call once per application (shared across all GPIOs)
    gpio_install_isr_service(0);

    // Attach handler
    #if PTT_INTERRUPT_ENABLE
    gpio_isr_handler_add(PTT_INPUT_PIN, ptt_interrupt_handler, NULL);
    #endif

    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);

    ptt_level = gpio_get_level(PTT_INPUT_PIN);

}

// Clears any outstanding interrupts on a pin
static inline void clear_gpio_intr_status(gpio_num_t pin){

    uint32_t m = 1u << (uint32_t)pin;
    if (pin < 32) { GPIO.status_w1tc = m; }
    else { GPIO.status1_w1tc.val = 1u << (pin - 32); }
}

// Disables ptt input pin interrupts
static inline void disable_ptt_gpio() {

    gpio_intr_disable(PTT_INPUT_PIN); // disable interrupts on ptt
    clear_gpio_intr_status(PTT_INPUT_PIN);
    ptt_level = PTT_INACTIVE_LVL;
}

// Enables ptt input pin interrupts
static inline void enable_ptt_gpio() {

    clear_gpio_intr_status(PTT_INPUT_PIN);
    gpio_intr_enable(PTT_INPUT_PIN);
}

//================================= PTT DISABLE =========================================================

// Callback function which reenables interrupts
static void ptt_disable_timer_callback() {

    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);
    enable_ptt_gpio();
    ptt_disabled = 1;
}

// Configures ptt disable timer
static void config_ptt_disable_timer() {

    ptt_disable_timer = xTimerCreate(
        "ptt_disable_timer",
        pdMS_TO_TICKS(PTT_DISABLE_TIMER_MS),
        pdFALSE,           // auto-reload = false
        NULL,
        ptt_disable_timer_callback
    );

    ptt_long_disable_timer = xTimerCreate(
        "ptt_disable_timer",
        pdMS_TO_TICKS(PTT_LONG_DISABLE_TIMER_MS),
        pdFALSE,           // auto-reload = false
        NULL,
        ptt_disable_timer_callback
    );

}
