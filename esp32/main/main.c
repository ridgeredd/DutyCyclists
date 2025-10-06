/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

static const char *TAG = "gps_tx";

#define SAMPLE_RATE             48000                   // 48 kHz sample rate; ideally this is a multiple of each symbol frequency
#define BUF_LEN                 256                // frames per DAC buffer. 2 * 256 because stereo
#define PI                      3.141592653589793f
#define TX_LENGTH               10          // number of symbols being sent
#define SAMPLES_PER_SYMBOL      1000

#define BITS_PER_SYMBOL         1           // power of 2; will cause error if > 8 because using uint8
#define NSYMBOLS                (1u << BITS_PER_SYMBOL)

static const float SYMBOL_FREQS[NSYMBOLS]   = { 1200.0f, 2200.0f }; // symbol frequencies in order of bit index

#define SYMBOL_MASK             (NSYMBOLS - 1u)
#define TRANSMISSION_FLAG       0x7E        // start/stop condition

#define CODEWORD_BITS           4           // how many bits are combined for RS code alphabet
#define REDUNDANT_BITS          4           // how many bits are appended for error correction

#define PAYLOAD_LEN             48
#define TRANSMISSION_LEN        64          //(2 * 8) + PAYLOAD_LEN + REDUNDANT_BITS        // Total bits being sent

#define TRANSMISSION_TIME_MS    (1000 * SAMPLES_PER_SYMBOL * TRANSMISSION_LEN) / (BITS_PER_SYMBOL * SAMPLE_RATE)
#define MIN_INTERVAL_MS         2 * TRANSMISSION_TIME_MS   // Don't allow transmissions within 2 * min interval of eachother 

#define AUTOMATIC_TRANSMISSION       1           // 1 enables 0 disables auto transmission at regular interval
#define TIMER_DELAY_MS               10000       // milliseconds before resending signal
static volatile uint64_t last_tx_us = 0;         // Last transmission time

#define PTT_INPUT_PIN                 GPIO_NUM_4
#define PTT_OUTPUT_PIN                GPIO_NUM_5
#define PTT_BUFFER_MS                 10        // amount of additional time you hold PTT high after transmission length
#define PTT_ACTIVE_LVL                0         // active high (1) or low (0)
#define PTT_INACTIVE_LVL              !PTT_ACTIVE_LVL

#if PTT_ACTIVE_LVL
    #define PTT_PRESS_INTR            EDGE_RISING // Press is rising edge     
    #define PTT_RELEASE_INTR          EDGE_FALLING // release is falling edge
#else
    #define PTT_PRESS_INTR            EDGE_FALLING
    #define PTT_RELEASE_INTR          EDGE_RISING
#endif         

#if AUTOMATIC_TRANSMISSION
static TimerHandle_t auto_gps_timer = NULL;         // automatically requests a gps send at regular intervals
#endif
static TimerHandle_t ptt_disable_timer = NULL;      // times how long to keep ptt high for  
static TaskHandle_t send_gps_handle = NULL;         // task handle to request transmission

// Send a string of 32 bit words; Sends symbols in little-endian
// TODO: implement cosine look up table; Separate this into a different file
static void encode_words(uint32_t *words, uint32_t nwords, int16_t *buf) {

    int word_idx = 0;
    int bit_idx = BITS_PER_SYMBOL;
    uint32_t word = words[0];
    float freq = SYMBOL_FREQS[word & SYMBOL_MASK];

    static float phase = 0;
    static int count = 0;   // times the symbol switches

    size_t written = 0;
    
    while (1) {

        int next_switch = SAMPLES_PER_SYMBOL - count; // Ensure that the buffer length is less than the samples per symbol
        // TODO: don't do this because samples per symbol may not be an integer
        for (int i = 0; i < BUF_LEN; i++) {

            float val = cosf(phase);                              // cos wave with 1 amplitude
            uint8_t u8 = (uint8_t)((val * 0.5f + 0.5f) * 255);    // cast to byte for DAC
            buf[2 * i] = (int16_t)(u8 << 8);                    // Top byte gets fed to DAC
            buf[2 * i + 1] = 0;                                 // Required for mono

            // Swap frequency mid buffer
            if (i == next_switch) {

                // increment bits; TODO: optionally don't send all the bits of last word
                if (bit_idx < 32) {
                    word = word >> BITS_PER_SYMBOL;
                    freq = SYMBOL_FREQS[word & SYMBOL_MASK];
                    bit_idx += BITS_PER_SYMBOL;
                } 
                // increment to new word
                else {
                    if (word_idx == nwords - 1) { // transmission finished
                        i2s_write(I2S_NUM_0, buf, 2 * (i + 1) * sizeof(int16_t), &written, portMAX_DELAY); 
                        //vTaskDelete(NULL);
                        return;
                    }
                    else {
                        word_idx++;
                        word = words[word_idx];
                        freq = SYMBOL_FREQS[word & SYMBOL_MASK]; // out of bounds exception
                        bit_idx = BITS_PER_SYMBOL;
                    }
                }

            }

            // update phase
            phase += 2 * PI * freq / SAMPLE_RATE;
            if (phase >= 2 * PI) { phase -= 2 * PI; }

        }
        i2s_write(I2S_NUM_0, buf, 2 * BUF_LEN * sizeof(int16_t), &written, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1)); // delay for some amount of time to free up other tasks
        count += BUF_LEN;
        if (count > SAMPLES_PER_SYMBOL) { count -= SAMPLES_PER_SYMBOL; } // wrap around if transmit changed
        // it is important this is > not >= bc >= would cause issue where new count == samples per symbol, would reset to 0 rather than changing in next iteration

    }

}

// TODO: add start stop conditions and error correction
static void send_gps() {

    int16_t *buf = (int16_t*)heap_caps_malloc(BUF_LEN * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);

    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t words[] = { 0xA4AB9AA0, 0xA4083A5A };   // literal is temporary
        uint32_t nwords = 2;
        gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL);
        xTimerStart(ptt_disable_timer, 0);  // start timer to reset ptt after some time
        encode_words(words, nwords, buf);

    }

}

#if AUTOMATIC_TRANSMISSION
// Resets ptt after gps transmission
static void disable_ptt_timer_callback() {
    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);
}
#endif

// Spawns send gps task when ptt releases, or drives ptt active when ptt pushed
static void IRAM_ATTR ptt_interrupt_handler() { 

    BaseType_t hpw = pdFALSE;

    #if AUTOMATIC_TRANSMISSION // reset internal gps timer
        xTimerResetFromISR(auto_gps_timer, &hpw); 
    #endif 

    int level = gpio_get_level(PTT_INPUT_PIN);

    // ptt press -> forward ptt to radio
    if (level == PTT_ACTIVE_LVL) { gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL); }
    // ptt release -> send gps
    else { 
        uint64_t now_us = esp_timer_get_time();
        if ((now_us - last_tx_us) / 1000 < MIN_INTERVAL_MS) return;   // ignore too-soon interrupts
        last_tx_us = now_us;
        vTaskNotifyGiveFromISR(send_gps_handle, &hpw); 
    }

    if (hpw) portYIELD_FROM_ISR();

}

#if AUTOMATIC_TRANSMISSION
// Spawns send gps task when timer callback occurs
static void gps_timer_callback() {

    int ptt_level = gpio_get_level(PTT_INPUT_PIN);
    if (ptt_level == PTT_ACTIVE_LVL) { return; } // if ptt currently active, abort

    uint64_t now_us = esp_timer_get_time();
    if ((now_us - last_tx_us) / 1000 < MIN_INTERVAL_MS) return;   // ignore too-soon interrupts
    last_tx_us = now_us;

    gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL);
    xTaskNotifyGive(send_gps_handle);

}
#endif

// Build and configure dac
static void config_dac() {

    #if BUF_LEN > SAMPLES_PER_SYMBOL
        ESP_LOGI(TAG, "Ensure that DAC buffer length is less than samples per symbol");
    #endif

    i2s_config_t config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
    };

    i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); // Mono; GPIO 25 carries output
    i2s_set_pin(I2S_NUM_0, NULL);
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

}

// Build and configure ptt pins and handlers
static void config_ptt() {

    gpio_config_t ptt_in_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,  // activates on either edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PTT_INPUT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,   // enable if signal is open drain
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&ptt_in_conf);

    gpio_config_t ptt_out_conf = {
        .pin_bit_mask = 1ULL << PTT_OUTPUT_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,     // pulls not needed for push-pull output
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&ptt_out_conf);

    ptt_disable_timer = xTimerCreate(
        "ptt_disable_timer",
        pdMS_TO_TICKS(TRANSMISSION_TIME_MS + PTT_BUFFER_MS),
        pdFALSE,           // auto-reload = false
        NULL,
        disable_ptt_timer_callback
    );

    // TODO: move. Only call once per application (shared across all GPIOs)
    gpio_install_isr_service(0);

    // Attach handler
    gpio_isr_handler_add(PTT_INPUT_PIN, ptt_interrupt_handler, NULL);

    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);

}

#if AUTOMATIC_TRANSMISSION
// Build and configure internal gps timer
static void config_auto_gps_timer() {

    #if TIMER_DELAY_MS < MIN_INTERVAL_MS
        ESP_LOGI(TAG, "Ensure that timer delay is greater than minimum interval between tx and total transmission time");
    #endif

    auto_gps_timer = xTimerCreate(
        "gps_timer",
        pdMS_TO_TICKS(TIMER_DELAY_MS),
        pdTRUE,           // auto-reload = true (repeats)
        NULL,
        gps_timer_callback
    );

    xTimerStart(auto_gps_timer, 0);

}
#endif

// Changes a message to NRZI form to encode flips rather than bits; EG 1101 -> 1011; assumed starting bit of 0;
// only use when bits per symbol is 1
static void NRZI(uint32_t *words, uint32_t nwords) {

    int prev = 0;
    for (int word_idx = 0; word_idx < nwords; word_idx++) {
        uint32_t word = words[word_idx];
        int cur = word & 1;
        uint32_t mask = (word << 1) ^ prev; //
        words[word_idx] = word ^ mask;
        prev = cur;
    }

}

static void envelope() {

    

}

static void reed_solomon() {



}

void app_main(void) {

    xTaskCreate(send_gps, "gps_task", 4096, NULL, 3 /*priority*/, &send_gps_handle);

    #if MIN_INTERVAL_MS < TRANSMISSION_TIME_MS
        ESP_LOGI(TAG, "Ensure that minimum interval time between transmission is greater than transmission time");
    #endif

    config_dac();
    config_ptt();

    #if AUTOMATIC_TRANSMISSION 
        config_auto_gps_timer();
    #endif
    
}

