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
#include "soc/gpio_struct.h"

#include "gps.h"

#include "libfec/fec.h"

static const char *TAG = "gps_tx";

#define SAMPLE_RATE             48000                   // 48 kHz sample rate; ideally this is a multiple of each symbol frequency
#define BUF_LEN                 256                // frames per DAC buffer. 2 * 256 because stereo
#define PI                      3.141592653589793f
#define TX_LENGTH               10          // number of symbols being sent
#define SAMPLES_PER_SYMBOL      400

#define BITS_PER_SYMBOL         1           // power of 2; will cause error if > 8 because using uint8
#define NSYMBOLS                (1u << BITS_PER_SYMBOL)

static const float SYMBOL_FREQS[NSYMBOLS]   = { 2200.0f, 1200.0f }; // symbol frequencies in order of bit index

#define SYMBOL_MASK             (NSYMBOLS - 1u)
#define TRANSMISSION_FLAG       0x7E        // start/stop condition

#define CODEWORD_BITS           4           // how many bits are combined for RS code alphabet
#define REDUNDANT_BITS          4           // how many bits are appended for error correction

#define AUTOMATIC_TRANSMISSION       1           // 1 enables 0 disables auto transmission at regular interval
#define TIMER_DELAY_MS               10000       // milliseconds before resending signal
static volatile uint64_t last_tx_ticks = 0;         // Last transmission time
static volatile uint64_t last_isr_ticks = 0;

#define PTT_INPUT_PIN                 GPIO_NUM_4
#define PTT_OUTPUT_PIN                GPIO_NUM_5
#define PTT_BUFFER_MS                 100        // amount of additional time you hold PTT high after transmission length
#define PTT_ACTIVE_LVL                0         // active high (1) or low (0)
#define PTT_INACTIVE_LVL              1

#define CALL_SIGN                     0x67      // unique id for this device

#if PTT_ACTIVE_LVL
    #define PTT_PRESS_INTR            EDGE_RISING // Press is rising edge     
    #define PTT_RELEASE_INTR          EDGE_FALLING // release is falling edge
#else
    #define PTT_PRESS_INTR            EDGE_FALLING
    #define PTT_RELEASE_INTR          EDGE_RISING
#endif         

#define DAC_AMPLITUDE_V                1.0    // Must be within [0, 3.3]
#define MIN_DAC_V                      1.0    // band aid solution to dac voltage being clipped when ptt is high; 2 * dac amplitude + dac v must be < 5
#define START_STOP                     0x7E   // starting and stopping condition

#define PARITY_BYTES                   6      // These must sum to < 255
#define PAYLOAD_BYTES                  9      // payload + callsign      
    
#define START_NBYTES                   2      // 6 start bytes in a row
#define END_NBYTES                     1      // 4 end bytes
#define TRANSMISSION_BYTES             PAYLOAD_BYTES + PARITY_BYTES + START_NBYTES + END_NBYTES

const double TRANSMISSION_TIME_MS   = (1000 * SAMPLES_PER_SYMBOL * 8 * TRANSMISSION_BYTES) / (BITS_PER_SYMBOL * SAMPLE_RATE);
const double MIN_INTERVAL_MS        = 4 * TRANSMISSION_TIME_MS;   // Don't allow transmissions within min interval of eachother 
const double BAUD_RATE              = SAMPLE_RATE / SAMPLES_PER_SYMBOL;
const double BIT_RATE               = BAUD_RATE * BITS_PER_SYMBOL;

#define DEBOUNCE_MS             100          // ignore isr requests within some recent amount of time to debounce signal

#if AUTOMATIC_TRANSMISSION
static TimerHandle_t auto_gps_timer = NULL;         // automatically requests a gps send at regular intervals
#endif
static TimerHandle_t ptt_disable_timer = NULL;      // times how long to keep ptt high for  
static TaskHandle_t send_gps_handle = NULL;         // task handle to request transmission

static void* rs_encoder = NULL;

static int gps_tx = 0;                                     // whether or not gps is currently being transmitted

static inline void clear_gpio_intr_status(gpio_num_t pin){
    uint32_t m = 1u << (uint32_t)pin;
    if (pin < 32) GPIO.status_w1tc = m;
    else          GPIO.status1_w1tc.val = 1u << (pin - 32);
}

// Send a string of bytes; Sends symbols in little-endian
// TODO: implement cosine look up table (possible with a phase update?); Separate this into a different file
// TODO: issue. takes about twice as long; due to integer division of #defines
// Uses bit stuffing to add a 0 after every 5 ones, excluding first and last bytes
static void tx_bytes(uint8_t *bytes, uint32_t nbytes, int16_t *buf){

    uint8_t byte_idx = 0;
    uint8_t byte = bytes[0];
    uint8_t bit_idx = 0;

    #if BITS_PER_SYMBOL == 1

    uint8_t bstuff_en = 0;    // is bit stuffing currently enabled for this bit
    uint8_t prev_nrzi = 0;     // stores last bit transmitted for NRZI
    uint8_t bit = ( byte & SYMBOL_MASK );

    uint8_t nrzi_bit;
    if( bit ) { nrzi_bit = prev_nrzi; } else { nrzi_bit = !prev_nrzi; }
    printf("%d", nrzi_bit);
    uint8_t num_ones = 0;   // used for bit stuffing; after 5 ones insert a 0; not used in first / last byte
    float freq = SYMBOL_FREQS[nrzi_bit];

    #else

    float freq = SYMBOL_FREQS[byte & SYMBOL_MASK]

    #endif

    // do the first read and shift before the loop
    byte = byte >> BITS_PER_SYMBOL; 
    bit_idx += BITS_PER_SYMBOL;

    float phase = 0;
    int count = 0;   // times the symbol switches

    size_t written = 0;
    
    while (1) {

        // determines how many samples until will switch 
        // Ensure that the buffer length is less than the samples per symbol
        int next_switch = SAMPLES_PER_SYMBOL - count; 

        // fill the transmission buffer with samples at a frequency
        for (int i = 0; i < BUF_LEN; i++) {

            // Write sample to dac
            float cos01 = 0.5f * cosf(phase) + 0.5f;                                    // cos wave between 0 and 1 with amplitude 0.5
            float voltage = 2 * DAC_AMPLITUDE_V * cos01 + MIN_DAC_V;                    // use parameters to determine voltage to send at
            uint8_t u8 = (uint8_t)(voltage * 255 / 3.3);                                // normalize to 0-255 byte to send
            buf[2 * i] = (int16_t)(u8 << 8);                    // Top byte gets fed to DAC
            buf[2 * i + 1] = 0;                                 // Other DAC zeroed; Required for mono

            // If this sample swaps frequency
            // TODO: bit stuffing should happen before nrzi; currently happens after
            if (i == next_switch) {

                // If made through entire byte, continue in byte array
                if (bit_idx == 8) {

                    printf(" ");

                    // If transmission finished; auto flush buffer
                    if (byte_idx == nbytes - 1) { 
                        printf("\n\n");
                        i2s_write(I2S_NUM_0, buf, 2 * (i + 1) * sizeof(int16_t), &written, portMAX_DELAY); 
                        return;
                    }
                    // else continue through byte array
                    byte_idx++;
                    byte = bytes[byte_idx];
                    bit_idx = 0;

                }

                // Get frequency of next symbol
                #if BITS_PER_SYMBOL == 1

                    // save the previous level
                    prev_nrzi = nrzi_bit;

                    // normal case; grab next symbol and shift
                    // don't bit stuff on first or last byte
                    if ( !bstuff_en || byte_idx < START_NBYTES || byte_idx >= TRANSMISSION_BYTES - END_NBYTES ) {  
                        bit = (byte & SYMBOL_MASK);
                        bit_idx += BITS_PER_SYMBOL;
                        byte = byte >> BITS_PER_SYMBOL;
                    }
                    // this bit should be stuffed; retain current position and inject 0
                    else { bit = 0; }

                    // if 1 increment consecutive 1's counter 
                    if ( bit && byte_idx != 0 && byte_idx != nbytes - 1 ) { num_ones += 1; } 
                    // else reset consecutive 1's counter 
                    else { 
                        num_ones = 0; 
                        bstuff_en = 0; // catches case where we are currently bit stuffing (bit = 0) and disables it
                    }

                    // five ones in a row, enable bit stuffing for next bit
                    if ( num_ones >= 5 ) { bstuff_en = 1; num_ones = 0; }

                    // calculate nrzi
                    if( bit ) { nrzi_bit = prev_nrzi; } else { nrzi_bit = !prev_nrzi; }
                    printf("%d", nrzi_bit);

                    // correct frequency to transmit at
                    freq = SYMBOL_FREQS[nrzi_bit];

                #else
                    // simple; no NRZI or stuffing; simply shift and read
                    freq = SYMBOL_FREQS[byte & SYMBOL_MASK];
                    bit_idx += BITS_PER_SYMBOL;
                #endif

            }

            // update phase; use a phase accumulator for smooth transitions between frequencies
            phase += 2 * PI * freq / SAMPLE_RATE;
            if (phase >= 2 * PI) { phase -= 2 * PI; }

        }
        // buffer full; flush
        i2s_write(I2S_NUM_0, buf, 2 * BUF_LEN * sizeof(int16_t), &written, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1)); // delay for some amount of time to free up other tasks
        count += BUF_LEN;
        if (count > SAMPLES_PER_SYMBOL) { count -= SAMPLES_PER_SYMBOL; } // wrap around if transmitted symbol changed
        // it is important this is > not >= bc >= would cause issue where new count == samples per symbol, would reset to 0 rather than changing in next iteration

    }

}

// TODO: implement
// currently stub
static void get_gps(uint8_t *bytes) {

    // for (uint8_t count = 0; count < PAYLOAD_BYTES - 1; count++) {
    //     bytes[count] = count;
    //     //32 bit lat 32 bit long
    // }

    gnss_data_t fix;
    if(!gps_get_fix(&fix)) {
        // no GPS fix; send zeros
        memset(bytes, 0, PAYLOAD_BYTES - 1);
        return;
    } 

    // // check if fix/location data is >1 minute old
    // if(fix.fix_age_ms > 60000) { 
    //     // gps fix too old; send zeros
    //     memset(bytes, 0, PAYLOAD_BYTES - 1);
    //     return;
    // }

    // esp prints lat/long/age values
    // ESP_LOGI(TAG, "Encoding GPS: lat=%.6f lon=%.6f (age=%lu ms)", fix.latitude, fix.longitude, fix.fix_age_ms);

    // quantize lat/lon to uint32_t (with error of ~0.04 m)
    uint32_t lat_q = (uint32_t)((fix.latitude + 90.0) * (4294967295.0 / 180.0));
    uint32_t lon_q = (uint32_t)((fix.longitude + 180.0) * (4294967295.0 / 360.0));

     // Pack into 8 bytes (little-endian)
    // latitude (32 bits)
    bytes[0] = (uint8_t)(lat_q & 0xFF);
    bytes[1] = (uint8_t)((lat_q >> 8) & 0xFF);
    bytes[2] = (uint8_t)((lat_q >> 16) & 0xFF);
    bytes[3] = (uint8_t)((lat_q >> 24) & 0xFF);
    
    // longitude (32 bits)
    bytes[4] = (uint8_t)(lon_q & 0xFF);
    bytes[5] = (uint8_t)((lon_q >> 8) & 0xFF);
    bytes[6] = (uint8_t)((lon_q >> 16) & 0xFF);
    bytes[7] = (uint8_t)((lon_q >> 24) & 0xFF);
}

// Continuously running process that wakes to send gps transmission
static void tx_gps_task() {

    int16_t *buf = (int16_t*)heap_caps_malloc(BUF_LEN * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    uint8_t *tx_data = (uint8_t*)heap_caps_malloc(TRANSMISSION_BYTES * sizeof(uint8_t), MALLOC_CAP_DEFAULT);

    tx_data[0] = START_STOP;
    tx_data[1] = START_STOP;
    tx_data[START_NBYTES] = CALL_SIGN;
    tx_data[TRANSMISSION_BYTES - 1] = START_STOP;

    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                // wait until woken up
        uint8_t *payload = (uint8_t*)(tx_data + START_NBYTES);             // pointer to the payload bytes
        uint8_t *gps     = (uint8_t*)(payload + 1);                

        get_gps(gps);                                       // currently stubbed
        uint8_t *parity = (uint8_t*)(payload + PAYLOAD_BYTES);  // pointer to the parity bytes

        encode_rs_char(rs_encoder, payload, parity); // set parity bytes

        printf("payload: ");
        for (size_t i = 0; i < PAYLOAD_BYTES; i++) {
            printf("%02X ", payload[i]);  // two-digit uppercase hex
        }
        printf("\n");

        printf("Message: ");
        for (size_t i = 0; i < TRANSMISSION_BYTES; i++) {
            printf("%02X ", tx_data[i]);  // two-digit uppercase hex
        }
        printf("\n\n");

        gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL); // set ptt active
        xTimerStart(ptt_disable_timer, 0);  // start timer to reset ptt after some time
        tx_bytes(tx_data, TRANSMISSION_BYTES, buf);

    }

}

// Resets ptt after gps transmission
static void disable_ptt_timer_callback() {
    gpio_set_level(PTT_OUTPUT_PIN, PTT_INACTIVE_LVL);
    #if !AUTOMATIC_TRANSMISSION
    clear_gpio_intr_status(PTT_INPUT_PIN);
    gpio_intr_enable(PTT_INPUT_PIN);
    #endif
    gps_tx = 0;
}

// Spawns send gps task when ptt releases
static void IRAM_ATTR ptt_interrupt_handler() {
    
    // ignore this edge; this is from a gps transmit, not a ptt release
    if (gps_tx == 1) { return; }

    int level = gpio_get_level(PTT_INPUT_PIN);
    if (level == 0) {
        return;
    }

    uint64_t now_ticks = xTaskGetTickCountFromISR();
    uint64_t elapsed_isr_ms = (now_ticks - last_isr_ticks) * portTICK_PERIOD_MS;
    if (elapsed_isr_ms < DEBOUNCE_MS) return; // debounce
    last_isr_ticks = now_ticks;

    BaseType_t hpw = pdFALSE;

    #if AUTOMATIC_TRANSMISSION // reset internal gps timer
        xTimerResetFromISR(auto_gps_timer, &hpw); 
    #endif 

    // ptt release -> send gps
    uint64_t elapsed_tx_ms = (now_ticks - last_tx_ticks) * portTICK_PERIOD_MS;
    if (elapsed_tx_ms < MIN_INTERVAL_MS) return;   // ignore too-soon interrupts
    last_tx_ticks = now_ticks;

    #if !AUTOMATIC_TRANSMISSION
    gpio_intr_disable(PTT_INPUT_PIN); // disable interrupts on ptt
    clear_gpio_intr_status(PTT_INPUT_PIN);
    #endif
    gps_tx = 1;


    vTaskNotifyGiveFromISR(send_gps_handle, &hpw); 

    if (hpw) portYIELD_FROM_ISR();

}

#if AUTOMATIC_TRANSMISSION
// Spawns send gps task when timer callback occurs
// TODO: use ticks not get_time
static void gps_timer_callback() {

    int ptt_level = gpio_get_level(PTT_INPUT_PIN);
    if (ptt_level == PTT_ACTIVE_LVL) { return; } // if ptt currently active, abort

    uint64_t now_ticks = xTaskGetTickCount();
    uint64_t elapsed_tx_ms = (now_ticks - last_tx_ticks) * portTICK_PERIOD_MS;
    if (elapsed_tx_ms < MIN_INTERVAL_MS) return;   // ignore too-soon interrupts
    last_tx_ticks = now_ticks;

    #if !AUTOMATIC_TRANSMISSION
    gpio_intr_disable(PTT_INPUT_PIN); // disable interrupts on ptt
    clear_gpio_intr_status(PTT_INPUT_PIN);
    #endif
    gps_tx = 1;

    //gpio_set_level(PTT_OUTPUT_PIN, PTT_ACTIVE_LVL);
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

    #if !AUTOMATIC_TRANSMISSION
    gpio_config_t ptt_in_conf = {
        .intr_type = GPIO_INTR_POSEDGE,  // activates on rising edge
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

    ptt_disable_timer = xTimerCreate(
        "ptt_disable_timer",
        pdMS_TO_TICKS(3 * TRANSMISSION_TIME_MS),
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

void app_main(void) {

    printf("Transmission Time: %f ms\n", TRANSMISSION_TIME_MS);
    printf("Baud Rate: %f sym/sec\n", BAUD_RATE);
    printf("Bit Rate: %f bit/sec\n", BIT_RATE);
    printf("DAC Range: %fV to %fV\n", MIN_DAC_V, MIN_DAC_V + 2 * DAC_AMPLITUDE_V);
    #if PTT_ACTIVE_LVL
        printf("PTT Active High\n");
    #else
        printf("PTT Active Low\n");
    #endif

    // Initialize reed-solomon encoder. Requirements for library specified here: 
    // https://manpages.debian.org/unstable/libfec-dev/rs.3.en.html?utm_source=chatgpt.com
    rs_encoder = init_rs_char(
        8, // gives the symbol size in bits, up to 32
        0x11D, // gives the extended Galois field generator polynomial coefficients, with the 0th coefficient in the low order bit. The polynomial must be primitive; if not, the call will fail and NULL will be returned.
		0, // gives, in index form, the first consecutive root of the Reed Solomon code generator polynomial
        1, // gives, in index form, the primitive element in the Galois field used to generate the Reed Solomon code generator polynomial.
        PARITY_BYTES, // gives the number of roots in the Reed Solomon code generator polynomial. This equals the number of parity symbols per code block.
		255 - PARITY_BYTES - PAYLOAD_BYTES // gives the number of leading symbols in the codeword that are implicitly padded to zero in a shortened code block.
        // The resulting Reed-Solomon code has parameters (N,K), where N = 2^symsize - pad - 1 and K = N-nroots.
    );

    // Initialize GPS Reader
    gps_start_task();

    xTaskCreate(tx_gps_task, "gps_task", 4096, NULL, 3 /*priority*/, &send_gps_handle);

    #if MIN_INTERVAL_MS < TRANSMISSION_TIME_MS
        ESP_LOGI(TAG, "Ensure that minimum interval time between transmission is greater than transmission time");
    #endif

    config_dac();
    config_ptt();

    #if AUTOMATIC_TRANSMISSION 
        config_auto_gps_timer();
    #endif
    
}

