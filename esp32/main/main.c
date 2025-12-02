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


#include "libfec/fec.h"

static int gps_tx = 0;                                     // whether or not gps is currently being transmitted

static volatile uint64_t last_tx_ticks = 0;         // Last transmission time
static volatile uint64_t last_isr_ticks = 0;

#if AUTOMATIC_TRANSMISSION
// Spawns send gps task when timer callback occurs
// TODO: use ticks not get_time


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

void app_main(void) {

    printf("Transmission Time: %d ms\n", TRANSMISSION_TIME_MS);
    printf("Baud Rate: %d sym/sec\n", BAUD_RATE);
    printf("Bit Rate: %d bit/sec\n", BIT_RATE);
    printf("DAC Range: %fV to %fV\n", MIN_DAC_V, MIN_DAC_V + 2 * DAC_AMPLITUDE_V);
    #if PTT_ACTIVE_LVL
        printf("PTT Active High\n");
    #else
        printf("PTT Active Low\n");
    #endif

    #if MIN_INTERVAL_MS < TRANSMISSION_TIME_MS
        ESP_LOGI(TAG, "Ensure that minimum interval time between transmission is greater than transmission time");
    #endif

    config_dac();
    config_ptt();

    #if AUTOMATIC_TRANSMISSION 
        config_auto_gps_timer();
    #endif
    
}

