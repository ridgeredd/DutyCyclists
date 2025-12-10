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
/*#include "esp_timer.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "soc/gpio_struct.h"*/

#include "fsm.h"
#include "ptt.h"
#include "tx.h"
#include "auto_tx.h"

static const char *TAG = "gps_tx";

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

    #if MIN_INTERVAL_MS < TRANSMISSION_TIME_MS
        ESP_LOGI(TAG, "Ensure that minimum interval time between transmission is greater than transmission time");
    #endif

    ptt_init();
    tx_init();
    auto_tx_init();
    fsm_init();

    #if AUTOMATIC_TRANSMISSION 
        config_auto_gps_timer();
    #endif

    while(1) {

        //reload_gps();
        fsm_main();
        vTaskDelay(pdMS_TO_TICKS(10));

    }
    
}

