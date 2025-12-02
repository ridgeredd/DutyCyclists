#ifndef TX_H
#define TX_H

#include <inttypes.h>
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

// Initializes all timers, memory, tasks, and reed solomon encoder. Returns the handle of the task to wake to send gps
TaskHandle_t tx_config();

// Start the automatic transmit timer
void start_auto_tx_timer();

// Reset the automatic transmit timer to 0
void reset_auto_tx_timer();

#endif