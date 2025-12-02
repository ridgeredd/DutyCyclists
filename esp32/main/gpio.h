#ifndef GPIO_H
#define GPIO_H

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

// Clears pending interrupts on pin
inline void clear_gpio_intr_status(gpio_num_t pin);

// Sets up gpio pins
void config_gpio();

#endif