/*
 * gps_test.c - Simple test to verify ESP32 <-> MAX-M10S UART connection
 * 
 * This test simply reads raw UART data from the GNSS module and prints
 * all received NMEA sentences to the console, with special highlighting
 * for GNGLL sentences.
 * 
 * Use this to verify:
 * 1. UART wiring is correct
 * 2. GNSS module is powered and transmitting
 * 3. NMEA sentences are being received
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "gps_test";

/* UART Configuration - Match your hardware */
#define GPS_UART_NUM      UART_NUM_2
#define GPS_TX_PIN        17   // ESP32 TX -> GNSS RX
#define GPS_RX_PIN        16   // ESP32 RX <- GNSS TX
#define GPS_BAUD_RATE     9600 // MAX-M10S default
#define GPS_BUF_SIZE      1024

/* Optional: LED indicator on GPIO 2 (built-in LED on many ESP32 boards) */
#define LED_PIN           GPIO_NUM_2
#define USE_LED_INDICATOR 1

static void init_led(void)
{
#if USE_LED_INDICATOR
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN, 0);
#endif
}

static void blink_led(void)
{
#if USE_LED_INDICATOR
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LED_PIN, 0);
#endif
}

static void gps_test_task(void *arg)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GPS Connection Test - MAX-M10S");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "UART: %d", GPS_UART_NUM);
    ESP_LOGI(TAG, "RX Pin: GPIO %d (ESP32 <- GNSS TX)", GPS_RX_PIN);
    ESP_LOGI(TAG, "TX Pin: GPIO %d (ESP32 -> GNSS RX)", GPS_TX_PIN);
    ESP_LOGI(TAG, "Baud Rate: %d", GPS_BAUD_RATE);
    ESP_LOGI(TAG, "========================================\n");

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_LOGI(TAG, "Configuring UART...");
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, 
                                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized successfully!");
    ESP_LOGI(TAG, "Waiting for NMEA data from GNSS module...\n");

    // Buffers
    uint8_t *uart_buf = (uint8_t*)malloc(GPS_BUF_SIZE);
    char line_buf[256];
    size_t line_pos = 0;
    
    uint32_t total_sentences = 0;
    uint32_t gngll_count = 0;
    uint32_t last_data_time = 0;
    bool first_data_received = false;

    while (1) {
        // Read from UART with timeout
        int len = uart_read_bytes(GPS_UART_NUM, uart_buf, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        
        if (len > 0) {
            if (!first_data_received) {
                ESP_LOGI(TAG, "✓ First data received from GNSS! (%d bytes)", len);
                first_data_received = true;
                blink_led();
            }
            
            last_data_time = xTaskGetTickCount();
            
            // Process each byte
            for (int i = 0; i < len; i++) {
                char c = (char)uart_buf[i];
                
                // Check for end of line
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = '\0';
                        
                        // Check if it's a valid NMEA sentence (starts with $)
                        if (line_buf[0] == '$') {
                            total_sentences++;
                            
                            // Check specifically for GNGLL or GPGLL
                            if (strncmp(line_buf + 1, "GNGLL", 5) == 0 || 
                                strncmp(line_buf + 1, "GPGLL", 5) == 0) {
                                gngll_count++;
                                
                                // Print GNGLL with highlighting
                                printf("\n");
                                ESP_LOGI(TAG, "==================== GNGLL #%lu ====================", 
                                         gngll_count);
                                printf("%s\n", line_buf);
                                ESP_LOGI(TAG, "====================================================\n");
                                
                                blink_led();
                            } else {
                                // Print other NMEA sentences normally
                                printf("%s\n", line_buf);
                            }
                        }
                        
                        line_pos = 0;
                    }
                } else {
                    // Add character to line buffer
                    if (line_pos < sizeof(line_buf) - 1) {
                        line_buf[line_pos++] = c;
                    } else {
                        // Buffer overflow - reset
                        line_pos = 0;
                        ESP_LOGW(TAG, "Line buffer overflow - sentence too long");
                    }
                }
            }
        } else {
            // No data received in timeout period
            uint32_t now = xTaskGetTickCount();
            if (first_data_received && (now - last_data_time) > pdMS_TO_TICKS(5000)) {
                ESP_LOGW(TAG, "No data received for 5 seconds - check connection");
                last_data_time = now;
            } else if (!first_data_received) {
                ESP_LOGW(TAG, "Waiting for data... (check wiring and power)");
            }
        }
        
        // Print statistics every 30 seconds
        static uint32_t last_stats_time = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_stats_time) > pdMS_TO_TICKS(30000)) {
            last_stats_time = now;
            ESP_LOGI(TAG, "--- Statistics ---");
            ESP_LOGI(TAG, "Total NMEA sentences: %lu", total_sentences);
            ESP_LOGI(TAG, "GNGLL/GPGLL count: %lu", gngll_count);
            ESP_LOGI(TAG, "------------------\n");
        }
        
        // Small delay to yield
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(uart_buf);
}

void app_main(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32 + MAX-M10S Connection Test    ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    ESP_LOGI(TAG, "\n");
    
    // Optional LED indicator
    init_led();
    
    // Create GPS test task
    xTaskCreate(gps_test_task, "gps_test", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Test task started. Monitor output below:");
    ESP_LOGI(TAG, "- All NMEA sentences will be printed");
    ESP_LOGI(TAG, "- GNGLL sentences will be highlighted");
    ESP_LOGI(TAG, "- Statistics printed every 30 seconds\n");
}