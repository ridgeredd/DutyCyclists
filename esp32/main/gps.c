#include "gps.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <ctype.h>

static const char *TAG = "gps_uart";

/* config the UART pins */
#define GPS_UART_NUM      UART_NUM_2 // use UART2 for flexibility
#define GPS_TX_PIN        17   // esp32 -> gnss (RX)
#define GPS_RX_PIN        16   // esp32 <- gnss (TX)
#define GPS_BAUD_RATE     9600
#define GPS_BUF_SIZE      1024
#define GPS_DELAY_MS      50

/* latest fix + synchronization */
static gnss_data_t latest_fix = {0};
static SemaphoreHandle_t latest_fix_mutex = NULL;
static uint64_t latest_fix_time_us = 0; // timestamp when fix was set (0 ms)


static void set_latest_fix(const gnss_data_t *in)
{
    if (!latest_fix_mutex) return;
    if (xSemaphoreTake(latest_fix_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        latest_fix = *in;
        latest_fix_time_us = esp_timer_get_time();
        xSemaphoreGive(latest_fix_mutex);
    }
}

static bool gps_get_fix(gnss_data_t *out)
{
    if (!latest_fix_mutex) return false;
    bool ok = false;
    if (xSemaphoreTake(latest_fix_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *out = latest_fix;
        uint64_t now_us = esp_timer_get_time();
        out->fix_age_ms = (uint32_t)((now_us - latest_fix_time_us) / 1000ULL); // age in ms since fix was set
        ok = latest_fix.valid;
        xSemaphoreGive(latest_fix_mutex);
    }
    return ok;
}

/* converts NMEA lat/lon into decimal degrees */
static bool nmea_to_deg(const char *field, const char *hemisphere, double *out_deg)
{
    if (!field || !*field) return false;
    double v = atof(field);
    int degrees = (int)(v / 100.0);
    double minutes = v - (degrees * 100.0);
    double deg = degrees + (minutes / 60.0);
    if (hemisphere && hemisphere[0]) {
        if (toupper((unsigned char)hemisphere[0]) == 'S' || toupper((unsigned char)hemisphere[0]) == 'W') deg = -deg;
    }
    *out_deg = deg;
    return true;
}

/* small GLL parser: expects a NMEA line beginning with $GP/GN GLL */
static bool parse_gll(const char *line, gnss_data_t *out)
{
    // Copy to local buffer to use strtok safely
    char buf[160];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    // tokens: $GNGGA,time,lat,NS,lon,EW,fix,num_sat,hdop,alt,units,...
    char *tokens[16] = {0};
    int ti = 0;
    char *p = strtok(buf, ",");
    while (p && ti < 16) {
        tokens[ti++] = p;
        p = strtok(NULL, ",");
    }
    if (ti < 7) return false;

    /* usual tokens from a GNGGA sentence */
        // tokens[0] = $GNGGA
        // tokens[1] = time hhmmss.sss
        // tokens[2] = latitude ddmm.mmmm
        // tokens[3] = North/South
        // tokens[4] = longitude dddmm.mmmm
        // tokens[5] = East/West
        // tokens[6] = fix quality (0 = invalid, 1 = GPS fix, 2 = DGPS fix, etc)
        // tokens[7] = number of satellites
        // tokens[9] = altitude (meters)
    
    /* usual tokens from a GNGLL sentence */
        // tokens[0] = $GNGLL
        // tokens[1] = latitude ddmm.mmmm
        // tokens[2] = North/South
        // tokens[3] = longitude dddmm.mmmm
        // tokens[4] = East/West
        // tokens[5] = time hhmmss.sss
        // tokens[6] = status A=valid, V=invalid
        
    if (!tokens[1] || !tokens[3]) return false;
    if (tokens[6][0] == 'V') return false; // invalid

    gnss_data_t res = {0};
    if (!nmea_to_deg(tokens[1], tokens[2], &res.latitude)) return false;
    if (!nmea_to_deg(tokens[3], tokens[4], &res.longitude)) return false;
    // res.time = atof(tokens[5]); 
    res.valid = true;
    res.fix_age_ms = 0;
    *out = res;
    return true;
}

/* read UART, accumulate lines, parse GGA */
static void gps_task(void *arg)
{
    ESP_LOGI(TAG, "Starting GPS UART task (baud=%d)", GPS_BAUD_RATE);

    // init mutex
    latest_fix_mutex = xSemaphoreCreateMutex();

    // init uart
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);

    // buffer for incoming bytes and assembling lines
    uint8_t *uart_buf = (uint8_t*)malloc(GPS_BUF_SIZE);
    char linebuf[256];
    size_t line_pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, uart_buf, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(500));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)uart_buf[i];
                // NMEA sentences end in \r\n; accept either
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        linebuf[line_pos] = '\0';
                        // process line
                        if (linebuf[0] == '$') {
                            // only handle GLL for now
                            if ((strncmp(linebuf + 1, "GNGLL", 5) == 0) || (strncmp(linebuf + 1, "GPGLL", 5) == 0)) {
                                gnss_data_t fix;
                                if (parse_gll(linebuf, &fix)) {
                                    set_latest_fix(&fix);
                                    ESP_LOGI(TAG, "GPS fix: lat=%.6f lon=%.6f", fix.latitude, fix.longitude);
                                }
                            }
                        }
                        line_pos = 0;
                    }
                } else {
                    if (line_pos < sizeof(linebuf)-1) {
                        linebuf[line_pos++] = c;
                    } else {
                        // overflow - reset
                        line_pos = 0;
                    }
                }
            }
        }
        // loop delay small to yield CPU
        vTaskDelay(pdMS_TO_TICKS(GPS_DELAY_MS));
    }
}

//==================================================== API ===============================================================

/* starter */
void gps_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));  // let GPS boot
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}

// TODO: change to boolean; delay if gps outdated
void get_gps(uint8_t *bytes) {

    // for (uint8_t count = 0; count < PAYLOAD_BYTES - 1; count++) {
    //     bytes[count] = count;
    //     //32 bit lat 32 bit long
    // }

    gnss_data_t fix;
    if(!gps_get_fix(&fix)) {
        // no GPS fix; send zeros
        memset(bytes, 0, 8);
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