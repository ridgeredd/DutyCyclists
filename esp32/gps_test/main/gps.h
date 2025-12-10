#include <stdbool.h>

#define GPS_UART_NUM      UART_NUM_2 // use UART2 for flexibility
#define GPS_TX_PIN        17   // esp32 -> gnss (RX)
#define GPS_RX_PIN        16   // esp32 <- gnss (TX)
#define GPS_BAUD_RATE     9600
#define GPS_BUF_SIZE      1024

typedef struct {
    double latitude;   // decimal degrees (+north, -south)
    double longitude;  // decimal degrees (+east, -weset)
    double time;   // hhmmss.sss UTC
    bool valid;        // true if fix (location data) is valid & recent
    uint32_t fix_age_ms; // ms since the fix was received
} gnss_data_t;

/**
 * starts the GPS UART task. Call once during init
 */
void gps_start_task(void);

/**
 * 
 * returns true if a valid fix was returned (out filled), false otherwise.
 */
bool gps_get_fix(gnss_data_t *out);

