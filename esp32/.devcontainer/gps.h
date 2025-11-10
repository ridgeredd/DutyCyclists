#include <stdbool.h>

typedef struct {
    double latitude;   // decimal degrees (+north, -south)
    double longitude;  // decimal degrees (+east, -weset)
    double altitude;   // meters
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