#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

typedef struct {
    double latitude;   // decimal degrees (+north, -south)
    double longitude;  // decimal degrees (+east, -weset)
    double timestamp;   // hhmmss.sss UTC
    bool valid;        // true if fix (location data) is valid & recent
    int id;
    uint32_t fix_age_ms; // ms since the fix was received
} gnss_data_t;


/**
 * 
 * returns true if a valid fix was returned (out filled), false otherwise.
 */
//bool gps_get_fix(gnss_data_t *out);

#endif
