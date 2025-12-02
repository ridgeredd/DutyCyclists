#include "gps.h"
#include "const.h"

// Interfaces with gps module and receives gps data
// TODO: implement
// currently stub
void get_gps(uint8_t *bytes) {
    for (uint8_t count = 0; count < PAYLOAD_BYTES; count++) {
        bytes[count] = count;
    }
}

