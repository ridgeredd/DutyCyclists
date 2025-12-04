#include "direwolf.h"
#include <stdint.h>
#include <stdio.h>
#include "duty_cyclists_decoder.h"

unsigned char DUTY_CYCLISTS_RADIO_ID = 0x67;

//const static char[] filename = "gps_data/{}.json";

/*
 *  frame_buf a frame of bytes after NRZI, bit unstuffing, and removing start/stop
 *  frame_len is length of frame_buf
 *  
*/
void duty_cyclists_decode(unsigned char frame_buf[], int frame_len) {

    //printf("Frame Length: %d\n", frame_len);

    // Filter out noise that does not match our packet length
    if (frame_len != DUTY_CYCLISTS_PACKET_LEN + 2) { return; }

    // Filter out noise that does not match a listed radio (in this case there is only 1)
    if (frame_buf[0] != DUTY_CYCLISTS_RADIO_ID) { return; }

    // Filter 

    // RS Error Correction

    // Extract Radio ID
    //uint8_t id = 0;

    // Extract Latitude
    //uint32_t lat = 0;

    // Extract Longitude
    //uint32_t lng = 0;

    // Create time stamp
    //uint32_t time = 0;

    printf("Raw Bytes\n");
    for (int i = 0; i < frame_len; i++) {
        printf("%02X ", frame_buf[i]);
    }
    printf("\n");

}