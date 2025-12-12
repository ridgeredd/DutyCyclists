#include "direwolf.h"
#include "gps.h"
#include <stdint.h>
#include <stdio.h>
#include "duty_cyclists_decoder.h"
#include "fx25.h"
#include "json_object.h"
#include "gpsToJSON.c"
#include <time.h>

const uint8_t DUTY_CYCLISTS_RADIO_ID = 0x67;

#define PARITY_BYTES                   6      // These must sum to < 255
#define PAYLOAD_BYTES                  9      // payload + callsign      
    
#define START_NBYTES                   2      // 6 start bytes in a row
#define END_NBYTES                     1      // 4 end bytes
#define TRANSMISSION_BYTES             PAYLOAD_BYTES + PARITY_BYTES + START_NBYTES + END_NBYTES

static int is_rs_init = 0;
static struct rs *rs_encoder = NULL;

//const static char[] filename = "gps_data/{}.json";

static void init_rs() {

    is_rs_init = 1;

    // Initialize reed-solomon encoder. Requirements for library specified here: 
    // https://manpages.debian.org/unstable/libfec-dev/rs.3.en.html?utm_source=chatgpt.com
    rs_encoder = INIT_RS(
        8, // gives the symbol size in bits, up to 32
        0x11D, // gives the extended Galois field generator polynomial coefficients, with the 0th coefficient in the low order bit. The polynomial must be primitive; if not, the call will fail and NULL will be returned.
		0, // gives, in index form, the first consecutive root of the Reed Solomon code generator polynomial
        1, // gives, in index form, the primitive element in the Galois field used to generate the Reed Solomon code generator polynomial.
        PARITY_BYTES // gives the number of roots in the Reed Solomon code generator polynomial. This equals the number of parity symbols per code block.
		//255 - PARITY_BYTES - PAYLOAD_BYTES // gives the number of leading symbols in the codeword that are implicitly padded to zero in a shortened code block.
        // The resulting Reed-Solomon code has parameters (N,K), where N = 2^symsize - pad - 1 and K = N-nroots.
    );
}


/*
 *  frame_buf a frame of bytes after NRZI, bit unstuffing, and removing start/stop
 *  frame_len is length of data stream
 *  
*/
int duty_cyclists_decode(unsigned char frame[], int frame_len) {

    if( !is_rs_init ) {
        init_rs();
    }

    if ( frame_len != DUTY_CYCLISTS_PACKET_LEN ) { return 0; }

    int success = DECODE_RS(rs_encoder, frame, NULL, 0);

    if( !success ) { return 0; }

    if ( frame[0] != DUTY_CYCLISTS_RADIO_ID ) { return 0; }

    printf("Success!\n");
    for (int i = 0; i < DUTY_CYCLISTS_PAYLOAD_LEN; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n\n");
    
    uint8_t id = frame[0];
    uint32_t lat_int =  frame[4] << 24 |
                        frame[3] << 16 |
                        frame[2] << 8 |
                        frame[1];

    uint32_t lon_int =  frame[8] << 24 |
                        frame[7] << 16 |
                        frame[6] << 8  |
                        frame[5];

    float lat = ((double)lat_int * 180.0 / 4294967295.0) - 90.0;
    float lon = ((double)lon_int * 360.0 / 4294967295.0) - 180.0;

    time_t timestamp = time(NULL); //grabs current system time
    
    gnss_data_t coord = { 0 };
    coord.latitude = lat;
    coord.longitude = lon;
    coord.id = id;
    coord.timestamp = timestamp;
    write_coordinates_to_json(&coord, sizeof(coord), id ,0x7);

    printf("lat: %f, lon: %f\n", lat, lon);

    return 1;
    

    //printf("Frame Length: %d\n", frame_len);

    // Filter out noise that does not match our packet length
    //if (frame_len != DUTY_CYCLISTS_PACKET_LEN + 2) { return; }

    // Filter out noise that does not match a listed radio (in this case there is only 1)
    //if (frame_buf[0] != DUTY_CYCLISTS_RADIO_ID) { return; }

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

}