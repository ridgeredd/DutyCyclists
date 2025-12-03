#ifndef DUTY_CYCLISTS_DECODER_H
#define DUTY_CYCLISTS_DECODER_H

/*
 *  frame_buf a frame of bytes after NRZI, bit unstuffing, and removing start/stop
 *  frame_len is length of frame_buf
 *  calls another function to add to a json file
*/
void duty_cyclists_decode(unsigned char frame_buf[], int frame_len);

// Right now, we only have one radio in network
extern unsigned char DUTY_CYCLISTS_RADIO_ID;

/*
Duty Cyclists packet standard:
_________________________________________________________________________
|               |               |               |                       |
|   Byte 0      |   Bytes 1-5   |   Bytes 6-10  |   Bytes 11-XX         |
|   Radio ID    |   Latitude    |   Longitude   |   Error Correction    |
|_______________|_______________|_______________|_______________________|

*/

//#include <json-c/json.h>

#define RS_BYTES                    4
#define DUTY_CYCLISTS_PACKET_LEN     1 + 4 + 4 + RS_BYTES

#endif
