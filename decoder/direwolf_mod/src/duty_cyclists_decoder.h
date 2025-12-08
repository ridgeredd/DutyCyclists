#ifndef DUTY_CYCLISTS_DECODER_H
#define DUTY_CYCLISTS_DECODER_H

/*
 *  frame_buf a frame of bytes after NRZI, bit unstuffing, and removing start/stop
 *  frame_len is length of frame_buf
 *  calls another function to add to a json file
*/
int duty_cyclists_decode(unsigned char frame_buf[], int frame_len);

// Right now, we only have one radio in network
extern unsigned char DUTY_CYCLISTS_RADIO_ID;

/*
Duty Cyclists packet standard:
_________________________________________________________________________
|               |               |               |                       |
|   Byte 0      |   Bytes 1-5   |   Bytes 6-10  |   Bytes 11-17         |
|   Radio ID    |   Latitude    |   Longitude   |   Error Correction    |
|_______________|_______________|_______________|_______________________|

Each packet begins with two 0x7E flags, and ends with one

*/

//#include <json-c/json.h>

#define RS_BYTES                    6
#define DUTY_CYCLISTS_PACKET_LEN    1 + 4 + 4 + RS_BYTES
#define DUTY_CYCLISTS_PAYLOAD_LEN   1 + 4 + 4

#endif
