#include "gps.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "json_object.h"

static gnss_data_t byteToGPS(uint8_t *bytes, size_t length){
    gnss_data_t data;
    
    // decode data 
    // currently: reads from memory using pointer and use length for blocking size
    
    //return struct of data 

    return data;
}

// additional function for converting struct to json string
// for function, edit json per id if existing, else make new json 

