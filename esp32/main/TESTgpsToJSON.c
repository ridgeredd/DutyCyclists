#include "gpsToJSON.c"

int main(void) {
    double lat = 38.0197207;
    double lon = -78.3056908;
    time_t timestamp = time(NULL); //grabs current system time
    int id = 29;
    struct {
        double lat;
        double lon;
        long timestamp;
    } coord = {lat ,lon ,timestamp};
    write_coordinates_to_json(&coord, sizeof(coord), id ,0x7);

}