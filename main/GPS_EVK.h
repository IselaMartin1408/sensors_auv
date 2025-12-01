#ifndef GPS_EVK_H
#define GPS_EVK_H

#include <stdbool.h>

typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double speed_kmph;
    double course_deg;
    int satellites;
    int day, month, year;
    int hour, minute, second;
    bool valid_fix;
} gps_data_t;

void gps_init(gps_data_t *gps);
void gps_encode(gps_data_t *gps, char c);

#endif
