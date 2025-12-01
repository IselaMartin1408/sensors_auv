

#ifndef NEO6M_H_
#define NEO6M_H_

#include <stdio.h>
#include <sys/_intsup.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/ledc_types.h"
#include "portmacro.h"
#include "esp_log.h"

void raw_nmea(void);
void gps_start(void);

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
