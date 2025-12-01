#include "neo6m.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/_intsup.h>
#include <unistd.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "hal/uart_types.h"
#include "portmacro.h"
#include "esp_log.h"
#include "string.h"

#define GPS_UART_NUM      UART_NUM_2
#define GPS_TX_PIN        27      // ESP32 TX -> RX del GPS
#define GPS_RX_PIN        26      // ESP32 RX -> TX del GPS
#define GPS_BAUD_RATE     38400   // EVK-M101 por defecto si ya lo configuraste


static char buf[BUFFER]; 

static const char *TAG = "GPS_MODULE";

gps_data_t gps;

void init_gps_uart() {
    uart_config_t cfg = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(GPS_UART_NUM, &cfg);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, 2048, 0, 0, NULL, 0);
}


void gps_start(void)
{   
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
   
    uart_param_config(GPS_UART_NUM, &cfg);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, 2048, 0, 0, NULL, 0);
}

void raw_nmea(void)
{ 
    memset(gps, 0, sizeof(gps_data_t));
	
     int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
   ESP_LOGI(TAG, "%s", buf);
   
}

void parse_nmea(const char *buf, char *latitude, char *longitude) {
    const char *start;
    char lat[12], lon[12];

  
    start = strstr(buf, "$GPGGA");
    if (start != NULL) {
        sscanf(start, "$GPGGA,%*f,%11[^,],%*c,%11[^,],%*c", lat, lon);
        sprintf(latitude, "%s", lat);
        sprintf(longitude, "%s", lon);
    }
}

void lat_long(void)
{
    
    char lat[12];
    char lon[12];
    void parse_nmea(const char *buf, char *lat, char *lon);
    ESP_LOGI(TAG, "Latitude is:%s\n", lat);
    ESP_LOGI(TAG, "Longitude is:%s\n", lon);
  }

void time(void)
{
    
    char time[20];
    sscanf(buf, "$GPRMC,%10[^,]", time); 
    ESP_LOGI(TAG, "Time is %s", time); 
    memset(time, 0, 20);
}

void speed_course(void)
{
    
    float speedKmh;
    float course;
    sscanf(buf, "$GPVTG,%*f,%*c,%*f,%*c,%*f,%*c,%f", &speedKmh); 
    sscanf(buf, "$GPVTG,%f", &course); 
    sscanf(buf, "$GPRMC,%*f,%*c,%*f,%*c,%*f,%*c,%*f,%f", &course); 
    ESP_LOGI(TAG, "Speed is %f", speedKmh); 
    ESP_LOGI(TAG, "Course is %f", course); 
   
   
}
