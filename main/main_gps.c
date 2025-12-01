#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "GPS_EVK.h"

#define GPS_UART_NUM UART_NUM_2     // UART2 to avoid conflicts with UART1
#define GPS_TX_PIN  17             // ESP32 TX2 -> GPS RX (optional, for sending commands)
#define GPS_RX_PIN  16             // ESP32 RX2 -> GPS TX (receives NMEA data)
#define GPS_BAUD_RATE 38400         // Standard for NEO-6M (change to 38400 for EVK-M101)

static const char *TAG = "GPS_MAIN";

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

void print_gps_data() {
    ESP_LOGI(TAG, "==================================================");

    ESP_LOGI(TAG, "ESP POSICIÓN Y CALIDAD DEL FIX");
    ESP_LOGI(TAG, "Latitud:   %.6f", gps.latitude);
    ESP_LOGI(TAG, "Longitud:  %.6f", gps.longitude);
    ESP_LOGI(TAG, "Altitud:   %.2f m", gps.altitude);
    ESP_LOGI(TAG, "Satélites: %d", gps.satellites);

    ESP_LOGI(TAG, "--------------------------------------------------");
    ESP_LOGI(TAG, "Velocidad: %.2f km/h", gps.speed_kmph);
    ESP_LOGI(TAG, "Rumbo:     %.2f°", gps.course_deg);

    ESP_LOGI(TAG, "--------------------------------------------------");
    ESP_LOGI(TAG, "ESP FECHA Y HORA (UTC)");
    ESP_LOGI(TAG, "Fecha: %02d/%02d/%04d",
             gps.day, gps.month, gps.year);

    ESP_LOGI(TAG, "Hora:  %02d:%02d:%02d",
             gps.hour, gps.minute, gps.second);

    ESP_LOGI(TAG, "==================================================");
}

void app_main() {
    gps_init(&gps);
    init_gps_uart();

    ESP_LOGI(TAG, "GPS inicializado en UART%d (RX:%d, TX:%d, Baud:%d)",
             GPS_UART_NUM, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
    ESP_LOGI(TAG, "Esperando datos NMEA... (coloca GPS con vista al cielo)");

    uint8_t buf[512];  // Increased buffer size
    uint32_t no_data_count = 0;
    uint32_t last_fix_time = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), 200 / portTICK_PERIOD_MS);

        if (len > 0) {
            no_data_count = 0;
            
            // Log raw data for debugging (first 30 seconds or when no fix)
            if (xTaskGetTickCount() < pdMS_TO_TICKS(30000) || !gps.valid_fix) {
                buf[len < sizeof(buf) ? len : sizeof(buf)-1] = '\0';
                ESP_LOGD(TAG, "Raw NMEA [%d bytes]: %s", len, buf);
            }
            
            for (int i = 0; i < len; i++) {
                gps_encode(&gps, buf[i]);
            }
        } else {
            no_data_count++;
            if (no_data_count > 10) {
                ESP_LOGW(TAG, "No hay datos del GPS. Verifica conexión TX->RX");
                no_data_count = 0;
            }
        }

        if (gps.valid_fix) {
            // Print full data every 5 seconds when fix is valid
            if ((xTaskGetTickCount() - last_fix_time) > pdMS_TO_TICKS(5000)) {
                print_gps_data();
                last_fix_time = xTaskGetTickCount();
            }
        } else {
            ESP_LOGI(TAG, "Esperando Fix... Satélites: %d (necesita ≥4 para fix)", gps.satellites);
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // Check more frequently
    }
}
