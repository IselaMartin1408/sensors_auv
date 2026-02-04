#include <stdio.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <bmp280.h>
#include "i2cdev.h"
#include <string.h>
#include <math.h>

#define I2C_BUS_PORT I2C_NUM_1
#define I2C_SDA_GPIO 13
#define I2C_SCL_GPIO 14

#ifndef SEA_LEVEL_PRESSURE
#define SEA_LEVEL_PRESSURE 101325.0f
#endif

static const char *TAG = "BMP280_DEMO";
static bmp280_t sensor;

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando I2C...");
    i2cdev_init();
    memset(&sensor, 0, sizeof(sensor));

    bmp280_init_desc(&sensor, BMP280_I2C_ADDRESS_0, I2C_BUS_PORT,
                     I2C_SDA_GPIO, I2C_SCL_GPIO);

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_init(&sensor, &params);

    ESP_LOGI(TAG, "Esperando estabilización del sensor (1 segundo)...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ==========================================
    //     DESCARTAR PRIMERAS LECTURAS
    // ==========================================
    ESP_LOGI(TAG, "Descartando lecturas iniciales...");
    float t, p, h;

    for (int i = 0; i < 15; i++) {  // ← descartar ruido inicial
        bmp280_read_float(&sensor, &t, &p, &h);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ==========================================
    //     PROMEDIAR 30 LECTURAS ESTABLES
    // ==========================================
    ESP_LOGI(TAG, "Calculando altitud inicial (promedio de 30 muestras)...");

    float sum_alt = 0.0f;
    int count = 0;

    while (count < 30) {

        if (bmp280_read_float(&sensor, &t, &p, &h) == ESP_OK) {

            // validar presión razonable
            if (p > 60000 && p < 110000) {
                float alt = 44330.0f * (1.0f - powf(p / SEA_LEVEL_PRESSURE, 0.190294957f));
                sum_alt += alt;
                count++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    float altitude_initial = sum_alt / 30.0f;

    ESP_LOGI(TAG, "ALTITUD DE REFERENCIA = %.2f m", altitude_initial);

    // ====================================================
    //              LOOP PRINCIPAL
    // ====================================================
    while (1) {
        float temperature, pressure, humidity;
        vTaskDelay(pdMS_TO_TICKS(100));

        if (bmp280_read_float(&sensor, &temperature, &pressure, &humidity) != ESP_OK)
            continue;

        if (pressure < 60000 || pressure > 110000)
            continue;

        float altitude =
            44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190294957f));

        float relative_height = altitude - altitude_initial;

        ESP_LOGW(TAG,
                 "Temp, %.2f, Presion, %.2f, Altitud, %.2f, Altura relativa, %.2f",
                 temperature, pressure, altitude, relative_height);
    }
}
