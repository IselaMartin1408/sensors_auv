// main.c - example usage of pmw3901 driver in ESP-IDF
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "flow.h"

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_SCLK 18
#define PIN_NUM_CS   5   // ajusta a tu conexión

static const char *TAG = "PMW_EXAMPLE";

void app_main(void)
{
    esp_err_t ret;

    // 1) configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1); // use VSPI_HOST or HSPI_HOST
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    // 2) init device
    pmw3901_handle_t *pmw = NULL;
    ret = pmw3901_init(VSPI_HOST, PIN_NUM_CS, &pmw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pmw3901_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "PMW3901 inited OK");

    while (1) {
        int16_t dx = 0, dy = 0;
        if (pmw3901_read_motion_count(pmw, &dx, &dy) == ESP_OK) {
            ESP_LOGI(TAG, "DX=%d DY=%d", dx, dy);
        } else {
            ESP_LOGW(TAG, "read_motion_count failed");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // never reached
    pmw3901_deinit(pmw);
}
