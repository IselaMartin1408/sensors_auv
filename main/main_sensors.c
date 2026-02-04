// Código unificado: BNO055 + PMW3901 + BMP280 en un solo log
// -----------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "BNO055.h"
#include "flow.h"
#include "driver/uart.h"
#include "GPS_EVK.h"

//------------------------------------------------------------
// PINOUT
//------------------------------------------------------------
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 20000
#define BNO055_SENSOR_ADDR 0x28

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_SCLK 18
#define PIN_NUM_CS    5

#define GPS_UART_NUM UART_NUM_2
#define GPS_TX_PIN  17
#define GPS_RX_PIN  16
#define GPS_BAUD_RATE 38400

static const char *TAG_S = "SENSORS_ALL";
static gps_data_t gps;

//------------------------------------------------------------
// BNO055 INIT
//------------------------------------------------------------
static void bno055_init_struct(void) {
    bno055_device.dev_addr = BNO055_SENSOR_ADDR;
    bno055_device.bus_read = bno055_bus_read;
    bno055_device.bus_write = bno055_bus_write;
    bno055_device.delay_msec = bno055_delay_msec;
}

static esp_err_t i2c_master_init_bno(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

static void init_gps_uart(void) {
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
    ESP_LOGI(TAG_S, "GPS UART inicializado en puerto %d", GPS_UART_NUM);
}

static esp_err_t bno055_setup(void) {
    unsigned char chip_id = 0;

    bno055_delay_msec(1000);
    bno055_bus_read(BNO055_SENSOR_ADDR, BNO055_CHIP_ID_ADDR, &chip_id, 1);

    if (chip_id != 0xA0) {
        ESP_LOGE(TAG_S, "BNO055 NOT FOUND! Chip=0x%02X", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_S, "BNO055 detected (ID=0x%02X)", chip_id);

    unsigned char rst = 0x20;
    bno055_bus_write(BNO055_SENSOR_ADDR, BNO055_SYS_TRIGGER_ADDR, &rst, 1);
    bno055_delay_msec(700);

    unsigned char page0 = 0x00;
    bno055_bus_write(BNO055_SENSOR_ADDR, BNO055_Page_ID_ADDR, &page0, 1);

    unsigned char mode = OPERATION_MODE_NDOF;
    bno055_bus_write(BNO055_SENSOR_ADDR, BNO055_OPR_MODE_ADDR, &mode, 1);
    bno055_delay_msec(30);

    ESP_LOGI(TAG_S, "BNO055 NDOF mode ready");
    return ESP_OK;
}

//------------------------------------------------------------
// TASK UNIFICADA BNO055 + PMW3901
//------------------------------------------------------------
static void sensors_task(void *arg)
{
    struct bno055_euler e;
    float yaw=0, roll=0, pitch=0;

    // --- IMU ---
    bno055_init_struct();
    ESP_ERROR_CHECK(i2c_master_init_bno());
    ESP_ERROR_CHECK(bno055_setup());

    // --- FLOW ---
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));

    pmw3901_handle_t *pmw = NULL;
    ESP_ERROR_CHECK(pmw3901_init(VSPI_HOST, PIN_NUM_CS, &pmw));
    ESP_LOGI(TAG_S, "PMW3901 listo");

    // --- GPS ---
    gps_init(&gps);
    init_gps_uart();
    ESP_LOGI(TAG_S, "GPS listo");

    uint8_t gps_buf[512];

    while (1) {
        // ---- IMU ----
        if (bno055_read_euler_hrp(&e) == SUCCESS) {
            yaw   = e.h / 16.0f;
            roll  = e.r / 16.0f;
            pitch = e.p / 16.0f;
        }

        // ---- FLOW ----
        int16_t dx = 0, dy = 0;
        pmw3901_read_motion_count(pmw, &dx, &dy);

        // ---- GPS ----
        int len = uart_read_bytes(GPS_UART_NUM, gps_buf, sizeof(gps_buf), 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                gps_encode(&gps, gps_buf[i]);
            }
        }

        // ---- LOG UNIFICADO ----
        printf(
            "YAW,%.2f,ROLL,%.2f,PITCH,%.2f,DX,%d,DY,%d,LAT,%.6f,LON,%.6f,ALT,%.2f,SAT,%d,SPD,%.2f,DATE,%02d/%02d/%04d,V, TIME,%02d:%02d:%02d\n",
            yaw, roll, pitch, dx, dy, gps.latitude, gps.longitude, gps.altitude, gps.satellites, gps.speed_kmph,
            gps.day, gps.month, gps.year, gps.hour, gps.minute, gps.second
        );

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    xTaskCreate(sensors_task, "sensors_task", 8192, NULL, 5, NULL);
}
