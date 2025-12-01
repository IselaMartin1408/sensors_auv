// Código unificado: BNO055 + PMW3901 + BMP280 + GPS en un solo log
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

// --- INCLUDES ADICIONALES PARA BMP280 ---
#include "bmp280_simple.h" // Asumiendo que es el driver para 'bmx280'
#include "driver/i2c_types.h" // Necesario para la nueva API I2C si se usa, pero usaremos la antigua para simplificar la mezcla de código.

//------------------------------------------------------------
// PINOUT
//------------------------------------------------------------
#define I2C_MASTER_PORT I2C_NUM_0 // Puerto I2C a usar
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 20000
#define BNO055_SENSOR_ADDR 0x28

// Los pines para el BMP280 deben coincidir con los del BNO055 si están en el mismo bus I2C (22 y 21)
// #define BMX280_SDA_NUM GPIO_NUM_13 // NO USADO, se usa 21
// #define BMX280_SCL_NUM GPIO_NUM_14 // NO USADO, se usa 22
#define BMX280_I2C_ADDR 0x76 // Dirección I2C típica para BMP280/BMP280

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_SCLK 18
#define PIN_NUM_CS    5

#define GPS_UART_NUM UART_NUM_2
#define GPS_TX_PIN  17
#define GPS_RX_PIN  16
#define GPS_BAUD_RATE 38400

static const char *TAG_S = "SENSORS_ALL";
static gps_data_t gps;

//------------------------------------------------------------
// BNO055 INIT (I2C MASTER DRIVER)
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
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
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

// -----------------------------------------------------------
// BMX280/BMP280 SETUP
// -----------------------------------------------------------
// Usamos la inicialización I2C de bno055_setup (i2c_master_init_bno) para el bus.

static esp_err_t bmx280_setup(bmx280_t** bmx280_ptr) {
    // Dado que bmx280_create_master requiere i2c_master_bus_handle_t (nueva API)
    // y el resto del código usa la API antigua, necesitamos una forma de obtener el handle o usar una versión compatible.
    // Asumiendo que tenemos una función auxiliar que obtiene el handle o usamos una versión simplificada:
    // Para simplificar, asumiremos que se usa una implementación compatible con la API antigua o que 'bmx280_create_master'
    // puede ser adaptado, pero usaremos una estructura de inicialización más simple por ahora.

    // --- INICIALIZACIÓN BMP280 SIMPLIFICADA (requiere que el bus I2C ya esté instalado) ---
    // El código original usa 'bmx280_create_master', adaptaremos el código para llamarlo aquí si es posible.
    
    // NOTA: Para que esto funcione, 'i2c_new_master_bus' debe ser llamado en app_main.
    // Usaremos la nueva API I2C en app_main y eliminaremos la antigua 'i2c_master_init_bno' para consistencia.
    // **SIN EMBARGO, PARA MANTENER LA ESTRUCTURA DEL CÓDIGO ORIGINAL, DEJAMOS EL SETUP AQUÍ Y MODIFICAMOS APP_MAIN**
    
    // Usaremos la función de inicialización del código del usuario para ser precisos,
    // pero movida a 'sensors_task' o 'app_main'. La inicialización del driver I2C se hace una sola vez.

    *bmx280_ptr = bmx280_create_master_on_port(I2C_MASTER_PORT, BMX280_I2C_ADDR);
    if (!*bmx280_ptr) { 
        ESP_LOGE(TAG_S, "Could not create bmx280 driver.");
        return ESP_FAIL;
    }
    
    ESP_ERROR_CHECK(bmx280_init(*bmx280_ptr));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(*bmx280_ptr, &bmx_cfg));
    ESP_ERROR_CHECK(bmx280_setMode(*bmx280_ptr, BMX280_MODE_CYCLE));
    
    ESP_LOGI(TAG_S, "BMP280/BMX280 listo");
    return ESP_OK;
}

//------------------------------------------------------------
// TASK UNIFICADA BNO055 + PMW3901 + BMP280 + GPS
//------------------------------------------------------------
static void sensors_task(void *arg)
{
    struct bno055_euler e;
    float yaw=0, roll=0, pitch=0;
    
    float temp = 0, pres = 0, hum = 0; // BMP280 variables
    bmx280_t* bmx280 = NULL; // Handle para BMP280

    // --- IMU (BNO055) ---
    bno055_init_struct();
    // Se asume que i2c_master_init_bno() fue llamado en app_main
    ESP_ERROR_CHECK(bno055_setup());

    // --- BMP280/BMX280 ---
    // Usamos el mismo bus I2C (GPIO 21/22)
    ESP_ERROR_CHECK(bmx280_setup(&bmx280));

    // --- FLOW (PMW3901) ---
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
            yaw   = e.h / 16.0f;
            roll  = e.r / 16.0f;
            pitch = e.p / 16.0f;
        }

        // ---- FLOW ----
        int16_t dx = 0, dy = 0;
        pmw3901_read_motion_count(pmw, &dx, &dy);

        // ---- BMP280/BMX280 ----
        if (bmx280) {
            if (!bmx280_isSampling(bmx280)) {
                ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));
            }
        }

        // ---- GPS ----
        int len = uart_read_bytes(GPS_UART_NUM, gps_buf, sizeof(gps_buf), 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                gps_encode(&gps, gps_buf[i]);
            }
        }

        // ---- LOG UNIFICADO ----
        printf(
            "YAW,%.2f,ROLL,%.2f,PITCH,%.2f,DX,%d,DY,%d,LAT,%.6f,LON,%.6f,ALT,%.2f,SAT,%d,SPD,%.2f,DATE,%02d/%02d/%04d,TIME,%02d:%02d:%02d,TEMP,%.2f,PRES,%.2f\n",
            yaw, roll, pitch, dx, dy, gps.latitude, gps.longitude, gps.altitude, gps.satellites, gps.speed_kmph,
            gps.day, gps.month, gps.year, gps.hour, gps.minute, gps.second, temp, pres
        );

        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // NOTA: Nunca se llega aquí, pero si se hiciera, se debería liberar el driver I2C y el handle bmx280.
}

void app_main(void)
{
    // Inicialización del driver I2C (usado por BNO055 y BMP280)
    ESP_ERROR_CHECK(i2c_master_init_bno());
    
    // Creamos la tarea unificada de sensores
    xTaskCreate(sensors_task, "sensors_task", 8192, NULL, 5, NULL);
}