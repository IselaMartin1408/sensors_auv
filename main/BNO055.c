#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BNO055.h" // Asegúrate de que este archivo esté en la carpeta main/

//------------------------------BNO055--------------------------------
// ***************************************************************
// CONFIGURACIÓN DE HARDWARE
// ***************************************************************
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21

// Velocidad baja para estabilidad, confirmada: 24 kHz
#define I2C_MASTER_FREQ_HZ          24000
// Dirección I2C confirmada
#define BNO055_SENSOR_ADDR          0x28

static void bno055_init_struct(void) {
    bno055_device.dev_addr = BNO055_SENSOR_ADDR;
    bno055_device.bus_read = bno055_bus_read;
    bno055_device.bus_write = bno055_bus_write;
    bno055_device.delay_msec = bno055_delay_msec;
}

// Inicialización del bus I2C (se mantiene igual)
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Funciones de control del sensor (se mantienen igual)
static BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(unsigned char op_mode) {
    unsigned char data = OPERATION_MODE_CONFIG;
    if (bno055_bus_write(bno055_device.dev_addr, BNO055_OPR_MODE_ADDR, &data, 1) != SUCCESS) return ERROR1;
    bno055_delay_msec(20);
    data = op_mode;
    if (bno055_bus_write(bno055_device.dev_addr, BNO055_OPR_MODE_ADDR, &data, 1) != SUCCESS) return ERROR1;
    bno055_delay_msec(30);
    return SUCCESS;
}

// Configuración del sensor
static esp_err_t bno055_setup(void) {
    BNO055_RETURN_FUNCTION_TYPE com_rslt;
    unsigned char chip_id_data;
    
    // Espera inicial para que el sensor se estabilice (1 segundo)
    bno055_delay_msec(1000); 

    com_rslt = bno055_bus_read(bno055_device.dev_addr, BNO055_CHIP_ID_ADDR, &chip_id_data, 1);
    if (com_rslt != SUCCESS || chip_id_data != 0xA0) {
        ESP_LOGE(TAG, "BNO055 NOT FOUND! Chip ID: 0x%X (Expected 0xA0)", chip_id_data);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BNO055 detected! Chip ID: 0x%X", chip_id_data);
    
    // Resetear el sistema
    unsigned char rst_data = 0x20;
    bno055_bus_write(bno055_device.dev_addr, BNO055_SYS_TRIGGER_ADDR, &rst_data, 1);
    bno055_delay_msec(700); // Esperar el tiempo de inicio (650ms min)
    
    // Seleccionar la página 0
    unsigned char page_data = PAGE_ZERO;
    bno055_bus_write(bno055_device.dev_addr, BNO055_Page_ID_ADDR, &page_data, 1);
    
    // Configurar en modo NDOF
    com_rslt = bno055_set_operation_mode(OPERATION_MODE_NDOF);
    if (com_rslt != SUCCESS) {
        ESP_LOGE(TAG, "Failed to set NDOF mode.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BNO055 set to NDOF mode successfully.");
    return ESP_OK;
}

//------------------------------BNO055--------------------------------



// Función principal
void app_main(void) {
    struct bno055_euler euler_angles;
    float yaw_deg, roll_deg, pitch_deg;

    bno055_init_struct();
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C bus initialized.");

    // Los diagnósticos de escaneo y test de ID se pueden mantener o comentar
    i2c_scanner();
    test_chip_id(BNO055_SENSOR_ADDR);

    if (bno055_setup() != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 Setup Failed! Deteniendo programa.");
        return;
    }

    while (1) {
        if (bno055_read_euler_hrp(&euler_angles) == SUCCESS) {
            yaw_deg = (float)euler_angles.h / 16.0f;
            roll_deg = (float)euler_angles.r / 16.0f;
            pitch_deg = (float)euler_angles.p / 16.0f;
            
            ESP_LOGI(TAG, "Yaw: %7.2f° | Roll: %7.2f° | Pitch: %7.2f°",
                     yaw_deg, roll_deg, pitch_deg);
        } else {
            ESP_LOGW(TAG, "Failed to read Euler angles. (I2C Error)");
        }

        // ESPERA CORTA PARA LECTURA FLUIDA (10 Hz)
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}