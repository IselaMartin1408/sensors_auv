// pmw3901.h - PMW3901 driver for ESP-IDF (C)
#ifndef flow_H
#define flow_H

#include <stdint.h>
#if __has_include(<driver/spi_master.h>)
#include <driver/spi_master.h>
#elif __has_include(<esp_driver/spi_master.h>)
#include <esp_driver/spi_master.h>
#else
#error "spi_master.h not found; adjust include paths or install ESP-IDF SDK"
#endif

typedef struct {
    spi_device_handle_t spi;   // handle SPI device
    int cs_gpio;               // CS pin (optionally used)
} pmw3901_handle_t;

/**
 * @brief Initialize PMW3901 on given SPI host and CS pin
 * @param host SPI host (e.g., HSPI_HOST or VSPI_HOST)
 * @param cs_gpio CS GPIO number (or -1 if using spi_bus_add_device's cs)
 * @param out_handle pointer to returned handle (must be freed with pmw3901_deinit)
 * @return ESP_OK on success
 */
esp_err_t pmw3901_init(spi_host_device_t host, int cs_gpio, pmw3901_handle_t **out_handle);

/**
 * @brief Deinitialize driver and free handle
 */
void pmw3901_deinit(pmw3901_handle_t *h);

/**
 * @brief Read 8-bit register
 */
esp_err_t pmw3901_register_read(pmw3901_handle_t *h, uint8_t reg, uint8_t *value);

/**
 * @brief Write 8-bit register
 */
esp_err_t pmw3901_register_write(pmw3901_handle_t *h, uint8_t reg, uint8_t value);

/**
 * @brief Read motion counts (deltaX, deltaY) as int16
 */
esp_err_t pmw3901_read_motion_count(pmw3901_handle_t *h, int16_t *deltaX, int16_t *deltaY);

/**
 * @brief Enable frame buffer mode (if needed)
 */
esp_err_t pmw3901_enable_frame_buffer(pmw3901_handle_t *h);

/**
 * @brief Read one frame into provided buffer (size expected by caller)
 */
esp_err_t pmw3901_read_frame_buffer(pmw3901_handle_t *h, uint8_t *frame_buffer, size_t len);

/**
 * @brief Initialize sensor performance registers (call after reset)
 */
esp_err_t pmw3901_init_registers(pmw3901_handle_t *h);

#endif // PMW3901_H
