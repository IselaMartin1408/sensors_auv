// pmw3901.c - PMW3901 driver for ESP-IDF (C)
#include "flow.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char *TAG = "PMW3901";

/* SPI settings chosen similar to Arduino driver */
#define PMW3901_SPI_FREQUENCY_HZ 4000000  // 4 MHz
#define PMW3901_SPI_MODE          3

// Helper: small delay microseconds
static inline void delay_us(uint32_t us) {
    // vTaskDelay gives ms resolution, use busy-loop for small us delays
    esp_rom_delay_us(us);
}

esp_err_t pmw3901_init(spi_host_device_t host, int cs_gpio, pmw3901_handle_t **out_handle) {
    if (!out_handle) return ESP_ERR_INVALID_ARG;

    esp_err_t ret;
    pmw3901_handle_t *h = calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;
    h->cs_gpio = cs_gpio;

    // Configure SPI device interface
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = PMW3901_SPI_FREQUENCY_HZ,
        .mode = PMW3901_SPI_MODE,
        .spics_io_num = cs_gpio >= 0 ? cs_gpio : -1, // if -1, user must manage CS externally
        .queue_size = 3,
        .flags = 0,
    };

    ret = spi_bus_add_device(host, &devcfg, &h->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        free(h);
        return ret;
    }

    // If cs_gpio was provided and spics_io_num used - it's handled by the SPI driver.
    // do power on reset sequence similar to Arduino: write 0x5A to 0x3A
    ret = pmw3901_register_write(h, 0x3A, 0x5A);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Power-on reset write failed (may be transient)");
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Read chip id and inverse chip id (0x00 and 0x5F)
    uint8_t chipId=0, dIpihc=0;
    pmw3901_register_read(h, 0x00, &chipId);
    pmw3901_register_read(h, 0x5F, &dIpihc);

    ESP_LOGI(TAG, "chipId=0x%02X inv=0x%02X", chipId, dIpihc);

    // read motion registers once to clear
    uint8_t tmp;
    pmw3901_register_read(h, 0x02, &tmp);
    pmw3901_register_read(h, 0x03, &tmp);
    pmw3901_register_read(h, 0x04, &tmp);
    pmw3901_register_read(h, 0x05, &tmp);
    pmw3901_register_read(h, 0x06, &tmp);
    vTaskDelay(pdMS_TO_TICKS(1));

    // init performance registers (like Arduino)
    pmw3901_init_registers(h);

    *out_handle = h;
    return ESP_OK;
}

void pmw3901_deinit(pmw3901_handle_t *h) {
    if (!h) return;
    if (h->spi) {
        spi_bus_remove_device(h->spi);
    }
    free(h);
}

static esp_err_t pmw3901_transfer(spi_device_handle_t spi, const uint8_t *tx, uint8_t *rx, size_t len) {
    if (!spi || len==0) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_transmit(spi, &t);
}

esp_err_t pmw3901_register_write(pmw3901_handle_t *h, uint8_t reg, uint8_t value) {
    if (!h) return ESP_ERR_INVALID_ARG;
    reg |= 0x80u; // set MSB for write

    // Create tx buffer [reg, value]
    uint8_t tx[2] = { reg, value };
    esp_err_t ret = pmw3901_transfer(h->spi, tx, NULL, sizeof(tx));
    // after write sensor requires some delays
    delay_us(200); // 200 us similar to Arduino delays
    return ret;
}

esp_err_t pmw3901_register_read(pmw3901_handle_t *h, uint8_t reg, uint8_t *value) {
    if (!h || !value) return ESP_ERR_INVALID_ARG;
    reg &= ~0x80u; // clear MSB for read

    uint8_t tx[2] = { reg, 0x00 };
    uint8_t rx[2] = {0};
    // According to sensor timing, read needs a small pause between address and data
    // perform two transfers to emulate the delays and behavior
    esp_err_t ret = pmw3901_transfer(h->spi, tx, rx, 2);
    if (ret != ESP_OK) return ret;
    // The returned second byte is the read data (rx[1])
    *value = rx[1];
    delay_us(100);
    return ESP_OK;
}

esp_err_t pmw3901_read_motion_count(pmw3901_handle_t *h, int16_t *deltaX, int16_t *deltaY) {
    if (!h || !deltaX || !deltaY) return ESP_ERR_INVALID_ARG;
    uint8_t tmp;

    // registers: 0x02 motion, 0x03 deltaX_L, 0x04 deltaX_H, 0x05 deltaY_L, 0x06 deltaY_H
    pmw3901_register_read(h, 0x02, &tmp);
    uint8_t xl, xh, yl, yh;
    pmw3901_register_read(h, 0x03, &xl);
    pmw3901_register_read(h, 0x04, &xh);
    pmw3901_register_read(h, 0x05, &yl);
    pmw3901_register_read(h, 0x06, &yh);

    int16_t sx = ((int16_t)xh << 8) | xl;
    int16_t sy = ((int16_t)yh << 8) | yl;

    *deltaX = sx;
    *deltaY = sy;
    return ESP_OK;
}

esp_err_t pmw3901_enable_frame_buffer(pmw3901_handle_t *h) {
    if (!h) return ESP_ERR_INVALID_ARG;
    // replicate the Arduino sequence
    pmw3901_register_write(h, 0x7F, 0x07);
    pmw3901_register_write(h, 0x41, 0x1D);
    pmw3901_register_write(h, 0x4C, 0x00);
    pmw3901_register_write(h, 0x7F, 0x08);
    pmw3901_register_write(h, 0x6A, 0x38);
    pmw3901_register_write(h, 0x7F, 0x00);
    pmw3901_register_write(h, 0x55, 0x04);
    pmw3901_register_write(h, 0x40, 0x80);
    pmw3901_register_write(h, 0x4D, 0x11);
    pmw3901_register_write(h, 0x70, 0x00);
    pmw3901_register_write(h, 0x58, 0xFF);

    // wait until status bits indicate ready
    uint8_t temp;
    do {
        pmw3901_register_read(h, 0x58, &temp);
        uint8_t check = temp >> 6;
        if (check != 0x03) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (1);

    delay_us(50);
    return ESP_OK;
}

esp_err_t pmw3901_read_frame_buffer(pmw3901_handle_t *h, uint8_t *frame_buffer, size_t len) {
    if (!h || !frame_buffer) return ESP_ERR_INVALID_ARG;
    // original driver reads 1225 bytes (35x35). Caller must provide correct len.
    size_t expected = len;
    uint8_t a, b;
    uint8_t mask = 0x0C;
    size_t count = 0;

    for (size_t ii = 0; ii < expected; ++ii) {
        pmw3901_register_read(h, 0x58, &a);
        uint8_t hold = a >> 6;
        // wait for valid hold
        int guard = 0;
        while ((hold == 0x03) || (hold == 0x00)) {
            pmw3901_register_read(h, 0x58, &a);
            hold = a >> 6;
            if (++guard > 1000) return ESP_ERR_TIMEOUT;
        }
        if (hold == 0x01) {
            pmw3901_register_read(h, 0x58, &b);
            uint8_t pixel = (a << 2) + (b & mask);
            frame_buffer[count++] = pixel;
        } else {
            // in original code 'else' does nothing (skip)
            ii--; // retry this iteration
        }
    }

    // finish sequence
    pmw3901_register_write(h, 0x70, 0x00);
    pmw3901_register_write(h, 0x58, 0xFF);

    // wait ready
    uint8_t temp;
    do {
        pmw3901_register_read(h, 0x58, &temp);
        uint8_t check = temp >> 6;
        if (check != 0x03) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (1);

    return ESP_OK;
}

esp_err_t pmw3901_init_registers(pmw3901_handle_t *h) {
    if (!h) return ESP_ERR_INVALID_ARG;
    // Init sequence (abridged: same values as Arduino driver)
    pmw3901_register_write(h,0x7F,0x00);
    pmw3901_register_write(h,0x61,0xAD);
    pmw3901_register_write(h,0x7F,0x03);
    pmw3901_register_write(h,0x40,0x00);
    pmw3901_register_write(h,0x7F,0x05);
    pmw3901_register_write(h,0x41,0xB3);
    pmw3901_register_write(h,0x43,0xF1);
    pmw3901_register_write(h,0x45,0x14);
    pmw3901_register_write(h,0x5B,0x32);
    pmw3901_register_write(h,0x5F,0x34);
    pmw3901_register_write(h,0x7B,0x08);
    pmw3901_register_write(h,0x7F,0x06);
    pmw3901_register_write(h,0x44,0x1B);
    pmw3901_register_write(h,0x40,0xBF);
    pmw3901_register_write(h,0x4E,0x3F);
    pmw3901_register_write(h,0x7F,0x08);
    pmw3901_register_write(h,0x65,0x20);
    pmw3901_register_write(h,0x6A,0x18);
    pmw3901_register_write(h,0x7F,0x09);
    pmw3901_register_write(h,0x4F,0xAF);
    pmw3901_register_write(h,0x5F,0x40);
    pmw3901_register_write(h,0x48,0x80);
    pmw3901_register_write(h,0x49,0x80);
    pmw3901_register_write(h,0x57,0x77);
    pmw3901_register_write(h,0x60,0x78);
    pmw3901_register_write(h,0x61,0x78);
    pmw3901_register_write(h,0x62,0x08);
    pmw3901_register_write(h,0x63,0x50);
    pmw3901_register_write(h,0x7F,0x0A);
    pmw3901_register_write(h,0x45,0x60);
    pmw3901_register_write(h,0x7F,0x00);
    pmw3901_register_write(h,0x4D,0x11);
    pmw3901_register_write(h,0x55,0x80);
    pmw3901_register_write(h,0x74,0x1F);
    pmw3901_register_write(h,0x75,0x1F);
    pmw3901_register_write(h,0x4A,0x78);
    pmw3901_register_write(h,0x4B,0x78);
    pmw3901_register_write(h,0x44,0x08);
    pmw3901_register_write(h,0x45,0x50);
    pmw3901_register_write(h,0x64,0xFF);
    pmw3901_register_write(h,0x65,0x1F);
    pmw3901_register_write(h,0x7F,0x14);
    pmw3901_register_write(h,0x65,0x60);
    pmw3901_register_write(h,0x66,0x08);
    pmw3901_register_write(h,0x63,0x78);
    pmw3901_register_write(h,0x7F,0x15);
    pmw3901_register_write(h,0x48,0x58);
    pmw3901_register_write(h,0x7F,0x07);
    pmw3901_register_write(h,0x41,0x0D);
    pmw3901_register_write(h,0x43,0x14);
    pmw3901_register_write(h,0x4B,0x0E);
    pmw3901_register_write(h,0x45,0x0F);
    pmw3901_register_write(h,0x44,0x42);
    pmw3901_register_write(h,0x4C,0x80);
    pmw3901_register_write(h,0x7F,0x10);
    pmw3901_register_write(h,0x5B,0x02);
    pmw3901_register_write(h,0x7F,0x07);
    pmw3901_register_write(h,0x40,0x41);
    pmw3901_register_write(h,0x70,0x00);

    vTaskDelay(pdMS_TO_TICKS(100));
    pmw3901_register_write(h,0x32,0x44);
    pmw3901_register_write(h,0x7F,0x07);
    pmw3901_register_write(h,0x40,0x40);
    pmw3901_register_write(h,0x7F,0x06);
    pmw3901_register_write(h,0x62,0xF0);
    pmw3901_register_write(h,0x63,0x00);
    pmw3901_register_write(h,0x7F,0x0D);
    pmw3901_register_write(h,0x48,0xC0);
    pmw3901_register_write(h,0x6F,0xD5);
    pmw3901_register_write(h,0x7F,0x00);
    pmw3901_register_write(h,0x5B,0xA0);
    pmw3901_register_write(h,0x4E,0xA8);
    pmw3901_register_write(h,0x5A,0x50);
    pmw3901_register_write(h,0x40,0x80);

    return ESP_OK;
}
