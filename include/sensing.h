#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "bmp280/bmp280.h"
#include "i2c/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"

#include "defs.h"

/**
 * The definitions of the BMP280 sensor quantities.
 */
typedef enum {
    BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

/**
 * The BMP280 sensor data.
 */
typedef struct {
    float temperature;
    float pressure;
} bmp280_data_t;

/**
 * The BMP280 sensor device.
 */
bmp280_t bmp280_device;

/**
 * Initializes the BMP280 sensor with the specified I2C bus and address.
 *
 * @param i2c_bus The I2C bus number.
 */
inline void init_bmp280(int i2c_bus) {
    i2c_init(i2c_bus, SCL, SDA, I2C_FREQ_100K);
    gpio_enable(SCL, GPIO_OUTPUT);

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.mode = BMP280_MODE_FORCED;
    bmp280_device.i2c_dev.bus = i2c_bus;
    bmp280_device.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
    if (!bmp280_init(&bmp280_device, &params)) {
        printf("Failed to initialize BMP280\n");
    }
}

/**
 * Reads the specified quantity from the BMP280 sensor.
 *
 * @param quantity A BMP280 sensor quantity.
 * @return The value of the requested quantity.
 */
inline float read_bmp280(const bmp280_quantity quantity) {
    gpio_write(CS_NRF, 1);
    i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);

    float temperature;
    float pressure;

    if (!bmp280_force_measurement(&bmp280_device)) {
        printf("Failed to start forced measurement in BMP280\n");
    }

    // Wait for the measurement to complete.
    while (bmp280_is_measuring(&bmp280_device)) {
    }

    if (!bmp280_read_float(&bmp280_device, &temperature, &pressure, NULL)) {
        printf("Failed to read data from BMP280\n");
    }

    if (quantity == BMP280_TEMPERATURE) {
        return temperature;
    }
    if (quantity == BMP280_PRESSURE) {
        return pressure;
    }

    return 0.0f; // Invalid quantity
}

#endif
