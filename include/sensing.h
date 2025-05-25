#include "bmp280/bmp280.h"
#include "RF24/RF24.h"

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

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
 * The BMP280 sensor data structure.
 */
bmp280_data_t bmp280_data;

/**
 * The payload type received/transmitted over the RF24 module. The payload contains both the BMP280 data and the RF24
 * data, as both are sent in the same communication.
 */
typedef struct {
	bmp280_data_t bmp280_data;
} payload_t;

/**
 * The payload instances used for communication.
 */
payload_t receive_payload;
payload_t send_payload;

/**
 * Initializes various values used in the program.
 */
void init_bm280_values() {
	// Initialize the BMP280 data structure.
	bmp280_data.temperature = -1;
	bmp280_data.pressure = -1;

	// Initialize the payloads used for communication.
	receive_payload.bmp280_data.temperature = -1;
	receive_payload.bmp280_data.pressure = -1;

	send_payload.bmp280_data.temperature = -1;
	send_payload.bmp280_data.pressure = -1;
}

/**
 * Initializes the BMP280 sensor with the specified I2C bus and address.
 *
 * @param i2c_bus The I2C bus number.
 */
inline void init_bmp280(int i2c_bus) {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
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

		xSemaphoreGive(i2c_mutex);
	}
}

/**
 * Reads the specified quantity from the BMP280 sensor.
 *
 * @param quantity A BMP280 sensor quantity.
 * @return The value of the requested quantity.
 */
inline float read_bmp280(const bmp280_quantity quantity) {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
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

		xSemaphoreGive(i2c_mutex);

		if (quantity == BMP280_TEMPERATURE) {
		return temperature;
		}
		if (quantity == BMP280_PRESSURE) {
		return pressure;
		}
	}

	return 0.0f; // Invalid quantity
}

#endif
