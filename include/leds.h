#include "task.h"

static uint8_t red_led_state = 0xFF;

static inline void turn_red_led_on(uint8_t led_mask) {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
		red_led_state &= ~led_mask;

		// disable radio
		gpio_write(CS_NRF, 1);
		// reinitialize i2c
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		// write data byte
		i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &red_led_state, 1);

		xSemaphoreGive(i2c_mutex);
	}
}

static inline void turn_red_led_off(uint8_t led_mask) {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
		red_led_state |= led_mask;

		// disable radio
		gpio_write(CS_NRF, 1);
		// reinitialize i2c
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		// write updated state
		i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &red_led_state, 1);

		xSemaphoreGive(i2c_mutex);
	}
}

static inline void turn_all_red_leds_off() {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
		uint8_t data = redLeds_off;

		// disable radio
		gpio_write(CS_NRF, 1);
		// reinitialize i2c
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		// write data byte
		i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);

		red_led_state = redLeds_off;

		xSemaphoreGive(i2c_mutex);
	}
}

static inline void turn_all_red_leds_on() {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
		uint8_t data = redLeds_on;

		// disable radio
		gpio_write(CS_NRF, 1);
		// reinitialize i2c
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		// write data byte
		i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);

		red_led_state = redLeds_on;

		xSemaphoreGive(i2c_mutex);
	}
}

// Turns on all leds and than reverts to previous state
static inline void flash_all_red_leds_on() {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
		uint8_t data = redLeds_on;
		uint8_t previousState = red_led_state;

		// Turn all LEDs ON (active-low: 0)
		gpio_write(CS_NRF, 1);
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);

		vTaskDelay(pdMS_TO_TICKS(200));

		// Restore previous state
		gpio_write(CS_NRF, 1);
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &previousState, 1);

		xSemaphoreGive(i2c_mutex);
	}
}

// write byte to PCF on I2C bus
static inline void turn_blue_led_on() {
	gpio_write(blueLed, 0);
}

// write byte to PCF on I2C bus
static inline void turn_blue_led_off() {
	gpio_write(blueLed, 1);
}

void led_indicate_cluster_head() {
	turn_all_red_leds_off();
	if (DEVICE_ID != 1) turn_red_led_on(redLed1);
	if (DEVICE_ID != 2) turn_red_led_on(redLed2);
	if (DEVICE_ID != 3) turn_red_led_on(redLed3);
	if (DEVICE_ID != 4) turn_red_led_on(redLed4);
}

void led_indicate_cluster_member() {
	turn_all_red_leds_off();
	if (headID == 1) turn_red_led_on(redLed1);
	if (headID == 2) turn_red_led_on(redLed2);
	if (headID == 3) turn_red_led_on(redLed3);
	if (headID == 4) turn_red_led_on(redLed4);
}

