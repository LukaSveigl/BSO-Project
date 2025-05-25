#include "i2c/i2c.h"
#include "espressif/esp_common.h"


// read byte from PCF on I2C bus
static inline uint8_t read_byte_pcf() {
	if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
		uint8_t data;

		// disable radio
		gpio_write(CS_NRF, 1);
		// reinitialize i2c
		i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
		// read data byte
		i2c_slave_read(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);

		xSemaphoreGive(i2c_mutex);

		return data;
	}

	return -1;
}

void generate_address_from_id(uint8_t id, uint8_t* address) {
	address[0] = 0xA1;
	address[1] = 0xA2;
	address[2] = 0xA3;
	address[3] = 0xA4;
	address[4] = id;
}

void update_member_temp(int node_id, float temp) {
	for (int i = 0; i < MAX_MEMBERS; ++i) {
		if (memberTemps[i].node_id == node_id || memberTemps[i].node_id == 0) {
			memberTemps[i].node_id = node_id;
			memberTemps[i].temperature = temp;
			memberTemps[i].last_update = xTaskGetTickCount();

			return;
		}
	}
}

