#include "string.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"


static RF24 radio(CE_NRF, CS_NRF);

void transmit_message(const char *msg) {
	turn_blue_led_on();

	radio.stopListening();
	radio.powerUp();

	radio.startWrite(msg, strlen(msg) + 1, false);

	radio.powerDown();
	turn_blue_led_off();
	radio.startListening();
}

static inline void setup_nrf() {
	uart_set_baud(0, 115200);
	gpio_enable(SCL, GPIO_OUTPUT);
	gpio_enable(CS_NRF, GPIO_OUTPUT);

	// radio configuration
	radio.begin();
	// Channel is 108 so normal wi-fi does not disturb it too much
	radio.setChannel(radioChannel);

	// Better for noisy environments
	radio.setPALevel(RF24_PA_LOW);   
	radio.setDataRate(RF24_250KBPS); 

	radio.setAutoAck(false);
}

