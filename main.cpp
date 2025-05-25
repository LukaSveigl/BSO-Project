#include "include/config.h"
#include "include/globals.h"
#include "include/utils.h"

#include "include/leds.h"
#include "include/sensing.h"
#include "include/comms.h"

#include "include/wifi_task.h"
#include "include/mqtt_tasks.h"
#include "include/leach_task.h"
#include "include/startup_tasks.h"


extern "C" void user_init(void) {
	i2c_mutex = xSemaphoreCreateMutex();

	amClusterHead = false;
	gottenHead = false;
	leachSetting = 0;

	gpio_enable(blueLed, GPIO_OUTPUT);

	// Start with all LEDs turned off
	turn_blue_led_off();
	turn_all_red_leds_off();

	init_bm280_values();
	init_bmp280(BUS_I2C);

	generate_address_from_id(DEVICE_ID, myAddress);

	setup_nrf();

	radio.openWritingPipe(broadcastAddress);
	radio.openReadingPipe(1, broadcastAddress);

	radio.startListening();

	xTaskCreate(startup_listener_task, "Radio listen task", 1000, NULL, 2, &xHandleStartupListen);
	xTaskCreate(startup_button_task, "PCF task", 1000, NULL, 3, &xHandleStartupButtons);

	vSemaphoreCreateBinary(wifi_alive);
	publish_queue = xQueueCreate(3, PUB_MSG_LEN);
	xTaskCreate(&wifi_task, "wifi_task", 256, NULL, 2, NULL);
	xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);

	xTaskCreate(&ch_publish_task, "ch_pub", 512, NULL, 2, NULL);
}
