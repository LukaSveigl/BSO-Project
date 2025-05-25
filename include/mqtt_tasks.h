extern "C" {
#include "paho_mqtt_c/MQTTESP8266.h"
#include "paho_mqtt_c/MQTTClient.h"
}

static const char * get_my_id(void) {
	// Use MAC address for Station as unique ID
	static char my_id[13];
	static bool my_id_done = false;
	int8_t i;
	uint8_t x;
	if (my_id_done)
		return my_id;
	if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *) my_id))
		return NULL;
	for (i = 5; i >= 0; --i) {
		x = my_id[i] & 0x0F;
		if (x > 9)
			x += 7;
		my_id[i * 2 + 1] = x + '0';
		x = my_id[i] >> 4;
		if (x > 9)
			x += 7;
		my_id[i * 2] = x + '0';
	}
	my_id[12] = '\0';
	my_id_done = true;
	return my_id;
}

static void topic_received(mqtt_message_data_t *md) {
	int i;
	mqtt_message_t *message = md->message;
	printf("Received: ");
	for (i = 0; i < md->topic->lenstring.len; ++i)
		printf("%c", md->topic->lenstring.data[i]);

	printf(" = ");
	for (i = 0; i < (int) message->payloadlen; ++i)
		printf("%c", ((char *) (message->payload))[i]);

	printf("\r\n");
}

void ch_publish_task(void *pvParameters) {
	while (1) {
		if (amClusterHead) {
			float ownTemp = read_bmp280(BMP280_TEMPERATURE);

			// Sanity check for sensor reading
			if (ownTemp < -40.0f || ownTemp > 85.0f) {
				printf("Invalid temperature read: %.2fC\r\n", ownTemp);
				ownTemp = 0.0f; // Optionally set to 0 or skip publishing
			}

			static char msg[PUB_MSG_LEN];
			memset(msg, 0, PUB_MSG_LEN); // Clear buffer every time

			int len = snprintf(msg, PUB_MSG_LEN, "H:%d:%.2fC", DEVICE_ID, ownTemp);
			if (len < 0 || len >= PUB_MSG_LEN) {
				printf("Error formatting head temp message.\r\n");
				continue;
			}

			for (int i = 0; i < MAX_MEMBERS; ++i) {
				if (memberTemps[i].node_id != 0) {
					int written = snprintf(msg + len, PUB_MSG_LEN - len, ";M:%d:%.2fC", memberTemps[i].node_id, memberTemps[i].temperature);
					if (written < 0 || written >= (PUB_MSG_LEN - len)) {
						printf("Message truncated at member %d.\r\n", i);
						break;
					}
						len += written;
				}
			}

			if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
				printf("Publish queue overflow.\r\n");
			}

			// Clear member temps after publishing
			for (int i = 0; i < MAX_MEMBERS; ++i) {
				memberTemps[i].node_id = 0;
				memberTemps[i].temperature = 0.0f;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

static void mqtt_task(void *pvParameters) {
	int ret = 0;
	struct mqtt_network network;
	mqtt_client_t client = mqtt_client_default;
	char mqtt_client_id[20];
	uint8_t mqtt_buf[100];
	uint8_t mqtt_readbuf[100];
	mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

	mqtt_network_new(&network);
	memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
	strcpy(mqtt_client_id, "ESP-");
	strcat(mqtt_client_id, get_my_id());

	while (1) {
		xSemaphoreTake(wifi_alive, portMAX_DELAY);
		printf("%s: started\n\r", __func__);
		printf("%s: (Re)connecting to MQTT server %s ... ", __func__,
		MQTT_HOST);
		ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
		if (ret) {
			printf("error: %d\n\r", ret);
			taskYIELD();
			continue;
		}
		printf("done\n\r");
		mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);

		data.willFlag = 0;
		data.MQTTVersion = 3;
		data.clientID.cstring = mqtt_client_id;
		data.username.cstring = MQTT_USER;
		data.password.cstring = MQTT_PASS;
		data.keepAliveInterval = 10;
		data.cleansession = 0;
		printf("Send MQTT connect ... ");
		ret = mqtt_connect(&client, &data);
		if (ret) {
			printf("error: %d\n\r", ret);
			mqtt_network_disconnect(&network);
			taskYIELD();
			continue;
		}
		printf("done\r\n");
		mqtt_subscribe(&client, "/esptopic", MQTT_QOS1, topic_received);
		xQueueReset(publish_queue);

		while (1) {

			char msg[PUB_MSG_LEN - 1] = "\0";
			while (xQueueReceive(publish_queue, (void *) msg, 0) ==
			pdTRUE) {
				turn_blue_led_on();
				printf("got message to publish\r\n");
				mqtt_message_t message;
				message.payload = msg;
				message.payloadlen = PUB_MSG_LEN;
				message.dup = 0;
				message.qos = MQTT_QOS1;
				message.retained = 0;
				ret = mqtt_publish(&client, MQTT_TOPIC, &message);
				if (ret != MQTT_SUCCESS) {
					printf("error while publishing message: %d\n", ret);
					break;
				}
				turn_blue_led_off();
			}

			ret = mqtt_yield(&client, 1000);
			if (ret == MQTT_DISCONNECTED)
				break;
		}
		printf("Connection dropped, request restart\n\r");
		mqtt_network_disconnect(&network);
		taskYIELD();
	}
}
