bool announcedAmHead = false;

void reset_member_temps() {
	for (int i = 0; i < MAX_MEMBERS; ++i) {
		memberTemps[i].node_id = 0;
		memberTemps[i].temperature = 0.0f;
	}
}

void prepare_for_leader_election() {
	printf("Leader check\n");
	turn_all_red_leds_off();
	reset_member_temps();
	radio.flush_rx();
	radio.openReadingPipe(1, broadcastAddress);
	nextLeaderElecTime = xTaskGetTickCount() + pdMS_TO_TICKS(LEADER_ELEC_DELAY * 1000);
	announcedAmHead = false;
	gottenHead = false;
}

void checkIfHead() {
	int probabilityPercent;

	switch (leachSetting) {
		case 1: probabilityPercent = 30; break;
		case 2: probabilityPercent = 40; break;
		case 3: probabilityPercent = 50; break;
		case 4: probabilityPercent = 60; break;
		default: probabilityPercent = 0; break;
	}

	int randVal = rand() % 100;
	amClusterHead = (randVal < probabilityPercent);

	printf("[ELECTION] Node %d: randVal = %d\n", DEVICE_ID, randVal);

	if (amClusterHead)
		printf("[ELECTION] Node %d became Cluster Head (rand=%d < %d%%)\n", DEVICE_ID, randVal, probabilityPercent);
	else
		printf("[ELECTION] Node %d is not Head (rand=%d >= %d%%)\n", DEVICE_ID, randVal, probabilityPercent);
}

void announce_head() {
	led_indicate_cluster_head();
	radio.openWritingPipe(broadcastAddress);

	printf("Announcing am leader in 2s..\n");
	vTaskDelay(pdMS_TO_TICKS(2000));

	printf("Announce am leader\n");
	nextLeaderElecTime = xTaskGetTickCount() + pdMS_TO_TICKS(LEADER_ELEC_DELAY * 1000);

	for (int resends = 0; resends < BROADCAST_RESENDS; ++resends) {
		printf("Announce am leader, resend:%d\n", resends);
		char msg[RX_SIZE];
		snprintf(msg, RX_SIZE, "CH:ID%d|SET%d|RSND%d", DEVICE_ID, leachSetting, resends);
		transmit_message(msg);

		if (resends < BROADCAST_RESENDS - 1)
			vTaskDelay(pdMS_TO_TICKS(TIME_BETWEEN_RESENDS));
	}

	generate_address_from_id(DEVICE_ID, clusterHeadAddress);
	radio.openReadingPipe(1, myAddress);
	announcedAmHead = true;
}

void listen_to_members() {
	printf("Listening to members..\n");
	while (radio.available()) {
		memset(rx_data, 0, RX_SIZE);
		radio.read(rx_data, RX_SIZE);
		rx_data[RX_SIZE - 1] = '\0';
		printf("[CH] Received: %s\n", rx_data);

		if (strstr(rx_data, "TS")) {
			int sender_id;
			float temp;
			sscanf(rx_data, "TS:ID%d|TMP%fC", &sender_id, &temp);
			update_member_temp(sender_id, temp);
		}
	}
}

void listen_for_head() {
	printf("Listening for head announcements\n");
	while (radio.available()) {
		memset(rx_data, 0, RX_SIZE);
		radio.read(rx_data, RX_SIZE);
		rx_data[RX_SIZE - 1] = '\0';
		printf("[MEMBER] Heard: %s\n", rx_data);

		if (strstr(rx_data, "CH")) {
			int resend = 0;
			sscanf(rx_data, "CH:ID%d|SET%d|RSND%d", &headID, &leachSetting, &resend);
			nextLeaderElecTime = xTaskGetTickCount() + (pdMS_TO_TICKS(LEADER_ELEC_DELAY * 1000) - pdMS_TO_TICKS(resend * TIME_BETWEEN_RESENDS));

			led_indicate_cluster_member();
			generate_address_from_id(headID, clusterHeadAddress);
			radio.openWritingPipe(clusterHeadAddress);
			gottenHead = true;

			printf("[MEMBER] Assigned CH: %d\n", headID);
			radio.flush_rx();
		}
	}
}

void transmit_to_head() {
	printf("Transmiting data to head..\n");

	send_payload.bmp280_data.temperature = read_bmp280(BMP280_TEMPERATURE);

	char msg[RX_SIZE];
	snprintf(msg, RX_SIZE, "TS:ID%d|TMP%.2fC", DEVICE_ID, send_payload.bmp280_data.temperature);
	transmit_message(msg);

	TickType_t toNextLeaderElec = nextLeaderElecTime - xTaskGetTickCount();
	if (toNextLeaderElec <= pdMS_TO_TICKS(1900))
		vTaskDelay(toNextLeaderElec);
	else
		vTaskDelay(pdMS_TO_TICKS(1900));
}

void leach_task(void *pvParameters) {
	srand((unsigned int)(xTaskGetTickCount() + DEVICE_ID * 100));

	while (1) {
		if (xTaskGetTickCount() >= nextLeaderElecTime) {
			prepare_for_leader_election();
			checkIfHead();
		}

		if (amClusterHead) {
			if (!announcedAmHead)
				announce_head();
			else
				listen_to_members();
		} else {
			if (!gottenHead)
				listen_for_head();
			else
				transmit_to_head();
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

