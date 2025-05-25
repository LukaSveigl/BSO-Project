void startup_button_task(void *pvParameters) {
    while (1) {
	printf("pcf task..\n");
        if ((read_byte_pcf() & button1) == 0) {
            leachSetting++;
            if (leachSetting == 1) turn_red_led_on(redLed1);
            else if (leachSetting == 2) turn_red_led_on(redLed2);
            else if (leachSetting == 3) turn_red_led_on(redLed3);
            else if (leachSetting == 4) turn_red_led_on(redLed4);
            else if (leachSetting == 5) {
                leachSetting = 0;
                turn_all_red_leds_off();
            }
        }

        if ((read_byte_pcf() & button3) == 0) {
            if (leachSetting < 1) flash_all_red_leds_on();
            else {
		vTaskDelete(xHandleStartupListen);

		printf("START FROM THIS NODE: %d\n", DEVICE_ID);
		turn_all_red_leds_off();

                nextLeaderElecTime = xTaskGetTickCount();

		int resends = 0;
		while (resends < BROADCAST_RESENDS) {
			// Tell all nodes to start the leach task
		        char msg[RX_SIZE];
		        snprintf(msg, RX_SIZE, "LS|SET%d", leachSetting);

		        transmit_message(msg);

			resends++;

			if (resends < BROADCAST_RESENDS)
				vTaskDelay(pdMS_TO_TICKS(TIME_BETWEEN_RESENDS));
		}

                xTaskCreate(leach_task, "LEACH", 1000, NULL, 2, NULL);
		vTaskDelete(NULL);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void startup_listener_task(void *pvParameters) {
    while (1) {
	printf("listening\n");
        while (radio.available()) {
            memset(rx_data, 0, RX_SIZE);
            radio.read(rx_data, RX_SIZE);
	    // radio.read(rx_data, RX_SIZE) might not null-terminate the buffer, so add it manually
	    rx_data[RX_SIZE - 1] = '\0';
            printf("[LISTEN] Received: %s\n", rx_data);

            if (strstr(rx_data, "LS")) {
		vTaskDelete(xHandleStartupButtons);

                sscanf(rx_data, "LS|SET%d", &leachSetting);
                nextLeaderElecTime = xTaskGetTickCount();

                xTaskCreate(leach_task, "LEACH", 1000, NULL, 2, NULL);
                vTaskDelete(NULL);
            }

	    // In case of powerloss, also listen for head announcments, if one is recieved become its member as normal
            else if (strstr(rx_data, "CH")) {
		vTaskDelete(xHandleStartupButtons);

		int resend = 0;

                sscanf(rx_data, "CH:ID%d|SET%d|RSND%d", &headID, &leachSetting, &resend);
		nextLeaderElecTime = xTaskGetTickCount() + (pdMS_TO_TICKS(LEADER_ELEC_DELAY * 1000) - pdMS_TO_TICKS(resend * TIME_BETWEEN_RESENDS));

                led_indicate_cluster_member();

                generate_address_from_id(headID, clusterHeadAddress);
                radio.openWritingPipe(clusterHeadAddress);

                gottenHead = true;
                printf("[MEMBER] Assigned CH: %d\n", headID);

		// To flush out any potential additonal head announcemnts since we just assigned this node a head
		radio.flush_rx();

                xTaskCreate(leach_task, "LEACH", 1000, NULL, 2, NULL);
                vTaskDelete(NULL);
            }

        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
