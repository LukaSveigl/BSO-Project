#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"
#include "ota-tftp.h"
#include "RF24/nRF24L01.h"
#include "RF24/RF24.h"

#define CE_NRF		3
#define CS_NRF		0
#define channel		33

#define DEVICE_ID    0x01
#define GROUP_PREFIX 0xDE, 0xAD, 0xBE, 0xEF

/**
 * Definitions of the addresses for the devices in the group.
 */
const uint8_t address_device_1[] = { GROUP_PREFIX, 0x01 };
const uint8_t address_device_2[] = { GROUP_PREFIX, 0x02 };
const uint8_t address_device_3[] = { GROUP_PREFIX, 0x03 };
const uint8_t* addresses[] = { address_device_1, address_device_2, address_device_3 };

/**
 * The RF24 radio object.
 */
static RF24 radio(CE_NRF, CS_NRF);

/**
 * Initializes the RF24 radio module and sets up the communication channels.
 *
 * @return The device IDs of the other devices in the group.
 */
inline uint8_t* init_nrf24() {
    gpio_enable(CS_NRF, GPIO_OUTPUT);

    radio.begin();
    radio.setChannel(channel);

    uint8_t reading_device_ids[2] = { 0x01, 0x02 };

    // Open reading pipes to all other devices.
    uint8_t pipe = 1;
    for (int i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
        if (addresses[i][4] != DEVICE_ID) {
            radio.openReadingPipe(pipe, addresses[i]);
            reading_device_ids[pipe - 1] = addresses[i][4];
            pipe++;
        }
    }

    // Open the writing pipe to one peer. Default is the next device - round-robin.
    const uint8_t target_index = (DEVICE_ID + 1) % (sizeof(addresses) / sizeof(addresses[0]));
    radio.openWritingPipe(addresses[target_index]);

    radio.startListening();

    return reading_device_ids;
}

/**
 * Sends data to a specific device.
 *
 * @param device_id The ID of the device to send data to.
 * @param data      The data to send.
 * @param length    The length of the data.
 */
inline void send_to_device(const uint8_t device_id, const void* data, uint8_t length) {
    radio.stopListening();
    radio.openWritingPipe(addresses[device_id]);
    radio.write(data, length);
    radio.startListening();
}

/**
 * Receives data from a device.
 *
 * @param pipe   The pipe number from which to receive data.
 * @param data   Pointer to the buffer where the received data will be stored.
 * @param length The length of the data to receive.
 */
inline void receive_from_device(uint8_t pipe, void* data, uint8_t length) {
    if (radio.available(&pipe)) {
        radio.read(data, length);
    }
}

#endif