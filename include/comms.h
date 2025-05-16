#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
#include <stdio.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"
#include "ota-tftp.h"
#include "RF24/nRF24L01.h"
#include "RF24/RF24.h"

#include "defs.h"

/**
 * Definitions of the addresses for the devices in the group.
 */
const uint8_t address_device_1[] = { GROUP_PREFIX, 0x01 };
const uint8_t address_device_2[] = { GROUP_PREFIX, 0x02 };
const uint8_t address_device_3[] = { GROUP_PREFIX, 0x03 };
const uint8_t* addresses[] = { address_device_1, address_device_2, address_device_3 };
uint8_t reading_device_ids[2] = { 0x01, 0x02 };

/**
 * The RF24 radio object.
 */
static RF24 radio(CE_NRF, CS_NRF);

/**
 * Initializes the RF24 radio module and sets up the communication channels.
 *
 * @return The device IDs of the other devices in the group.
 */
inline void init_nrf24() {
    gpio_write(CS_NRF, 1);
    gpio_enable(CS_NRF, GPIO_OUTPUT);

    radio.begin();
    radio.setChannel(channel);

    radio.setPALevel(RF24_PA_MAX);

    //radio.openReadingPipe(1, addresses[0]); // Device ID: 0x01
    radio.openReadingPipe(2, addresses[1]);   // Device ID: 0x02
    radio.openReadingPipe(3, addresses[2]);   // Device ID: 0x03


    // Open reading pipes to all other devices.
    /*uint8_t pipe = 1;
    for (unsigned int i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
        if (addresses[i][4] != DEVICE_ID) {
            radio.openReadingPipe(pipe, addresses[i]);
            reading_device_ids[pipe - 1] = addresses[i][4];
            pipe++;
        }
    }

    // Open the writing pipe to one peer. Default is the next device - round-robin.
    const uint8_t target_index = (DEVICE_ID + 1) % (sizeof(addresses) / sizeof(addresses[0]));
    radio.openWritingPipe(addresses[target_index]);*/

    //radio.startListening();
}

/**
 * Sends data to a specific device.
 *
 * @param device_index The index of the device address to send data to.
 * @param data         The data to send.
 * @param length       The length of the data.
 */
inline int send_to_device(const uint8_t device_index, const void* data, uint8_t length) {
    //radio.powerUp();
    radio.stopListening();
    //radio.openWritingPipe(addresses[device_index]);
    radio.openWritingPipe(addresses[0]); // Device ID: 0x01
    //radio.openWritingPipe(addresses[1]); // Device ID: 0x02
    radio.powerUp();
    int success = radio.write(data, length);
    radio.powerDown();
    if (!success) {
        printf("Failed to write to device\n");
    }
    radio.startListening();
    //radio.powerDown();
    return success;
}

/**
 * Receives data from a device.
 *
 * @param pipe   The pipe number from which to receive data.
 * @param data   Pointer to the buffer where the received data will be stored.
 * @param length The length of the data to receive.
 */
inline void receive_from_device(uint8_t pipe, void* data, uint8_t length) {
    //if (radio.available(&pipe)) {
    radio.powerUp();
    radio.read(data, length);
    radio.powerDown();
    //}
}

#endif