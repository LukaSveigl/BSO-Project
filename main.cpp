#include <stdio.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "i2c/i2c.h"

#include "ota-tftp.h"
#include "rboot-api.h"

#include "RF24/nRF24L01.h"
#include "RF24/RF24.h"

#include "include/comms.h"
#include "include/election.h"
#include "include/sensing.h"

#define BUS_I2C 0

#define NUM_DEVICES 3

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
    uint8_t data[3];
} payload_t;

/**
 * The payload instances used for communication.
 */
payload_t receive_payload;
payload_t send_payload;

/**
 * Transmit task that sends data over the RF24 module. This task is used to communicate with other nodes, transmitting
 * temperature information to the leader and data about leader elections (when necessary).
 *
 * @param pvParameters The parameters passed to the task.
 */
void transmit_task(void *pvParameters) {
    while (1) {
        for (int i = 0; i < NUM_DEVICES - 1; i++) {
            const uint8_t device_id = reading_device_ids[i];
            send_to_device(device_id, &send_payload, sizeof(payload_t));

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

/**
 * Receive task that listens for incoming data from the RF24 module. This task is used to receive data from other nodes,
 * including temperature information (in case it is chosen as the leader) and leader election messages.
 *
 * @param pvParameters The parameters passed to the task.
 */
void receive_task(void *pvParameters) {
    while (1) {
        if (radio.available()) {
            for (int i = 0; i < NUM_DEVICES - 1; i++) {
                receive_from_device(i, &receive_payload, sizeof(payload_t));

                printf(
                    "Temperature: %.2f C, Pressure: %.2f hPa\n",
                    receive_payload.bmp280_data.temperature,
                    receive_payload.bmp280_data.pressure
                );
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

/**
 * Sensing task that reads temperature and pressure from the BMP280 sensor.
 *
 * @param pvParameters The parameters passed to the task.
 */
void sensing_task(void *pvParameters) {
    while (1) {
        // Note: This should be removed, only the payload data should remain.
        bmp280_data.temperature = read_bmp280(BMP280_TEMPERATURE);
        bmp280_data.pressure = read_bmp280(BMP280_PRESSURE);

        send_payload.bmp280_data.temperature = bmp280_data.temperature;
        send_payload.bmp280_data.pressure = bmp280_data.pressure;

        printf("Temperature: %.2f C, Pressure: %.2f hPa\n", bmp280_data.temperature, bmp280_data.pressure);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * Initializes various values used in the program.
 */
void init_values() {
    // Initialize the BMP280 data structure.
    bmp280_data.temperature = -1;
    bmp280_data.pressure = -1;

    // Initialize the payloads used for communication.
    receive_payload.bmp280_data.temperature = -1;
    receive_payload.bmp280_data.pressure = -1;
    receive_payload.data[0] = 0;
    receive_payload.data[1] = 0;
    receive_payload.data[2] = 0;

    send_payload.bmp280_data.temperature = -1;
    send_payload.bmp280_data.pressure = -1;
    send_payload.data[0] = 0;
    send_payload.data[1] = 0;
    send_payload.data[2] = 0;
}

extern "C" void user_init(void);
void user_init(void){
    uart_set_baud(0, 115200);
    init_values();

    init_bmp280(BUS_I2C);
    init_nrf24();

    xTaskCreate(transmit_task, "transmit_task", 256, NULL, 2, NULL);
    xTaskCreate(receive_task, "receive_task", 256, NULL, 2, NULL);
    xTaskCreate(sensing_task, "sensing_task", 256, NULL, 2, NULL);
}