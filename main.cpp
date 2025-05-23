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

#include <semphr.h>

#include "include/comms.h"
#include "include/election.h"
#include "include/sensing.h"
#include "include/defs.h"

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
    int data = DEVICE_ID;
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
            send_payload.data = DEVICE_ID;
            send_to_device(i, &send_payload, sizeof(payload_t));

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
    radio.powerUp();
    radio.startListening();

    while (1) {
        uint8_t pipe;
        if (radio.available()) {
            //receive_from_device(pipe, &receive_payload, sizeof(payload_t));

            radio.startListening();
            radio.read(&receive_payload, sizeof(payload_t));

            printf(
                "Received from %d: Temperature: %.2f C, Pressure: %.2f Pa\n",
                receive_payload.data,
                receive_payload.bmp280_data.temperature,
                receive_payload.bmp280_data.pressure
            );
        } else {
            printf("No data available\n");
        }
        //vTaskDelay(500 / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void communication_task(void *pvParameters) {
    while (1) {
        // Step 1: Transmit.
        radio.stopListening();
        int success = radio.write(&send_payload, sizeof(payload_t));
        if (success) {
            printf("Data sent successfully\n");
        } else {
            printf("Failed to send data\n");
        }

        // Step 2: Listen for incoming data.
        radio.startListening();
        uint32_t listen_start = xTaskGetTickCount();

        // Listen for a maximum of 1 second.
        while (xTaskGetTickCount() - listen_start < pdMS_TO_TICKS(1000)) {
            if (radio.available()) {
                radio.read(&receive_payload, sizeof(payload_t));
                printf(
                    "Received from %d: Temperature: %.2f C, Pressure: %.2f Pa\n",
                    receive_payload.data,
                    receive_payload.bmp280_data.temperature,
                    receive_payload.bmp280_data.pressure
                );
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Check for incoming data every 100 ms.
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before the next transmission.
    }
}

/**
 * Sensing task that reads temperature and pressure from the BMP280 sensor.
 *
 * @param pvParameters The parameters passed to the task.
 */
void sensing_task(void *pvParameters) {
    while (1) {
        send_payload.bmp280_data.temperature = read_bmp280(BMP280_TEMPERATURE);
        send_payload.bmp280_data.pressure = read_bmp280(BMP280_PRESSURE);

        printf(
            "Temperature: %.2f C, Pressure: %.2f Pa\n",
            send_payload.bmp280_data.temperature,
            send_payload.bmp280_data.pressure
        );
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

    send_payload.bmp280_data.temperature = -1;
    send_payload.bmp280_data.pressure = -1;
}

extern "C" void user_init(void);
void user_init(void){
    uart_set_baud(0, 115200);
    init_values();

    gpio_write(CS_NRF, 1);
    gpio_enable(CS_NRF, GPIO_OUTPUT);
    init_bmp280(BUS_I2C);
    init_nrf24();

    xTaskCreate(communication_task, "communication_task", 1024, NULL, 2, NULL);
    //xTaskCreate(transmit_task, "transmit_task", 1000, NULL, 2, NULL);
    //xTaskCreate(receive_task, "receive_task", 1000, NULL, 2, NULL);
    xTaskCreate(sensing_task, "sensing_task", 256, NULL, 2, NULL);
}