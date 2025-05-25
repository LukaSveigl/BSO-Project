# BSO-Project
A repository for the project of our Wireless Sensor Networks course.

We implemented LEACH (Low-Energy Adaptive Clustering Hierarchy) protocol for wireless sensor networks.

In theory this code can be used for any number of devices, but we tested it with 3 devices (4 pending). If there are more than 4 devices, the LEDs won't display cluster heads and members correctly since we have only 4 LEDs, otherwise it should work fine.

## How to run

This is assuming you are running in the VM provided by the course, which has all the necessary tools installed.

### MQTT Broker
1. Since cluster heads use MQTT to communicate with the server, you need to have a MQTT broker running. You can use any MQTT broker, but we recommend using Mosquitto. You can install it from [here](https://mosquitto.org/download/).

2. After install Mosquitto, you should configure it to allow anonymous access. This is an example configuration file, your IP and paths may differ:
    ```conf
    # ========================
    # Basic Mosquitto Config
    # ========================

    # Run on default MQTT port
    listener 1883 192.168.1.223
    listener 1883 localhost

    # Allow anonymous clients to connect (default = true)
    allow_anonymous true

    # Persistence (store retained messages and session info)
    persistence true
    persistence_location \some\path\to\mosquitto\data\

    # Logging
    log_dest file \some\path\to\mosquitto\log\mosquitto.log
    log_type error
    log_type warning
    log_type notice
    log_type information
    connection_messages true
    log_timestamp true
    ```

3. Start the Mosquitto broker with the following command:
    ```bash
    mosquitto -c /path/to/your/mosquitto.conf
    ```

4. You can test the broker by publishing a message to a topic and subscribing to it. Use the following commands in separate terminal windows:
    ```bash
    mosquitto_pub -t "test_topic" -h 192.168.1.223 -m "bob"
    mosquitto_sub -t "test_topic" -h 192.168.1.223
    ```

5. If you see the message "bob" in the subscriber terminal, the broker is working correctly.

6. Depending on your operating system, you may need to allow the Mosquitto broker through your firewall.

7. In the `include/config.h` file,  confgure these settings to match your MQTT broker:
   ```c
   #define MQTT_HOST	"192.168.1.223"
   #define MQTT_PORT	1883
   #define MQTT_TOPIC	"LEACH_Temps"
   #define MQTT_USER	NULL
   #define MQTT_PASS	NULL
   ```
   Do not change the `MQTT_USER` and `MQTT_PASS` if you are allowing anonymous access and don't change `PUB_MSG_LEN`.
   

### How to flash
1. Before each flash, change the `DEVICE_ID` in `include/config.h` to a wanted number (e.g. 1, 2, 3, ...), make sure the number is unique for each device and starts from 1 and goes up to the number of devices you have. For example, if you have 3 devices, you should have:
   ```c
   #define DEVICE_ID 1 // Change this to 2, 3, etc. for other devices
   ```
   This is important because the device ID is used to identify cluster heads and members.

2. If using more than 10 devices, change the `MAX_MEMBERS` in `include/config.h` to the number of devices you have (e.g. 20, 100, etc.):
   ```c
   #define MAX_MEMBERS 100 // Change this to the number of devices you have
   ```
2. Flash to devices using the following command:
   ```bash
   make flash ESPPORT=/dev/ttyUSB0
   ```