#include "FreeRTOS.h"
#include "semphr.h"

// Leach Config
#define DEVICE_ID 1
#define LEADER_ELEC_DELAY 30 // Delay in seconds between each new cluster cycle
#define BROADCAST_RESENDS 3  // How many times to repeat a message when broadcasting (to mitigate packet loss and such..)
#define TIME_BETWEEN_RESENDS 200 // Time between broadacst resend in ms

// MQTT Config
#define MQTT_HOST	"192.168.1.223"
#define MQTT_PORT	1883
#define MQTT_TOPIC	"LEACH_Temps"
#define MQTT_USER	NULL
#define MQTT_PASS	NULL
#define PUB_MSG_LEN 64

// NRF24 Config
#define RX_SIZE 32

#define CE_NRF		3
#define CS_NRF		0
#define radioChannel	108

// Max nodes, this is used only for cluster head to track temps of other members of its cluster
#define MAX_MEMBERS 10

// Pins and such..
#define PCF_ADDRESS	0x38
#define BUS_I2C		0
#define SCL 		14
#define SDA 		12

#define button1		0x20
#define button2		0x10
#define button3		0x80
#define button4		0x40
#define clr_btn		0xf0

#define blueLed		2
#define redLed1  	0x01
#define redLed2  	0x02
#define redLed3  	0x04
#define redLed4  	0x08
#define redLeds_off	0xff
#define redLeds_on	0x00

