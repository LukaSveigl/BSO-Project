#include "task.h"

// Addresses
uint8_t myAddress[5];
uint8_t clusterHeadAddress[5];
const uint8_t broadcastAddress[5] = {0x11, 0x11, 0x11, 0x11, 0x11};

// Comms
static char rx_data[RX_SIZE];

// Queues
QueueHandle_t publish_queue;

// Mutexes
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t wifi_alive;

// Task handlers
TaskHandle_t xHandleStartupButtons = NULL;
TaskHandle_t xHandleStartupListen = NULL;

// Leach alg
TickType_t nextLeaderElecTime = 0;

bool amClusterHead;
bool gottenHead;

int headID;
int leachSetting;

// Sensing
typedef struct {
	int node_id;
	float temperature;
	TickType_t last_update;
} member_temp_t;

member_temp_t memberTemps[MAX_MEMBERS];


