#include <stdint.h>
#include "mpu6050.h"
#include "nmea_parser.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "common.h"

// Global vars
// Queues
QueueHandle_t xAltQueue = NULL;
QueueHandle_t xLittleFSQueue = NULL;
QueueHandle_t xSDQueue = NULL;
QueueHandle_t xLoraQueue = NULL;

// Mutexes
SemaphoreHandle_t xGPSMutex = NULL;
SemaphoreHandle_t xStatusMutex = NULL;

// Status
int32_t STATUS = 0;

// GPS object
gps_t gps;