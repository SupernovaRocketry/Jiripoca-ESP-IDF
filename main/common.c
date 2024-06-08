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
SemaphoreHandle_t xI2CMutex = NULL;

// Status
uint32_t STATUS = 0;