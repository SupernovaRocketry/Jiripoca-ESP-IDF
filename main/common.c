#include "common.h"

// Global vars
TaskHandle_t xTaskLora = NULL;

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
uint16_t STATUS = 0;