#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include "mpu6050.h"
#include "nmea_parser.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Status flags

#define LFS_FULL (1 << 7)

// Data structure to store sensor data
typedef struct // size = 64 bytes
{
    int32_t time;
    uint32_t status;

    float pressure;
    float temperature;
    float bmp_altitude;
    float max_altitude;

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;

    float latitude;
    float longitude;
    float gps_altitude;
    float hor_speed;
} data_t;

// Data structure to store file counter
typedef struct
{
    uint32_t file_num;
    uint32_t format;
} file_counter_t;

// Global vars
// Queues
extern QueueHandle_t xAltQueue;
extern QueueHandle_t xLittleFSQueue;
extern QueueHandle_t xSDQueue;
extern QueueHandle_t xLoraQueue;

// Mutexes
extern SemaphoreHandle_t xGPSMutex;
extern SemaphoreHandle_t xStatusMutex;

// Status
extern uint32_t STATUS;

// GPS object
extern gps_t gps;

#endif