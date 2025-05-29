#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

// ALTIMETER BOARD
// CONFIG_ALTIMETER_VERSION_ASR3000_V3
#define BUZZER_GPIO 38
#define LED_GPIO 15
#define ALED_GPIO 6
#define BUTTON_GPIO 0
#define DROGUE_GPIO 48
#define MAIN_GPIO 47
#define RBF_GPIO 4
#define GPS_RX 14
#define I2C_SCL 9
#define I2C_SDA 8
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12
#define SD_CS 10
#define E220_RX 36
#define E220_TX 35
#define E220_AUX 37

// Status flags
#define ARMED (1 << 0)
#define SAFE_MODE (1 << 1)
#define FLYING (1 << 2)
#define CUTOFF (1 << 3)

#define DROGUE_DEPLOYED (1 << 4)
#define MAIN_DEPLOYED (1 << 5)
#define LANDED (1 << 6)
#define LFS_FULL (1 << 7)

// Data structure to store sensor data
typedef struct // size = 64 bytes
{
    int32_t time;
    uint16_t count;
    uint16_t status;

    float pressure;
    float temperature;
    float bmp_altitude;
    float max_altitude;

    float accel_x;
    float accel_y;
    float accel_z;

    float rotation_x; // Heading
    float rotation_y; // Pitch
    float rotation_z; // Roll

    float latitude;
    float longitude;
    float gps_altitude;
    float voltage;
} data_t;

// Data structure to store file counter
typedef struct
{
    uint32_t file_num;
    uint32_t format;
} file_counter_t;

// Global vars
//Tasks
extern TaskHandle_t xTaskLora;

// Queues
extern QueueHandle_t xAltQueue;
extern QueueHandle_t xLittleFSQueue;
extern QueueHandle_t xSDQueue;
extern QueueHandle_t xLoraQueue;

// Mutexes
extern SemaphoreHandle_t xGPSMutex;
extern SemaphoreHandle_t xStatusMutex;
extern SemaphoreHandle_t xI2CMutex;

// Status
extern uint16_t STATUS;

#endif