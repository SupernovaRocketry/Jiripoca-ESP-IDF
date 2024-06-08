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

#include "freertos/queue.h"
#include "freertos/semphr.h"

// ALTIMETER BOARD
#ifdef CONFIG_ALTIMETER_VERSION_ASR3000_V3
//#define ENABLE_GPS
#define ENABLE_MPU9250
//#define ENABLE_I2CDEV
//#define ENABLE_BMP280
#define ENABLE_BMX280
//#define ENABLE_LORA
//#define ENABLE_E220
#define ENABLE_ALED
#define ENABLE_ADC

#define BUZZER_GPIO 38
#define LED_GPIO 15
#define ALED_GPIO 6
#define BUTTON_GPIO 0
#define DROGUE_GPIO 48
#define MAIN_GPIO 47
#define RBF_GPIO 4
#define GPS_RX 21
#define I2C_SCL 9
#define I2C_SDA 8
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12
#define SD_CS 10
#define E220_RX 35
#define E220_TX 36
#define E220_AUX 37

#elif CONFIG_ALTIMETER_VERSION_ASR3000_V2
#define ENABLE_MPU6050
#define ENABLE_LORA
#define ENABLE_SX1276

#define BUZZER_GPIO 32
#define LED_GPIO 33
#define BUTTON_GPIO 34
#define DROGUE_GPIO 26
#define MAIN_GPIO 25
#define RBF_GPIO 27
#define GPS_RX 16
#define I2C_SCL 22
#define I2C_SDA 21
#define SD_MOSI 13
#define SD_MISO 12
#define SD_SCK 14
#define SD_CS 15
#define SX1276_SCK 18
#define SX1276_MISO 19
#define SX1276_MOSI 23
#define SX1276_CS 5
#define SX1276_RST 2
#define SX1276_DIO0 4
#endif


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
    uint32_t status;

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
extern uint32_t STATUS;

#endif