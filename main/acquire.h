#ifndef ACQUIRE_H
#define ACQUIRE_H

#include "common.h"
#ifdef ENABLE_ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#endif
#ifdef ENABLE_BMP280
#include "bmp280.h"
#endif
#ifdef ENABLE_BMX280
#include "bmx280.h"
#endif
#ifdef ENABLE_MPU6050
#include "mpu6050.h"
#endif
#ifdef ENABLE_MPU9250
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common9250.h"
#endif
#ifdef ENABLE_GPS
#include "nmea_parser.h"
gps_t gps;
#endif

#define G 9.80665

#define FLYING_THRESHOLD 2 * G // Acceleration threshold to consider rocket flying
#define CUTOFF_THRESHOLD 2 * G // Acceleration threshold to consider motor cutoff
#define LANDED_THRESHOLD 2     // Altitude threshold to consider rocket landed

// MPU6050
#ifdef CONFIG_MPU6050_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

#endif

void task_acquire(void *pvParameters);