#ifndef ACQUIRE_H
#define ACQUIRE_H

#include "common.h"

// ENABLE_ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ENABLE_BMP280
#include "bmp280.h"
#include "esp_system.h"

// ENABLE_MPU9250
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common9250.h"
#include "i2cdev.h"

// ENABLE_GPS
#include "driver/uart.h"
#include "nmea.h"
#include "gpgga.h"

#define G 9.80665

#define FLYING_THRESHOLD 10 * G // Acceleration threshold to consider rocket flying
#define CUTOFF_THRESHOLD 3 * G // Acceleration threshold to consider motor cutoff
#define LANDED_THRESHOLD 2     // Altitude threshold to consider rocket landed

void task_acquire(void *pvParameters);

#endif