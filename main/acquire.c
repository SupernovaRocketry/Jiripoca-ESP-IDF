#include <stdio.h>
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
#include "bmp280.h"
#include "mpu6050.h"
#include "nmea_parser.h"

#include "acquire.h"
#include "common.h"

// TAGS
static const char *TAG_GPS = "GPS";
static const char *TAG_ACQ = "Acquire";

// gps_event_handler runs every time GPS data is received
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case GPS_UPDATE:
        xSemaphoreTake(xGPSMutex, portMAX_DELAY);
        gps = *(gps_t *)event_data;
        /*
        // Print information parsed from GPS statements
        ESP_LOGI(TAG_GPS, "%d/%d/%d %d:%d:%d => \r\n"
                         "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                         "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                         "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                         "\t\t\t\t\t\tspeed      = %fm/s\r\n"
                         "\t\t\t\t\t\tsatellite  = %d",
                 gps.date.year + 2000, gps.date.month, gps.date.day,
                 gps.tim.hour + CONFIG_TIMEZONE, gps.tim.minute, gps.tim.second,
                 gps.latitude, gps.longitude, gps.altitude, gps.speed, gps.sats_in_view);
        */
        xSemaphoreGive(xGPSMutex);
        break;
    case GPS_UNKNOWN:
        // Print unknown statements
        ESP_LOGW(TAG_GPS, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

// acquire_data reads data from sensors and GPS
void acquire_data(data_t *data, bmp280_t *dev_bmp, mpu6050_dev_t *dev_mpu)
{
    float temp_altitude = 0;

    // BMP280 read
    if (bmp280_read_float(dev_bmp, &data->temperature, &data->pressure, NULL) != ESP_OK)
    {
        ESP_LOGE(TAG_ACQ, "Temperature/pressure reading failed");
    }

    // BMP280 altitude calculation (barometric formula)
    temp_altitude = 44330 * (1 - powf(data->pressure / 101325, 1 / 5.255));

    // Update max altitude
    if (temp_altitude > data->max_altitude)
    {
        data->max_altitude = temp_altitude;
    }
    data->bmp_altitude = temp_altitude;

    // MPU6050 read
    ESP_ERROR_CHECK(mpu6050_get_motion(dev_mpu, &data->accel, &data->rotation));

    // Convert accel to m/s^2
    data->accel.x *= G;
    data->accel.y *= G;
    data->accel.z *= G;

    // GPS read
    xSemaphoreTake(xGPSMutex, portMAX_DELAY);
    data->latitude = gps.latitude;
    data->longitude = gps.longitude;
    data->gps_altitude = gps.altitude;
    data->hor_speed = gps.speed;
    xSemaphoreGive(xGPSMutex);

    // Time and status update
    data->time = (int32_t)(esp_timer_get_time() / 1000);
    xSemaphoreTake(xStatusMutex, portMAX_DELAY);
    data->status = STATUS;
    xSemaphoreGive(xStatusMutex);

    // Print data
    ESP_LOGI(TAG_ACQ, "\tTime: %ld, Status: %ld \r\n"
                      "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                      "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                      "\tGyro\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                      "\tGPS\t\tLat: %.5f, Lon: %.5f, Alt: %.2f, Spd: %.2f\r\n",
             data->time, data->status,
             data->pressure, data->temperature, data->bmp_altitude,
             data->accel.x, data->accel.y, data->accel.z,
             data->rotation.x, data->rotation.y, data->rotation.z,
             data->latitude, data->longitude, data->gps_altitude, data->hor_speed);
}

// send_queues sends data to queues
void send_queues(data_t *data)
{
    xQueueSend(xSDQueue, data, 0);  // Send to SD card queue
    if (!(data->status & LFS_FULL)) // If LittleFS is not full, send to LittleFS queue
        xQueueSend(xLittleFSQueue, data, 0);

    xQueueSend(xLoraQueue, data, 0); // Always send to Lora queue

    ESP_LOGI("Acquire", "Data sent to queues");
}

void task_acquire(void *pvParameters)
{
    // I2C Initialization
    ESP_ERROR_CHECK(i2cdev_init());

    // BMP280  initialization parameters
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.standby = BMP280_STANDBY_05; // Standby time 0.5ms

    bmp280_t dev_bmp;
    memset(&dev_bmp, 0, sizeof(bmp280_t));

    // BMP280 Initialization
    ESP_ERROR_CHECK(bmp280_init_desc(&dev_bmp, BMP280_I2C_ADDRESS_0, 0, CONFIG_I2C_SDA_GPIO, CONFIG_I2C_SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&dev_bmp, &params));

    // MPU6050 Initialization
    mpu6050_dev_t dev_mpu = {0};
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev_mpu, ADDR, 0, CONFIG_I2C_SDA_GPIO, CONFIG_I2C_SCL_GPIO));
    ESP_ERROR_CHECK(mpu6050_init(&dev_mpu));

    // Set accelerometer parameters
    mpu6050_set_full_scale_accel_range(&dev_mpu, MPU6050_ACCEL_RANGE_16); // Accelerometer range: +/- 16g

    // REMOVE AFTER IMPLEMENTING KALMAN FILTER
    mpu6050_set_dlpf_mode(&dev_mpu, MPU6050_DLPF_2); // Digital low pass filter: 2 (94Hz)

    // Print accelerometer and gyroscope ranges
    ESP_LOGI(TAG_ACQ, "Accel range: %d", dev_mpu.ranges.accel);
    ESP_LOGI(TAG_ACQ, "Gyro range:  %d", dev_mpu.ranges.gyro);

    // GPS Initialization
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT(); // NMEA parser configuration
    config.uart.baud_rate = CONFIG_NMEA_PARSER_BAUD_RATE;
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);  // init NMEA parser library
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL); // register event handler for NMEA parser library

    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1)
    {
        data_t data;

        acquire_data(&data, &dev_bmp, &dev_mpu);

        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}