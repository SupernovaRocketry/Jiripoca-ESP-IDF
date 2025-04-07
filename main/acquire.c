#include "acquire.h"

static const char *TAG_ACQ = "Acquire";

// status_checks checks if the rocket is flying, motor is cutoff, or landed
void status_checks(data_t *data)
{
    // Check if accel is higher than FLYING_THRESHOLD
    if (!(data->status & FLYING))
    {
        if (fabs(data->accel_x) > FLYING_THRESHOLD || fabs(data->accel_y) > FLYING_THRESHOLD || fabs(data->accel_z) > FLYING_THRESHOLD)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= FLYING;
            xSemaphoreGive(xStatusMutex);
        }
    }

    // Check if accel is lower than CUTOFF_THRESHOLD
    if ((data->status & FLYING) && !(data->status & CUTOFF))
    {
        if (fabs(data->accel_x) < CUTOFF_THRESHOLD && fabs(data->accel_y) < CUTOFF_THRESHOLD && fabs(data->accel_z) < CUTOFF_THRESHOLD)
        {
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            STATUS |= CUTOFF;
            xSemaphoreGive(xStatusMutex);
        }
    }

    // Check if landed by comparing altitude to altitude 5 seconds ago
    if ((data->status & FLYING) && !(data->status & LANDED))
    {
        static float aux_altitude = 0;
        static int64_t aux_time = 0;
        if (esp_timer_get_time() - aux_time > 5000000) // If 5 seconds have passed since last check
        {
            if (aux_time == 0) // If first time checking, set aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->bmp_altitude;
            }
            else if (fabs(data->bmp_altitude - aux_altitude) < LANDED_THRESHOLD) // If altitude has not changed more than LANDED_THRESHOLD, consider landed
            {
                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LANDED;
                xSemaphoreGive(xStatusMutex);
            }
            else // If altitude has changed more than LANDED_THRESHOLD, update aux_time and aux_altitude
            {
                aux_time = esp_timer_get_time();
                aux_altitude = data->bmp_altitude;
            }
        }
    }
}
// send_queues sends data to queues
void send_queues(data_t *data)
{
    if (!(data->status & LANDED)) // If not landed, send to queues
    {
        if ((data->status & ARMED)) // If armed, send to task_deploy
            xQueueSend(xAltQueue, &data->bmp_altitude, 0);
        xQueueSend(xSDQueue, data, 0);  // Send to SD card queue
        if (!(data->status & LFS_FULL)) // If LittleFS is not full, send to LittleFS queue
            xQueueSend(xLittleFSQueue, data, 0);
    }

    // ESP_LOGI("Acquire", "Data sent to queues");
}

void task_acquire(void *pvParameters)
{
    xI2CMutex = xSemaphoreCreateMutex();

    data_t data = {0};


    vTaskDelay(pdMS_TO_TICKS(1000));
    while (true)
    {
        // Time and status update
        data.time = (int32_t)(esp_timer_get_time() / 1000);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

        status_checks(&data);

        // Print data
        ESP_LOGI(TAG_ACQ, "\tTime: %ld, Status: %ld V: %.2f\r\n"
                          "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                          "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                          "\tGyro\t\tH: %.2f, P: %.2f, Y: %.2f\r\n"
                          "\tGPS\t\tLat: %.5f, Lon: %.5f, A-GPS: %.2f",
                 data.time, data.status, data.voltage,
                 data.pressure, data.temperature, data.bmp_altitude,
                 data.accel_x, data.accel_y, data.accel_z,
                 data.rotation_x, data.rotation_y, data.rotation_z,
                 data.latitude, data.longitude, data.gps_altitude);

        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}