#include "global.h"

static const char *TAG_ACQ = "ACQUIRE";

#define sACC_THRESHOLD 0.5f // Threshold for GPS sAcc above which xR is scaled up in eskf_update_gps

static void pack_save_data(const data_t *data, save_t *save_data) {
    save_data->time             = data->time;
    save_data->pressure         = data->bmp.pressure;
    save_data->latitude         = data->gps.latitude;
    save_data->longitude        = data->gps.longitude;
    save_data->gps_altitude     = data->gps.altitude;
    save_data->gps_vel_vertical = data->gps.vel_vertical;
    save_data->sAcc             = data->gps.sAcc;
    save_data->fix              = data->gps.fix;
    save_data->accel_x          = data->icm.accel_x;
    save_data->accel_y          = data->icm.accel_y;
    save_data->accel_z          = data->icm.accel_z;
    save_data->gyro_x           = data->icm.gyro_x;
    save_data->gyro_y           = data->icm.gyro_y;
    save_data->gyro_z           = data->icm.gyro_z;
    save_data->mag_x            = data->icm.mag_x;
    save_data->mag_y            = data->icm.mag_y;
    save_data->mag_z            = data->icm.mag_z;
    save_data->flight_state     = data->flight_state;
    save_data->voltage          = data->voltage;
}

static void pack_send_data(const data_t *data, send_t *send_data) {
    send_data->time            = data->time;
    send_data->latitude        = data->gps.latitude;
    send_data->longitude       = data->gps.longitude;
    send_data->fix             = data->gps.fix;
    send_data->q1              = data->icm.q1;
    send_data->q2              = data->icm.q2;
    send_data->q3              = data->icm.q3;
    send_data->q4              = data->icm.q4;
    send_data->kf_altitude     = data->kf.x.h;
    send_data->kf_vel_vertical = data->kf.x.vz;
    send_data->kf_apogee       = data->kf.x.apogee;
    send_data->accel           = data->icm.accel;
    send_data->flight_state    = data->flight_state;
    send_data->voltage         = data->voltage;
}

// Sends data to SD card, LittleFS and LoRa queues
void send_queues(const data_t *data) {
    // Pre-launch idle data
    if (data->flight_state == STATE_IDLE) {
        save_t save_data;
        pack_save_data(data, &save_data);
        xQueueSend(xB4LaunchQueue, &save_data, 0); // Saved to be dumped at landing
    } else if (data->flight_state >= STATE_BOOST && data->flight_state <= STATE_LANDING) {
        save_t save_data;
        pack_save_data(data, &save_data);

        xQueueSend(xSDQueue, &save_data, 0); // Send to SD card queue
        if (!atomic_load_explicit(&lfs_full, memory_order_relaxed))
            xQueueSend(xLittleFSQueue, &save_data, 0);
    }

    send_t send_data;
    pack_send_data(data, &send_data);
    xQueueOverwrite(xLoraQueue, &send_data); // Send to LoRa queue (length 1)
}

void task_acquire(void *pvParameters) {
    uint32_t notifiedValue;
    while (true) {
        // Wait for notification from reading tasks
        xTaskNotifyWait(0,          // don't clear any bits on entry
                        UINT32_MAX, // clear all bits on exit
                        &notifiedValue, portMAX_DELAY);

        data_t data;

        portENTER_CRITICAL(&xDATALock);
        data = data_g;
        portEXIT_CRITICAL(&xDATALock);

        portENTER_CRITICAL(&xBMPMutex);
        data.bmp = bmp_sample_g;
        portEXIT_CRITICAL(&xBMPMutex);

        portENTER_CRITICAL(&xICMMutex);
        data.icm = icm_sample_g;
        portEXIT_CRITICAL(&xICMMutex);

        portENTER_CRITICAL(&xGPSMutex);
        data.gps = gps_sample_g;
        portEXIT_CRITICAL(&xGPSMutex);

        portENTER_CRITICAL(&xADCMutex);
        data.voltage = battery_voltage_g;
        portEXIT_CRITICAL(&xADCMutex);

        // Update time
        data.time = (uint32_t)(esp_timer_get_time() / 1000UL); // ms since boot

        xTaskNotifyGive(xTaskFlightControl);

        switch (notifiedValue) {
        case ICM_BIT:
            // Trust accelerometer less due to vibration if in boost phase
            float xQ = data.flight_state == STATE_BOOST ? 25.0f : 1.0f;
            eskf_predict(&data.kf, data.icm.az, xQ);
            break;
        case BMP_BIT:
            if (!eskf_update_bar(&data.kf, data.bmp.altitude, 1.0f))
                ESP_LOGW(TAG_ACQ, "kf_update_bar skipped");
            break;
        case GPS_BIT:
            float sAcc = (float)data.gps.sAcc * 0.1f; // Convert back to m/s
            // Trust GPS less if speed accuracy is greater than threshold (2+sAcc-sACC_THRESHOLD)²
            float xR = (sAcc > sACC_THRESHOLD) ? (2.0f + sAcc - sACC_THRESHOLD) * (2.0f + sAcc - sACC_THRESHOLD) : 1.0f;
            if (!eskf_update_gps(&data.kf, data.gps.altitude, data.gps.vel_vertical, xR))
                ESP_LOGW(TAG_ACQ, "kf_update_gps skipped");
            break;
        case ADC_BIT:
            break;
        default:
            ESP_LOGW(TAG_ACQ, "Unknown notification received: %s", uint32_to_binary(notifiedValue));
        }

        data.flight_state = atomic_load(&data_g.flight_state);

        send_queues(&data); // SD, littleFS and lora queues

        portENTER_CRITICAL(&xDATALock);
        data_g = data; // Update global data with latest data
        portEXIT_CRITICAL(&xDATALock);
    }
}