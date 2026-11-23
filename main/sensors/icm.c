#include "global.h"

static const char *TAG_ICM = "ICM20948";

#define ICM_ERROR_CHECK(x)                                                                                                                 \
    do {                                                                                                                                   \
        icm20948_status_e err_rc_ = (x);                                                                                                   \
        if (unlikely(err_rc_ != ICM_20948_STAT_OK)) {                                                                                      \
            _esp_error_check_failed(ESP_FAIL, __FILE__, __LINE__, __ASSERT_FUNC, #x);                                                      \
        }                                                                                                                                  \
    } while (0)

#define GYRO_SCALE_RAD GYRO_SCALE * 3.14159265359f / 180.0f // LSB to rad/s
#define ACC_SCALE_MS2  ACC_SCALE *G                         // LSB to m/s²

// Rotate sensor frame acceleration to world frame with gravity removed (ENU)
static void rotateToWorld(float qw, float qx, float qy, float qz, float a_s[3], float a_w[3]) {
    float R11 = 1 - 2 * qy * qy - 2 * qz * qz;
    float R12 = 2 * qx * qy - 2 * qz * qw;
    float R13 = 2 * qx * qz + 2 * qy * qw;

    float R21 = 2 * qx * qy + 2 * qz * qw;
    float R22 = 1 - 2 * qx * qx - 2 * qz * qz;
    float R23 = 2 * qy * qz - 2 * qx * qw;

    float R31 = 2 * qx * qz - 2 * qy * qw;
    float R32 = 2 * qy * qz + 2 * qx * qw;
    float R33 = 1 - 2 * qx * qx - 2 * qy * qy;

    a_w[0] = R11 * a_s[0] + R12 * a_s[1] + R13 * a_s[2];
    a_w[1] = R21 * a_s[0] + R22 * a_s[1] + R23 * a_s[2];
    a_w[2] = R31 * a_s[0] + R32 * a_s[1] + R33 * a_s[2] - G;
}

static void icm_init(icm20948_device_t *icm_dev) {
    // ICM20948 struct setup
    icm20948_config_i2c_t icm_config = {
        .i2c_addr        = ICM20948_I2C_ADDRESS,
        .i2c_clock_speed = I2C_SPEED,
    };

    // ICM20948 setup device
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_init_i2c(bus_handle, &icm_config, icm_dev));
    xSemaphoreGive(xI2CSem);

    ESP_LOGI(TAG_ICM, "ICM20948 initialized");

    // Check ID
    icm20948_status_e stat;
    for (int i = 0; i < 5; i++) // Retry 5 times
    {
        xSemaphoreTake(xI2CSem, portMAX_DELAY);
        stat = icm20948_check_id(icm_dev);
        xSemaphoreGive(xI2CSem);
        if (stat == ICM_20948_STAT_OK)
            break;
        ESP_LOGD(TAG_ICM, "ICM20948 ID check failed, retry %d/5", i + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ICM_ERROR_CHECK(stat);

    // SW reset to make sure the device starts in a known state
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_sw_reset(icm_dev));
    xSemaphoreGive(xI2CSem);
    vTaskDelay(pdMS_TO_TICKS(250));

    // Now wake the sensor up
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_sleep(icm_dev, false));
    ICM_ERROR_CHECK(icm20948_low_power(icm_dev, false));
    xSemaphoreGive(xI2CSem);

    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

    // Set Gyro and Accelerometer to a particular sample mode
    // options: SAMPLE_MODE_CONTINUOUS; SAMPLE_MODE_CYCLED
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_set_sample_mode(icm_dev, sensors, SAMPLE_MODE_CONTINUOUS));
    xSemaphoreGive(xI2CSem);

    // Set up sensors sample rate
    icm20948_smplrt_t smplrt;
    smplrt.a = 4;
    smplrt.g = 4; // ODR 225 Hz
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_set_sample_rate(icm_dev, sensors, smplrt));
    xSemaphoreGive(xI2CSem);

    // Set full scale ranges for both acc and gyr
    icm20948_fss_t myfss;
    myfss.a = GPM_16;
    myfss.g = DPS_500;
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_set_full_scale(icm_dev, sensors, myfss));
    xSemaphoreGive(xI2CSem);

    // Set up Digital Low Pass Filter configuration
    icm20948_dlpcfg_t myDLPcfg;
    myDLPcfg.a = ACC_D23BW9_N34BW4;
    myDLPcfg.g = GYR_D23BW9_N35BW9;
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_set_dlpf_cfg(icm_dev, sensors, myDLPcfg));
    xSemaphoreGive(xI2CSem);

    // Choose whether or not to use DLPF
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_enable_dlpf(icm_dev, ICM_20948_INTERNAL_ACC, true));
    ICM_ERROR_CHECK(icm20948_enable_dlpf(icm_dev, ICM_20948_INTERNAL_GYR, true));
    xSemaphoreGive(xI2CSem);

    // Initialize magnetometer
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    ICM_ERROR_CHECK(icm20948_init_magnetometer(icm_dev));
    xSemaphoreGive(xI2CSem);
}

void task_fusion(void *pvParameters) {
    icm20948_device_t icm_dev;
    icm_init(&icm_dev);

    TickType_t       xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(ICM_SAMPLE_RATE_S * 1000);

    vqf_params_t params;
    vqf_params_init(&params);
    vqf_handle_t *vqf = vqf_init_custom(&params, ICM_SAMPLE_RATE_S, -1, -1); // gyrTs, accTs, magTs

    icm20948_sample_t icm;
    static float      initial_temp = 0.0f;

    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        icm20948_agmt_t agmt;
        xSemaphoreTake(xI2CSem, portMAX_DELAY);
        icm20948_status_e stat = icm20948_get_agmt(&icm_dev, &agmt);
        xSemaphoreGive(xI2CSem);
        if (stat == ICM_20948_STAT_OK) {
            if (initial_temp == 0)
                initial_temp = (agmt.tmp.val * TEMP_SCALE + TEMP_OFFSET) + 273;

            float gyr[3] = {agmt.gyr.axes.x * GYRO_SCALE_RAD, agmt.gyr.axes.y * GYRO_SCALE_RAD, agmt.gyr.axes.z * GYRO_SCALE_RAD};
            float acc[3] = {agmt.acc.axes.x * ACC_SCALE_MS2, agmt.acc.axes.y * ACC_SCALE_MS2, agmt.acc.axes.z * ACC_SCALE_MS2};
            float mag[3] = {agmt.mag.axes.x * MAG_SCALE, agmt.mag.axes.y * MAG_SCALE, agmt.mag.axes.z * MAG_SCALE};
            vqf_update(vqf, gyr, acc, mag);

            float q[4];
            vqf_get_quat9D(vqf, q);

            icm.q1 = q[0];
            icm.q2 = q[1];
            icm.q3 = q[2];
            icm.q4 = q[3];

            // Update raw sensor data
            icm.accel_x = agmt.acc.axes.x;
            icm.accel_y = agmt.acc.axes.y;
            icm.accel_z = agmt.acc.axes.z;

            icm.gyro_x = agmt.gyr.axes.x;
            icm.gyro_y = agmt.gyr.axes.y;
            icm.gyro_z = agmt.gyr.axes.z;

            icm.mag_x = agmt.mag.axes.x;
            icm.mag_y = agmt.mag.axes.y;
            icm.mag_z = agmt.mag.axes.z;

            float acc_w[3];
            rotateToWorld(q[0], q[1], q[2], q[3], acc, acc_w);
            float accel = sqrtf(acc_w[0] * acc_w[0] + acc_w[1] * acc_w[1] + acc_w[2] * acc_w[2]);
            accel /= G; // Convert to g
            icm.accel = (uint8_t)lroundf(accel * 10.0f);
            icm.az    = acc_w[2];

            // Update global ICM sample
            portENTER_CRITICAL(&xICMMutex);
            icm_sample_g = icm;
            portEXIT_CRITICAL(&xICMMutex);

            xTaskNotify(xTaskAcquire, ICM_BIT, eSetBits); // Notify acquire task that new data is available
        } else
            ESP_LOGE(TAG_ICM, "Get agmt failed");

        bool landed = (atomic_load(&data_g.flight_state) == STATE_LANDED);
        if (landed)
            break;
    }

    vqf_delete(vqf);
    xSemaphoreTake(xI2CSem, portMAX_DELAY);
    icm20948_sleep(&icm_dev, true);
    xSemaphoreGive(xI2CSem);
    vTaskDelete(NULL);
}