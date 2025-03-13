#include "acquire.h"

// calibration_t cal = {
//     .mag_offset = {.x = 89.923828, .y = 25.347656, .z = 371.917969},
//     .mag_scale = {.x = 0.980390, .y = 0.972483, .z = 1.050749},
//     .accel_offset = {.x = 0.079194, .y = 0.030222, .z = -0.191995},
//     //.accel_offset = {.x = 0, .y = 0.015, .z = 0},
//     .accel_scale_lo = {.x = 1.028991, .y = 1.017323, .z = 0.925066},
//     .accel_scale_hi = {.x = -0.969137, .y = -0.981898, .z = -1.107134},

//     .gyro_bias_offset = {.x = -3.395939, .y = -0.302809, .z = -0.823139}};
calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}};

vector_t va, vg, vm;

static void transform_accel_gyro(vector_t *v)
{
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -x;
    v->y = -z;
    v->z = -y;
}

static void transform_mag(vector_t *v)
{
    float x = v->x;
    float y = v->y;
    float z = v->z;

    v->x = -y;
    v->y = z;
    v->z = -x;
}

void mpu9250_task(void *pvParameters)
{

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    i2c_mpu9250_init(&cal, I2C_SDA, I2C_SCL);
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);
    set_full_scale_accel_range(MPU9250_ACCEL_FS_16);
    xSemaphoreGive(xI2CMutex);

    while (true)
    {
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        // Get the Accelerometer, Gyroscope and Magnetometer values.
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
        xSemaphoreGive(xI2CMutex);
        // Transform these values to the orientation of our device.
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Apply the AHRS algorithm
        ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                    va.x, va.y, va.z,
                    vm.x, vm.y, vm.z);

        vTaskDelay(0);
        pause9250();
    }
}
void init_mpu9250()
{
    xTaskCreate(mpu9250_task, "impu9250_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
void acquire_mpu9250(data_t *data)
{
    data->accel_x = va.x * G;
    data->accel_y = va.y * G;
    data->accel_z = va.z * G;
    ahrs_get_euler_in_degrees(&data->rotation_x, &data->rotation_y, &data->rotation_z);
}

acquire_mpu9250(&data);

init_mpu9250();