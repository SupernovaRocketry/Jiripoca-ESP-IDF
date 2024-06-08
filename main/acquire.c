#include "acquire.h"

// TAGS
static const char *TAG_GPS = "GPS";
static const char *TAG_ACQ = "Acquire";

#ifdef ENABLE_GPS
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

void init_nmea_parser()
{
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT(); // NMEA parser configuration
    config.uart.baud_rate = CONFIG_NMEA_PARSER_BAUD_RATE;
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);  // init NMEA parser library
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL); // register event handler for NMEA parser library
}

// acquire_data reads data from sensors and GPS
void acquire_gps(data_t *data)
{
    // GPS read
    xSemaphoreTake(xGPSMutex, portMAX_DELAY);
    data->latitude = gps.latitude;
    data->longitude = gps.longitude;
    data->gps_altitude = gps.altitude;
    xSemaphoreGive(xGPSMutex);
}
#endif

#ifdef ENABLE_BMP280
void init_bmp280(bmp280_t *dev_bmp)
{
    // BMP280  initialization parameters
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.standby = BMP280_STANDBY_05; // Standby time 0.5ms

    // BMP280 Initialization
    ESP_ERROR_CHECK(bmp280_init_desc(dev_bmp, BMP280_I2C_ADDRESS_0, 0, I2C_SDA, I2C_SCL));
    ESP_ERROR_CHECK(bmp280_init(dev_bmp, &params));

    // BMP280 Initialization
    ESP_ERROR_CHECK(bmp280_init_desc(dev_bmp, BMP280_I2C_ADDRESS_0, 0, I2C_SDA, I2C_SCL));
    ESP_ERROR_CHECK(bmp280_init(dev_bmp, &params));
}

void acquire_bmp280(data_t *data, bmp280_t *dev_bmp)
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
}
#endif

#ifdef ENABLE_MPU6050
void init_mpu6050(mpu6050_dev_t *dev_mpu)
{
    ESP_ERROR_CHECK(mpu6050_init_desc(dev_mpu, ADDR, 0, I2C_SDA, I2C_SCL));
    ESP_ERROR_CHECK(mpu6050_init(dev_mpu));

    // Set accelerometer parameters
    mpu6050_set_full_scale_accel_range(dev_mpu, MPU6050_ACCEL_RANGE_16); // Accelerometer range: +/- 16g

    // REMOVE AFTER IMPLEMENTING KALMAN FILTER
    mpu6050_set_dlpf_mode(dev_mpu, MPU6050_DLPF_2); // Digital low pass filter: 2 (94Hz)

    // Print accelerometer and gyroscope ranges
    ESP_LOGI(TAG_ACQ, "Accel range: %d", dev_mpu->ranges.accel);
    ESP_LOGI(TAG_ACQ, "Gyro range:  %d", dev_mpu->ranges.gyro);
}

void acquire_mpu6050(data_t *data, mpu6050_dev_t *dev_mpu)
{

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;

    // MPU6050 read
    ESP_ERROR_CHECK(mpu6050_get_motion(dev_mpu, &accel, &rotation));

    // Convert accel to m/s^2
    data->accel_x = G * accel.x;
    data->accel_y = G * accel.y;
    data->accel_z = G * accel.z;
}
#endif

#ifdef ENABLE_MPU9250
calibration_t cal = {
    .mag_offset = {.x = 85.699219, .y = 23.537109, .z = 389.960938},
    .mag_scale = {.x = 1.002491, .y = 0.987962, .z = 1.009796},
    .accel_offset = {.x = 0, .y = 0, .z = 0},
    .accel_scale_lo = {.x = 1, .y = 1, .z = 1},
    .accel_scale_hi = {.x = -1, .y = -1, .z = -1},

    .gyro_bias_offset = {.x = -3.508518, .y = -0.067434, .z = -0.333212}};
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
void mpu9250_task(void)
{

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    i2c_mpu9250_init(&cal, I2C_SDA, I2C_SCL);
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);
    set_full_scale_accel_range(MPU9250_ACCEL_FS_16);
    xSemaphoreGive(xI2CMutex);

    uint64_t i = 0;
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
    xTaskCreate(mpu9250_task, "impu9250_task", 4096, NULL, 5, NULL);
}
void acquire_mpu9250(data_t *data)
{
    data->accel_x = va.x * G;
    data->accel_y = va.y * G;
    data->accel_z = va.z * G;
    ahrs_get_euler_in_degrees(&data->rotation_x, &data->rotation_y, &data->rotation_z);
}
#endif

#ifdef ENABLE_BMX280
void init_bmx280(bmx280_t *bmx280)
{
    ESP_ERROR_CHECK(bmx280_init(bmx280));

    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_CYCLE));
}
void acquire_bmx280(data_t *data, bmx280_t *bmx280)
{
    float temp, pres, hum, current_altitude;
    ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));

    current_altitude = 44330 * (1 - powf(pres / 101325, 1 / 5.255));

    // Update max altitude
    if (current_altitude > data->max_altitude)
    {
        data->max_altitude = current_altitude;
    }
    data->bmp_altitude = current_altitude;
    data->pressure = pres;
    data->temperature = temp;
}
#endif

#ifdef ENABLE_ADC
void init_adc(adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle)
{

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE};
    adc_oneshot_new_unit(&init_config, adc_handle);

    adc_oneshot_chan_cfg_t adc_channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_oneshot_config_channel(*adc_handle, ADC_CHANNEL_4, &adc_channel);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_cali_create_scheme_curve_fitting(&cali_config, cali_handle);
}

void acquire_voltage(data_t *data, adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle)
{
    int raw_voltage = 0;
    int voltage = 0;

    // Read voltage
    adc_oneshot_read(*adc_handle, ADC_CHANNEL_4, &raw_voltage);

    // Convert raw voltage to voltage
    adc_cali_raw_to_voltage(*cali_handle, raw_voltage, &voltage);

    // Update voltage
    data->voltage = (float)voltage * 2 / 1000;
}
#endif

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
#ifdef ENABLE_LORA
    xQueueSend(xLoraQueue, data, 0); // Send to LoRa queue
#endif

    ESP_LOGI("Acquire", "Data sent to queues");
}

void task_acquire(void *pvParameters)
{
    xI2CMutex = xSemaphoreCreateMutex();

#ifdef ENABLE_I2CDEV
    ESP_ERROR_CHECK(i2cdev_init());
#endif

#ifdef ENABLE_BMP280
    // BMP280 Initialization
    bmp280_t dev_bmp;
    memset(&dev_bmp, 0, sizeof(bmp280_t));
    init_bmp280(&dev_bmp);
#endif

#ifdef ENABLE_MPU6050
    // MPU6050 Initialization
    mpu6050_dev_t dev_mpu = {0};
    init_mpu6050(&dev_mpu);
#endif

#ifdef ENABLE_MPU9250
    init_mpu9250();
#endif

#ifdef ENABLE_BMX280
    //vTaskDelay(pdMS_TO_TICKS(2000));
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bmx280_t *bmx280 = bmx280_create(I2C_NUM_0);
    init_bmx280(bmx280);
    xSemaphoreGive(xI2CMutex);
#endif

#ifdef ENABLE_GPS
    // GPS Initialization
    init_nmea_parser();
#endif

#ifdef ENABLE_ADC
    // ADC Initialization
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    init_adc(&adc_handle, &cali_handle);
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1)
    {
        data_t data;

        // Time and status update
        data.time = (int32_t)(esp_timer_get_time() / 1000);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);

#ifdef ENABLE_GPS
        acquire_gps(&data);
#endif

#ifdef ENABLE_BMP280
        acquire_bmp280(&data, &dev_bmp);
#endif

#ifdef ENABLE_BMX280
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        acquire_bmx280(&data, bmx280);
        xSemaphoreGive(xI2CMutex);
#endif

#ifdef ENABLE_MPU6050
        acquire_mpu6050(&data, &dev_mpu);
#endif

#ifdef ENABLE_MPU9250
        acquire_mpu9250(&data);
#endif

#ifdef ENABLE_ADC
        acquire_voltage(&data, &adc_handle, &cali_handle);
#endif

        status_checks(&data);

        // Print data
        /*ESP_LOGI(TAG_ACQ, "\tTime: %ld, Status: %ld V: %.2f\r\n"
                          "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                          "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                          "\tGyro\t\tH: %.2f, P: %.2f, Y: %.2f\r\n"
                          "\tGPS\t\tLat: %.5f, Lon: %.5f, Alt: %.2f",
                 data.time, data.status, data.voltage,
                 data.pressure, data.temperature, data.bmp_altitude,
                 data.accel_x, data.accel_y, data.accel_z,
                 data.rotation_x, data.rotation_y, data.rotation_z,
                 data.latitude, data.longitude, data.gps_altitude);*/

        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}