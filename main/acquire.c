#include "acquire.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define BMP280_I2C_ADDRESS (0x76)

// TAGS
static const char *TAG_GPS = "GPS";
static const char *TAG_ACQ = "Acquire";
static const char *TAG_BMP = "BMP280";
static const char *TAG_MPU = "MPU9250";

void task_nmea(void *pvParameters)
{
    data_t *acquire_data = (data_t *)pvParameters;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
                                 UART_PIN_NO_CHANGE, GPS_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0));

    // Configure a temporary buffer for the incoming data
    char *buffer = (char *)malloc(1024 + 1);
    size_t total_bytes = 0;

    while (true)
    {
        // Read data from the UART
        int read_bytes = uart_read_bytes(UART_NUM_1,
                                         (uint8_t *)buffer + total_bytes,
                                         1024 - total_bytes, pdMS_TO_TICKS(20));
        if (read_bytes <= 0)
            continue;

        nmea_s *data;
        total_bytes += read_bytes;

        /* find start (a dollar sign) */
        char *start = memchr(buffer, '$', total_bytes);
        if (start == NULL)
        {
            total_bytes = 0;
            continue;
        }

        // REVER ESSA PALHAÇADA

        /* find end of line */
        char *end = memchr(start, '\r', total_bytes - (start - buffer));
        if (end == NULL || *(++end) != '\n')
            continue;
        end[-1] = NMEA_END_CHAR_1;
        end[0] = NMEA_END_CHAR_2;

        // if (end == NULL ||
        //    (end + 1 >= buffer + total_bytes) ||
        //    end[1] != '\n')
        //      continue;
        //     end[0] = NMEA_END_CHAR_1;
        //     end[1] = NMEA_END_CHAR_2;
        //     ++end;

        /* handle data */
        data = nmea_parse(start, end - start + 1, 0);
        if (data == NULL)
        {
            ESP_LOGI(TAG_GPS, "Failed to parse the sentence!\n");
            ESP_LOGI(TAG_GPS, "  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        }
        else
        {
            if (data->errors)
                ESP_LOGI(TAG_GPS, "WARN: The sentence struct contains parse errors!");

            if (data->type == NMEA_GPGGA)
            {
                nmea_gpgga_s *gpgga = (nmea_gpgga_s *)data;
                acquire_data->latitude = gpgga->latitude.degrees + gpgga->latitude.minutes / 60;
                if (gpgga->latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH)
                    acquire_data->latitude = -acquire_data->latitude;

                acquire_data->longitude = gpgga->longitude.degrees + gpgga->longitude.minutes / 60;

                if (gpgga->longitude.cardinal == NMEA_CARDINAL_DIR_WEST)
                    acquire_data->longitude = -acquire_data->longitude;

                acquire_data->gps_altitude = gpgga->altitude;
            }
            nmea_free(data);
        }

        // buffer empty?
        if (end == buffer + total_bytes)
        {
            total_bytes = 0;
            continue;
        }

        // copy rest of buffer to beginning
        if (buffer != memmove(buffer, end, total_bytes - (end - buffer)))
        {
            total_bytes = 0;
            continue;
        }

        total_bytes -= end - buffer;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(buffer);
    vTaskDelete(NULL);
}

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

    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, ADC_CHANNEL_4, &adc_channel));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, cali_handle));
}

void acquire_voltage(data_t *data, adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle)
{
    int raw_voltage = 0;
    int voltage = 0;

    // Read voltage
    ESP_ERROR_CHECK(adc_oneshot_read(*adc_handle, ADC_CHANNEL_4, &raw_voltage));

    // Convert raw voltage to voltage
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(*cali_handle, raw_voltage, &voltage));

    // Update voltage
    data->voltage = (float)voltage * 2 / 1000;
}

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

void mpu9250_init(calibration_t *cal)
{
    /*calibration_t cal = {
        .mag_offset = {.x = 89.923828, .y = 25.347656, .z = 371.917969},
        .mag_scale = {.x = 0.980390, .y = 0.972483, .z = 1.050749},
        .accel_offset = {.x = 0.079194, .y = 0.030222, .z = -0.191995},
        //.accel_offset = {.x = 0, .y = 0.015, .z = 0},
        .accel_scale_lo = {.x = 1.028991, .y = 1.017323, .z = 0.925066},
        .accel_scale_hi = {.x = -0.969137, .y = -0.981898, .z = -1.107134},
        .gyro_bias_offset = {.x = -3.395939, .y = -0.302809, .z = -0.823139}};
    */
    calibration_t *cal;
    *cal = (calibration_t){
        .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
        .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
        .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
        .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
        .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
        .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}};
    
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    i2c_mpu9250_init(cal, I2C_SDA, I2C_SCL);
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);
    set_full_scale_accel_range(MPU9250_ACCEL_FS_16);
    xSemaphoreGive(xI2CMutex);
}

void acquire_mpu9250(data_t *data, vector_t *va, vector_t *vg, vector_t *vm)
{
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(va, vg, vm));
    xSemaphoreGive(xI2CMutex);
    // Transform these values to the orientation of our device.
    transform_accel_gyro(va);
    transform_accel_gyro(vg);
    transform_mag(vm);
    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg->x), DEG2RAD(vg->y), DEG2RAD(vg->z),
                va->x, va->y, va->z,
                vm->x, vm->y, vm->z);
    data->accel_x = va->x * G;
    data->accel_y = va->y * G;
    data->accel_z = va->z * G;
    ahrs_get_euler_in_degrees(&data->rotation_x, &data->rotation_y, &data->rotation_z);
    vTaskDelay(0);
    pause9250();
}

static void i2c_bus_init(i2c_bus_handle_t *i2c_bus)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {0},
        .clk_flags = 0,
    };
    conf.master.clk_speed = 400000;
    *i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (*i2c_bus == NULL)
        ESP_LOGE(TAG_BMP, "Failed to create I2C bus");
}

static esp_err_t i2c_sensor_bmp280_init(bmp280_handle_t *bmp280, i2c_bus_handle_t *i2c_bus)
{
    *bmp280 = bmp280_create(*i2c_bus, BMP280_I2C_ADDRESS);
    return bmp280_default_init(bmp280);
}

void acquire_bmp280(data_t *data, bmp280_handle_t *bmp280, i2c_bus_handle_t *i2c_bus)
{
    if (ESP_OK == bmp280_read_pressure(*bmp280, &data->pressure))
        ESP_LOGI(TAG_BMP, "pressure:%f ", data->pressure);
    else
        ESP_LOGE(TAG_BMP, "Failed to read pressure from BMP280");
    if (ESP_OK == bmp280_read_temperature(*bmp280, &data->temperature))
        ESP_LOGI(TAG_BMP, "temperature:%f ", data->temperature);
    else
        ESP_LOGE(TAG_BMP, "Failed to read temperature from BMP280");
    vTaskDelay(0);
}

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

    static int n = 0;
    if (n++ % 10 == 0) // This affects the frequency of the LoRa messages
    {
        xQueueSend(xLoraQueue, data, 0); // Send to LoRa queue
    }

    ESP_LOGI(TAG_ACQ, "Data sent to queues");
}

void task_acquire(void *pvParameters)
{
    xI2CMutex = xSemaphoreCreateMutex();

    data_t data = {0};

    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(task_nmea, "NMEA", 4096 * 2, &data, 4, NULL);

    // ADC Initialization
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    init_adc(&adc_handle, &cali_handle);

    // I2C Bus Initialization for BMP280
    i2c_bus_handle_t i2c_bus = NULL;
    static bmp280_handle_t bmp280 = NULL;

    i2c_bus_init(&i2c_bus);
    if(i2c_sensor_bmp280_init(&bmp280, &i2c_bus) != ESP_OK)
    {
        ESP_LOGE(TAG_ACQ, "Failed to initialize BMP280 sensor");
        vTaskDelete(NULL);
        return;
    }

    calibration_t cal;
    // MPU9250 Initialization
    mpu9250_init(&cal);
    vector_t va, vg, vm;

    vTaskDelay(pdMS_TO_TICKS(1000));
    while (true)
    {
        // Time and status update
        data.time = (int32_t)(esp_timer_get_time() / 1000);
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        data.status = STATUS;
        xSemaphoreGive(xStatusMutex);
#ifdef ENABLE_GPS2
        acquire_gps(&data);
#endif
        acquire_bmp280(&data, &bmp280, &i2c_bus);
        acquire_mpu9250(&data, &va, &vg, &vm);

        acquire_voltage(&data, &adc_handle, &cali_handle);

        status_checks(&data);

        // Print data
        ESP_LOGI(TAG_ACQ, "\tTime: %ld, \tCount: %d, Status: %d V: %.2f\r\n"
                          "\tBMP\t\tP: %.2f, T: %.2f, A: %.2f\r\n"
                          "\tAccel\t\tX: %.2f, Y: %.2f, Z: %.2f\r\n"
                          "\tGyro\t\tH: %.2f, P: %.2f, Y: %.2f\r\n"
                          "\tGPS\t\tLat: %.5f, Lon: %.5f, A-GPS: %.2f\n"
                          "----------------------------------------",
                 data.time, data.count, data.status, data.voltage,
                 data.pressure, data.temperature, data.bmp_altitude,
                 data.accel_x, data.accel_y, data.accel_z,
                 data.rotation_x, data.rotation_y, data.rotation_z,
                 data.latitude, data.longitude, data.gps_altitude);
        data.count++;
        send_queues(&data);

        // REDUCE AFTER OPTIMIZING CODE
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(&cal);
    vTaskDelete(NULL);
}