#include "global.h"

static const char *TAG_SYS = "System";

#define SETUP_TIMEOUT_MS 20000
#define FORMAT_MODE      false

const char *flight_state_to_string(flight_state_t state) {
    switch (state) {
    case STATE_IDLE:
        return "IDLE";
    case STATE_BOOST:
        return "BOOST";
    case STATE_COAST:
        return "COAST";
    case STATE_DROGUE:
        return "DROGUE";
    case STATE_MAIN:
        return "MAIN";
    case STATE_LANDING:
        return "LANDING";
    case STATE_LANDED:
        return "LANDED";
    case STATE_FATAL:
        return "FATAL";
    default:
        return "UNKNOWN";
    }
}

static esp_err_t setup_peripherals(void) {
    esp_err_t err = ESP_OK;

    /* I2C bus configuration */
    i2c_master_bus_config_t bus_config = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = I2C_SDA,
        .scl_io_num                   = I2C_SCL,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = false,
    };

    /* I2C bus Initialization */
    err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SYS, "I2c initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    /* SPI bus configuration */
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num     = SPI2_MOSI,
        .miso_io_num     = SPI2_MISO,
        .sclk_io_num     = SPI2_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };

    /* SPI Bus Initialization */
    err = spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, DMA_CHAN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SYS, "SPI initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    /* GPIO initialization */
    gpio_set_direction(RBF_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RBF_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_direction(BOOT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOOT_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(DROGUE_GPIO);
    gpio_set_direction(DROGUE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DROGUE_GPIO, LOW); // Ensure drogue is not deployed at startup

    gpio_reset_pin(MAIN_GPIO);
    gpio_set_direction(MAIN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MAIN_GPIO, LOW); // Ensure main is not deployed at startup

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SYS, "ISR install failed: %s", esp_err_to_name(err));
        return err;
    }

    if (xEventQueue == NULL)
        return ESP_ERR_INVALID_ARG;
    if (xInitEventGroup == NULL)
        return ESP_ERR_INVALID_ARG;
    if (xNVSCounterEventGroup == NULL)
        return ESP_ERR_INVALID_ARG;
    if (xFormatEventGroup == NULL)
        return ESP_ERR_INVALID_ARG;

    /* Initialization signal  */
    gpio_set_level(LED_GPIO, HIGH);
    gpio_set_level(BUZZER_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_GPIO, LOW);
    gpio_set_level(BUZZER_GPIO, LOW);

    return err;
}

static esp_err_t setup_nvs(bool format_mode) {
    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK) {
        /* Retry nvs_flash_init */
        ESP_LOGE("NVS", "%s, erasing NVS partition...", esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_SYS, "Setup NVS failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* Open NVS */
    nvs_handle_t nvs_handle;
    ESP_LOGI("NVS", "Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SYS, "Setup NVS failed: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t sd_num  = 0;
    uint32_t lfs_num = 0;

    nvs_get_u32(nvs_handle, "sd_counter", &sd_num);
    nvs_get_u32(nvs_handle, "lfs_counter", &lfs_num);

    if (format_mode) {
        sd_num  = 0;
        lfs_num = 0;
    }

    nvs_close(nvs_handle);

    /* Save global file counters */
    file_counter_g.sd_files  = sd_num;
    file_counter_g.lfs_files = lfs_num;
    file_counter_g.format    = format_mode;

    return err;
}

static esp_err_t setup_eskf(void) {
    esp_err_t err = ESP_OK;

    eskf_var_t var = {
        .acc    = 0.25f,
        .bar    = 1.65f,
        .gps_h  = 625.0f,
        .gps_vz = 0.0025f,
        .ba     = 1e-4f,
        .bb     = 5e-1f,
        .θe     = 1e-6f,
    };

    eskf_config_t cfg = {
        .var          = var,
        .dt           = ICM_SAMPLE_RATE_S,
        .g            = G,
        .igt          = 3.0f,
        .idle_samples = 1000, // 10 seconds at 100 Hz
    };

    data_g.kf.cfg = cfg;

    err = eskf_init(&data_g.kf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SYS, "ESKF initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    return err;
}

// Reset pyro channel to LOW
static void pyro_off_callback(void *arg) {
    pyro_channel_t *ch = (pyro_channel_t *)arg;
    gpio_set_level(ch->gpio, LOW);
    ESP_LOGI(TAG_SYS, "%s pyro channel off", ch->label);
}

static esp_err_t setup_pyro(pyro_channel_t *ch) {
    esp_err_t err = ESP_OK;

    esp_timer_create_args_t args = {
        .callback = pyro_off_callback,
        .arg      = ch,
        .name     = "pyro_off",
    };

    err = esp_timer_create(&args, &ch->timer);

    return err;
}

static bool check_for_format_mode(void) {
    if (gpio_get_level(BOOT_GPIO) == LOW) {
        int64_t time = esp_timer_get_time();
        while (gpio_get_level(BOOT_GPIO) == LOW) {
            if (esp_timer_get_time() - time > 5000000LL) {
                ESP_LOGW("RESET", "Button pressed for 5 seconds. Formatting...");
                // Signal format mode
                for (uint32_t i = 0; i < 2; ++i) {
                    gpio_set_level(LED_GPIO, HIGH);
                    gpio_set_level(BUZZER_GPIO, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(LED_GPIO, LOW);
                    gpio_set_level(BUZZER_GPIO, LOW);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
                return true;
            }
            vTaskDelay(10);
        }
    }
    return false;
}

void task_event_check(void *pvParameters) {
    EventBits_t evt;

    while (true) {
        if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        switch (evt) {
        default:
            break;
        }
    }
}

void task_setup(void *pvParameters) {
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG_SYS, "GPIO configuration");
    err = setup_peripherals();
    if (err != ESP_OK)
        goto setup_error;

    ESP_LOGI(TAG_SYS, "NVS flash init");
    err = setup_nvs(FORMAT_MODE);
    if (err != ESP_OK)
        goto setup_error;

    ESP_LOGI(TAG_SYS, "ESKF init");
    err = setup_eskf();
    if (err != ESP_OK)
        goto setup_error;

    ESP_LOGI(TAG_SYS, "Drogue pyro channel init");
    err = setup_pyro(&pyro_drogue);
    if (err != ESP_OK)
        goto setup_error;

    ESP_LOGI(TAG_SYS, "Main pyro channel init");
    err = setup_pyro(&pyro_main);
    if (err != ESP_OK)
        goto setup_error;

    // TEMPORARIO
    atomic_fetch_or(&sys_flags_g, STATUS_INITIALIZED);

    EventBits_t init_bits = xEventGroupWaitBits(xInitEventGroup, SETUP_INIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(SETUP_TIMEOUT_MS));

    if ((init_bits & SETUP_INIT) == SETUP_INIT) {
        status_event_t evt = EVT_SETUP_OK;
        xQueueSend(xEventQueue, &evt, portMAX_DELAY);
    } else {
        goto setup_error;
    }

    /* Wait for full initialization then wait for format input */
    const EventBits_t bits_to_wait = EVT_SD_DONE | EVT_LFS_DONE;

    do {
        bool format_mode = check_for_format_mode();
        setup_nvs(format_mode);
        // If format is true, format SD and LittleFS, then restart
        if (format_mode) {
            xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
            xTaskCreate(task_lfs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 5, NULL);
            xEventGroupWaitBits(xFormatEventGroup, bits_to_wait, pdTRUE, pdTRUE, portMAX_DELAY);
            ESP_LOGW(TAG_SYS, "Restarting after format...");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (gpio_get_level(RBF_GPIO) == LOW); // While not armed

    vTaskDelete(NULL);

setup_error:
    {
        status_event_t evt = EVT_SETUP_FAILED;
        xQueueSend(xEventQueue, &evt, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

// task_buzzer_led blinks LED and beeps buzzer to indicate status
void task_buzzer_led(void *pvParameters) {
    while (true) {
        bool landed = (atomic_load(&data_g.flight_state) == STATE_LANDED);

        gpio_set_level(LED_GPIO, HIGH);
        if (landed)
            gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, LOW);
        if (landed)
            gpio_set_level(BUZZER_GPIO, LOW);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_log(void *pvParameters) {
    data_t data;

    while (true) {
        portENTER_CRITICAL(&xDATALock);
        data = data_g;
        portEXIT_CRITICAL(&xDATALock);

        ESP_LOGD("LOG",
                 "\n\tTime (ms):\t\t%lu\r\n"
                 "\tStatus:\t\t\t%s\r\n"
                 "\tVoltage (V):\t\t%.1f\r\n"
                 "\tBMP pressure (Pa):\t%.2f\r\n"
                 "\tBMP altitude (m):\t%.2f\r\n"
                 "\tAccel (g):\t\tax: %.2f, ay: %.2f, az: %.2f\r\n"
                 "\tGyro (°/s):\t\tgx: %.2f, gy: %.2f, gz: %.2f\r\n"
                 "\tMag (µT):\t\tmx: %.2f, my: %.2f, mz: %.2f\r\n"
                 "\t|Accel| (g):\t\t%.1f\r\n"
                 "\tGPS coord (°):\t\tLat: %.7f, Lon: %.7f\r\n"
                 "\tGPS altitude (m):\t%.2f\r\n"
                 "\tGPS velZ (m/s):\t\t%.2f\r\n"
                 "\tGPS sAcc (m/s):\t\t%.1f\r\n"
                 "\tGPS fix:\t\t%d\r\n"
                 "\tOrientation\t\tq1: %.5f, q2: %.5f, q3: %.5f, q4: %.5f\r\n"
                 "\tKalman alt (m):\t\t%.2f\r\n"
                 "\tKalman velZ (m/s):\t%.2f\r\n"
                 "\tKalman apogee (m):\t%.2f\r\n"
                 "\tKalman ba (m/s²):\t%.5f\r\n"
                 "\tKalman bb (m):\t\t%.5f\r\n"
                 "\tKalman θe (rad):\t%.5f\r\n"
                 "-------------------------------------------------------------------------------------",
                 data.time, flight_state_to_string(data.flight_state), data.voltage * 0.1f, data.bmp.pressure, data.bmp.altitude,
                 data.icm.accel_x * ACC_SCALE, data.icm.accel_y * ACC_SCALE, data.icm.accel_z * ACC_SCALE, data.icm.gyro_x * GYRO_SCALE,
                 data.icm.gyro_y * GYRO_SCALE, data.icm.gyro_z * GYRO_SCALE, data.icm.mag_x * MAG_SCALE, data.icm.mag_y * MAG_SCALE,
                 data.icm.mag_z * MAG_SCALE, data.icm.accel * 0.1f, data.gps.latitude / 1e7, data.gps.longitude / 1e7, data.gps.altitude,
                 data.gps.vel_vertical, data.gps.sAcc * 0.1f, data.gps.fix, data.icm.q1, data.icm.q2, data.icm.q3, data.icm.q4, data.kf.x.h,
                 data.kf.x.vz, data.kf.x.apogee, data.kf.x.ba, data.kf.x.bb, data.kf.x.θe);
        vTaskDelay(pdMS_TO_TICKS(300));

        bool landed = (atomic_load(&data_g.flight_state) == STATE_LANDED);

        // if (landed)
        //     break;
    }

    vTaskDelete(NULL);
}