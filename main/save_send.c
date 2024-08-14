#include "save_send.h"

// TAGS
static const char *TAG_LITTLEFS = "LittleFS";
static const char *TAG_SD = "SD Card";
static const char *TAG_LORA = "LoRa";

// task_sd reads data from queue and writes it to SD card
void task_sd(void *pvParameters)
{
    file_counter_t counter = *(file_counter_t *)pvParameters;
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_SD_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = "/sdcard";
    ESP_LOGI(TAG_SD, "Initializing SD card");
    // Use settings defined above to initialize SD card and mount FAT filesystem.

    ESP_LOGI(TAG_SD, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SD, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG_SD, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem. "
                             "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG_SD, "Failed to initialize the card (%s). ",
                     esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG_SD, "Filesystem mounted");

    // Format mode
    if (counter.format == pdTRUE)
    {
        ret = esp_vfs_fat_sdcard_format(mount_point, card);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG_SD, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        }
        else
            ESP_LOGI(TAG_SD, "Format Successful");
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
        vTaskDelete(NULL);
    }

    // Print sd card info
    sdmmc_card_print_info(stdout, card);

    // Create log file
    char log_name[32];
    snprintf(log_name, 32, "%s/flight%ld.bin", mount_point, counter.file_num);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);
    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
    }
    fclose(f);

    while (1)
    {
        data_t data;
        data_t buffer[CONFIG_SD_BUFFER_SIZE / sizeof(data_t)];

        // Read data from queue
        for (int i = 0; i < CONFIG_SD_BUFFER_SIZE / sizeof(data_t); i++)
        {
            // Check if landed
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            if (STATUS & LANDED)
            {
                xSemaphoreGive(xStatusMutex);
                ESP_LOGW(TAG_SD, "Landed, unmounting SD card");
                esp_vfs_fat_sdcard_unmount(mount_point, card);
                ESP_LOGI(TAG_SD, "Card unmounted");
                vTaskDelete(NULL); // Delete task
            }
            else
                xSemaphoreGive(xStatusMutex);

            xQueueReceive(xSDQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }

        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG_SD, "Failed to open file for writing");
        }
        fwrite(buffer, sizeof(data_t), CONFIG_SD_BUFFER_SIZE / sizeof(data_t), f);
        fclose(f);
        ESP_LOGI(TAG_SD, "Data written to SD card");
    }
}

// task_littlefs reads data from queue and writes it to LittleFS
void task_littlefs(void *pvParameters)
{
    file_counter_t counter = *(file_counter_t *)pvParameters;

    // LittleFS INIT

    ESP_LOGW(TAG_LITTLEFS, "Initializing LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGW(TAG_LITTLEFS, "Partition size: total: %d, used: %d", total, used);
    }

    // Format mode
    if (counter.format == pdTRUE)
    {
        ret = esp_littlefs_format(conf.partition_label);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to format LittleFS (%s)", esp_err_to_name(ret));
        }
        else
            ESP_LOGI(TAG_LITTLEFS, "Format Successful");
        vTaskDelete(NULL);
    }

    // Create log file
    char log_name[32];
    snprintf(log_name, 32, "%s/flight%ld.bin", conf.base_path, counter.file_num);
    ESP_LOGI(TAG_LITTLEFS, "Creating file %s", log_name);
    FILE *f = fopen(log_name, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
    }

    uint32_t oldest_file_num = counter.file_num;

    while (1)
    {
        data_t data;
        data_t buffer[CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t)];

        // Read data from queue
        for (int i = 0; i < CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t); i++)
        {
            // Check if landed
            xSemaphoreTake(xStatusMutex, portMAX_DELAY);
            if (STATUS & LANDED)
            {
                xSemaphoreGive(xStatusMutex);
                ESP_LOGW(TAG_LITTLEFS, "Landed, unmounting LittleFS");
                esp_vfs_littlefs_unregister(conf.partition_label);
                ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
                vTaskDelete(NULL);
            }
            else
                xSemaphoreGive(xStatusMutex);

            xQueueReceive(xLittleFSQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }

        // Delete oldest file if disk space is full
        while (used + sizeof(buffer) > MAX_USED * total)
        {
            oldest_file_num++;
            if (oldest_file_num > CONFIG_MAX_LFS_FILES)
                oldest_file_num = 0;
            char oldest_file_name[32];
            snprintf(oldest_file_name, 32, "%s/flight%ld.bin", conf.base_path, oldest_file_num);

            struct stat st;
            if (stat(oldest_file_name, &st) == 0) // If file exists
            {
                ESP_LOGW(TAG_LITTLEFS, "Deleting file %s", oldest_file_name);
                unlink(oldest_file_name); // Delete file
            }

            esp_littlefs_info(conf.partition_label, &total, &used); // Update used space

            if (oldest_file_num == counter.file_num) // If oldest file is current file
            {
                ESP_LOGE(TAG_LITTLEFS, "No more disk space, unmounting LittleFS");

                esp_vfs_littlefs_unregister(conf.partition_label);
                ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");

                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LFS_FULL;
                xSemaphoreGive(xStatusMutex);

                vTaskDelete(NULL); // Delete task
            }
        }

        // Write buffer to file
        f = fopen(log_name, "a");
        if (f == NULL)
        {
            ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
        }
        fwrite(buffer, sizeof(data_t), CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t), f);
        fclose(f);
        used += sizeof(buffer);
        ESP_LOGI(TAG_LITTLEFS, "Data written to LittleFS.");
    }
}

#ifdef ENABLE_SX1276
sx127x *device = NULL;
TaskHandle_t handle_interrupt;
int32_t tx_ready = pdFALSE;
SemaphoreHandle_t xTXMutex;

// handle_interrupt_fromisr resumes handle_interrupt_task
void IRAM_ATTR handle_interrupt_fromisr(void *arg)
{
    xTaskResumeFromISR(handle_interrupt);
}

// handle_interrupt_task handles interrupts from sx127x
void handle_interrupt_task(void *arg)
{
    while (1)
    {
        vTaskSuspend(NULL);
        sx127x_handle_interrupt((sx127x *)arg);
    }
}

// tx_callback is called when sx127x is ready to transmit
void tx_callback(sx127x *device)
{
    xSemaphoreTake(xTXMutex, portMAX_DELAY);
    tx_ready = pdTRUE;
    xSemaphoreGive(xTXMutex);
    ESP_LOGI(TAG_LORA, "tx ready");
}

// task_lora reads data from queue and sends it to Lora module
void task_lora(void *pvParameters)
{
    // sx127x INIT
    // Reset pin init
    gpio_reset_pin(CONFIG_SX127X_RST);
    gpio_set_direction(CONFIG_SX127X_RST, GPIO_MODE_OUTPUT);

    // Mutex for tx_ready
    xTXMutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG_LORA, "starting up");

    // SPI init
    spi_bus_config_t config = {
        .mosi_io_num = CONFIG_SX127X_MOSI,
        .miso_io_num = CONFIG_SX127X_MISO,
        .sclk_io_num = CONFIG_SX127X_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &config, SPI_DMA_CH_AUTO));

    // SPI device init
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 9E6,
        .spics_io_num = CONFIG_SX127X_SS,
        .queue_size = 16,
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0};
    spi_device_handle_t spi_device;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev_cfg, &spi_device));

    // Lora reset
    gpio_set_level(CONFIG_SX127X_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(CONFIG_SX127X_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, device));
    ESP_ERROR_CHECK(sx127x_set_frequency(CONFIG_SX127X_FREQUENCY, device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(device)); // Reset transmit buffer
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127X_BW, device)); // Set bandwidth
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, device));
    ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127X_SF, device)); // Set spreading factor
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, device));
    sx127x_tx_set_callback(tx_callback, device); // Set callback function that is called when sx127x finishes transmitting

    // Create interrupt handling task and interrupt
    BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle interrupt", 8196, device, 2, &handle_interrupt, xPortGetCoreID());
    if (task_code != pdPASS)
    {
        ESP_LOGE(TAG_LORA, "can't create task %d", task_code);
        sx127x_destroy(device);
        return;
    }
    gpio_set_direction((gpio_num_t)CONFIG_SX127X_DIO0, GPIO_MODE_INPUT);
    gpio_pulldown_en((gpio_num_t)CONFIG_SX127X_DIO0);
    gpio_pullup_dis((gpio_num_t)CONFIG_SX127X_DIO0);
    gpio_set_intr_type((gpio_num_t)CONFIG_SX127X_DIO0, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)CONFIG_SX127X_DIO0, handle_interrupt_fromisr, (void *)device);

    // Set TX config
    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, CONFIG_SX127X_POWER, device));
    sx127x_tx_header_t header = {
        .enable_crc = true,
        .coding_rate = SX127x_CR_4_5};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, device));
    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, 20, device));

    tx_callback(device);

    while (1)
    {
        data_t data;
#ifdef CONFIG_SX127X_MODE_BUFFER
        data_t buffer[CONFIG_SX127X_BUFFER_SIZE / sizeof(data_t)];
        // Read data from queue and put it in fifo
        for (int i = 0; i < CONFIG_SX127X_BUFFER_SIZE / sizeof(data_t); i++)
        {
            xQueueReceive(xLoraQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }
        xSemaphoreTake(xTXMutex, portMAX_DELAY);
        if (tx_ready == pdTRUE)
        {
            ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(buffer, sizeof(buffer), device));
            ESP_LOGI(TAG_LORA, "sending %d byte packet", CONFIG_SX127X_BUFFER_SIZE);
            ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device)); // Set sx127x to transmit mode
            tx_ready = pdFALSE;
        }
        xSemaphoreGive(xTXMutex);
#else
        xQueueReceive(xLoraQueue, &data, portMAX_DELAY);
        xSemaphoreTake(xTXMutex, portMAX_DELAY);
        if (tx_ready == pdTRUE)
        {
            ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(&data, sizeof(data_t), device));
            ESP_LOGI(TAG_LORA, "sending %d byte packet", sizeof(data_t));
            ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device)); // Set sx127x to transmit mode
            tx_ready = pdFALSE;
        }
        xSemaphoreGive(xTXMutex);
#endif
    }
}
#endif

#ifdef ENABLE_E220
int32_t tx_ready = pdTRUE;

// handle_interrupt_fromisr resumes handle_interrupt_task
void IRAM_ATTR handle_interrupt_fromisr(void *arg)
{
    xTaskResumeFromISR(xTaskLora);
}

// task_lora reads data from queue and sends it to Lora module
void task_lora(void *pvParameters)
{
    gpio_set_direction((gpio_num_t)E220_AUX, GPIO_MODE_INPUT);
    gpio_pullup_dis((gpio_num_t)E220_AUX);
    gpio_set_intr_type((gpio_num_t)E220_AUX, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)E220_AUX, handle_interrupt_fromisr, NULL);

    uart_config_t uart_config = {
        .baud_rate = CONFIG_E220_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, E220_TX, E220_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    data_t data;

    while (1)
    {
#ifdef CONFIG_E220_MODE_BUFFER
        data_t buffer[CONFIG_E220_BUFFER_SIZE / sizeof(data_t)];
        // Read data from queue and put it in fifo
        for (int i = 0; i < CONFIG_E220_BUFFER_SIZE / sizeof(data_t); i++)
        {
            xQueueReceive(xLoraQueue, &data, portMAX_DELAY);
            buffer[i] = data;
        }
        ESP_LOGI(TAG_LORA, "sending %d byte packet", CONFIG_E220_BUFFER_SIZE);
        uart_write_bytes(UART_NUM_2, (const void *)buffer, CONFIG_E220_BUFFER_SIZE);
#else
        xQueueReceive(xLoraQueue, &data, portMAX_DELAY);
        ESP_LOGI(TAG_LORA, "sending %d byte packet", sizeof(data_t));
        uart_write_bytes(UART_NUM_2, (const void *)&data, sizeof(data_t));
#endif
        vTaskSuspend(NULL);
    }
}
#endif