#include "save_send.h"

// TAGS for logging
static const char *TAG_LITTLEFS = "LittleFS";
static const char *TAG_SD = "SD Card";
static const char *TAG_LORA = "LoRa";

/**
 * @brief Reads a buffer of data_t from the queue, checking for landing condition.
 * @return true if should continue, false if should stop.
 */
static bool read_sd_buffer(data_t *buffer, int buffer_len, sdmmc_card_t *card, const char *mount_point)
{
    for (int i = 0; i < buffer_len; ++i)
    {
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (STATUS & LANDED)
        {
            xSemaphoreGive(xStatusMutex);
            ESP_LOGW(TAG_SD, "Landed, unmounting SD card");
            esp_vfs_fat_sdcard_unmount(mount_point, card);
            ESP_LOGI(TAG_SD, "Card unmounted");
            vTaskDelete(NULL); // Task ends if landed
        }
        else
            xSemaphoreGive(xStatusMutex);

        xQueueReceive(xSDQueue, &buffer[i], portMAX_DELAY); // Wait for data from queue
    }
    return true;
}

/**
 * @brief Writes a buffer of data_t to the SD file.
 */
static void write_sd_buffer(const char *log_name, data_t *buffer, int buffer_len)
{
    FILE *f = fopen(log_name, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        return;
    }
    fwrite(buffer, sizeof(data_t), buffer_len, f);
    fclose(f);
    ESP_LOGI(TAG_SD, "Data written to SD card");
}

/**
 * @brief Task that reads data from the queue and saves it to the SD card.
 * @param pvParameters Pointer to file_counter_t.
 */
void task_sd(void *pvParameters)
{
    file_counter_t counter = *(file_counter_t *)pvParameters;
    esp_err_t ret;

    // SD card initialization (mount, check, etc.) should be here

    char log_name[32];
    snprintf(log_name, 32, "%s/flight%ld.bin", mount_point, counter.file_num);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);
    FILE *f = fopen(log_name, "w");
    if (f == NULL)
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
    fclose(f);

    while (true)
    {
        data_t buffer[CONFIG_SD_BUFFER_SIZE / sizeof(data_t)];

        // Read from queue and check for landing
        if (!read_sd_buffer(buffer, CONFIG_SD_BUFFER_SIZE / sizeof(data_t), card, mount_point))
            break;

        // Write buffer to SD file
        write_sd_buffer(log_name, buffer, CONFIG_SD_BUFFER_SIZE / sizeof(data_t));

        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to avoid busy loop
    }
    vTaskDelete(NULL);
}

/**
 * @brief Reads a buffer of data_t from the LittleFS queue, checking for landing.
 * @return true if should continue, false if should stop.
 */
static bool read_littlefs_buffer(data_t *buffer, int buffer_len, esp_vfs_littlefs_conf_t *conf)
{
    data_t data;
    for (int i = 0; i < buffer_len; ++i)
    {
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (STATUS & LANDED)
        {
            xSemaphoreGive(xStatusMutex);
            ESP_LOGW(TAG_LITTLEFS, "Landed, unmounting LittleFS");
            esp_vfs_littlefs_unregister(conf->partition_label);
            ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");
            vTaskDelete(NULL); // Task ends if landed
        }
        else
            xSemaphoreGive(xStatusMutex);

        xQueueReceive(xLittleFSQueue, &data, portMAX_DELAY); // Wait for data from queue
        buffer[i] = data;
    }
    return true;
}

/**
 * @brief Writes a buffer of data_t to the LittleFS file.
 */
static void write_littlefs_buffer(const char *log_name, data_t *buffer, int buffer_len)
{
    FILE *f = fopen(log_name, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");
        return;
    }
    fwrite(buffer, sizeof(data_t), buffer_len, f);
    fclose(f);
    ESP_LOGI(TAG_LITTLEFS, "Data written to LittleFS.");
}

/**
 * @brief Task that reads data from the queue and saves it to LittleFS.
 *        Also manages disk space by deleting oldest files if needed.
 * @param pvParameters Pointer to file_counter_t.
 */
void task_littlefs(void *pvParameters)
{
    file_counter_t counter = *(file_counter_t *)pvParameters;

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
            ESP_LOGE(TAG_LITTLEFS, "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG_LITTLEFS, "Failed to find LittleFS partition");
        else
            ESP_LOGE(TAG_LITTLEFS, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
        ESP_LOGE(TAG_LITTLEFS, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    else
        ESP_LOGW(TAG_LITTLEFS, "Partition size: total: %d, used: %d", total, used);

    // If format flag is set, format the partition and exit
    if (counter.format == pdTRUE)
    {
        ret = esp_littlefs_format(conf.partition_label);
        if (ret != ESP_OK)
            ESP_LOGE(TAG_LITTLEFS, "Failed to format LittleFS (%s)", esp_err_to_name(ret));
        else
            ESP_LOGI(TAG_LITTLEFS, "Format Successful");
        vTaskDelete(NULL);
    }

    // Create log file for this flight/session
    char log_name[32];
    snprintf(log_name, 32, "%s/flight%ld.bin", conf.base_path, counter.file_num);
    ESP_LOGI(TAG_LITTLEFS, "Creating file %s", log_name);
    FILE *f = fopen(log_name, "w");
    if (f == NULL)
        ESP_LOGE(TAG_LITTLEFS, "Failed to open file for writing");

    uint32_t oldest_file_num = counter.file_num;

    while (true)
    {
        data_t buffer[CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t)];

        // Read from queue and check for landing
        if (!read_littlefs_buffer(buffer, CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t), &conf))
            break;

        // If disk is full, delete oldest files until there is space
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

            // If all files are deleted and still no space, stop the task
            if (oldest_file_num == counter.file_num)
            {
                ESP_LOGE(TAG_LITTLEFS, "No more disk space, unmounting LittleFS");

                esp_vfs_littlefs_unregister(conf.partition_label);
                ESP_LOGI(TAG_LITTLEFS, "LittleFS unmounted");

                xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                STATUS |= LFS_FULL;
                xSemaphoreGive(xStatusMutex);

                vTaskDelete(NULL); // End task
            }
        }

        // Write buffer to LittleFS file
        write_littlefs_buffer(log_name, buffer, CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t));
        used += sizeof(buffer);

        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to avoid busy loop
    }
    vTaskDelete(NULL);
}

int32_t tx_ready = pdTRUE;

/**
 * @brief ISR handler for LoRa AUX pin interrupt.
 *        Resumes the LoRa task when interrupt occurs.
 */
void IRAM_ATTR handle_interrupt_fromisr(void *arg)
{
    xTaskResumeFromISR(xTaskLora);
}

/**
 * @brief Reads a buffer of data_t from the LoRa queue.
 * @param buffer Destination buffer.
 * @param buffer_len Number of data_t elements to read.
 */
static void read_lora_buffer(data_t *buffer, int buffer_len)
{
    for (int i = 0; i < buffer_len; ++i)
    {
        xQueueReceive(xLoraQueue, &buffer[i], portMAX_DELAY);
    }
}

/**
 * @brief Sends a buffer of data_t via UART to the LoRa module.
 * @param buffer Data buffer.
 * @param buffer_len Number of data_t elements.
 */
static void send_lora_buffer(data_t *buffer, int buffer_len)
{
    ESP_LOGI(TAG_LORA, "sending %d byte packet", buffer_len * sizeof(data_t));
    uart_write_bytes(UART_NUM_2, (const void *)buffer, buffer_len * sizeof(data_t));
}

/**
 * @brief Task that reads data from the queue and sends it via LoRa (UART).
 * @param pvParameters Not used.
 */
void task_lora(void *pvParameters)
{
    // Configure LoRa AUX pin and UART
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

    while (true)
    {
        data_t buffer[CONFIG_E220_BUFFER_SIZE / sizeof(data_t)];

        // Read from queue
        read_lora_buffer(buffer, CONFIG_E220_BUFFER_SIZE / sizeof(data_t));

        // Send buffer via UART/LoRa
        send_lora_buffer(buffer, CONFIG_E220_BUFFER_SIZE / sizeof(data_t));

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid busy loop
    }
    vTaskDelete(NULL);
}