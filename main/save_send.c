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
    const char* mount_point = "/sdcard";
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

    while (true)
    {
        data_t data;
        data_t buffer[CONFIG_SD_BUFFER_SIZE / sizeof(data_t)];

        // Read data from queue
        for (int i = 0; i < CONFIG_SD_BUFFER_SIZE / sizeof(data_t); ++i)
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

    while (true)
    {
        data_t data;
        data_t buffer[CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t)];

        // Read data from queue
        for (int i = 0; i < CONFIG_LITTLEFS_BUFFER_SIZE / sizeof(data_t); ++i)
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