#include "global.h"

static const char *TAG_SD  = "SD";
static const char *TAG_LFS = "LittleFS";
static const char *TAG_NVS = "NVS";

/* SD & LITTLEFS CONFIG */
#define SD_MAX_FILES    5
#define SD_MOUNT        "/sdcard"
#define SD_BUFFER_SIZE  4096
#define SD_UNIT_SIZE    32 * 1024
#define LFS_MAX_FILES   32
#define LFS_BUFFER_SIZE 512
#define LFS_MAX_FLASH   0.9 // Maximum percentage of flash to be used by littlefs
#define FILENAME_LENGTH 32

#define QUEUE_STOP_THRESHOLD_MS 100

void task_sd(void *pvParameters) {
    esp_err_t     err;
    sdmmc_card_t *card;

    bool sd_mounted = false;

    ESP_LOGI(TAG_SD, "Initializing SD card");

    /* SPI host driver (4-bit mode enabled, max frequency set to 20MHz) */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    /* SD SPI device config */
    sdspi_device_config_t sd_cfg = {
        .host_id          = SPI2_HOST,
        .gpio_cs          = SD_CS,
        .gpio_cd          = SDSPI_SLOT_NO_CD,
        .gpio_wp          = SDSPI_SLOT_NO_WP,
        .gpio_int         = GPIO_NUM_NC,
        .gpio_wp_polarity = SDSPI_IO_ACTIVE_LOW,
        .duty_cycle_pos   = 0,
        .wait_for_miso    = 0,
    };

    /* Options for mounting file system */
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = SD_MAX_FILES,
        .allocation_unit_size   = SD_UNIT_SIZE,
    };

    /* Mount filesystem */
    ESP_LOGI(TAG_SD, "Mounting filesystem");
    err = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &sd_cfg, &mount_cfg, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to mount SD card: %s.", esp_err_to_name(err));
        goto setup_error;
    }

    ESP_LOGI(TAG_SD, "Filesystem mounted");
    sdmmc_card_print_info(stdout, card);
    sd_mounted = true;

    /* SD format mode */
    if (file_counter_g.format == true) {
        ESP_LOGW(TAG_SD, "Format mode enabled, formatting SD card");
        err = esp_vfs_fat_sdcard_format(SD_MOUNT, card);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_SD, "Failed to format SD card: %s", esp_err_to_name(err));
            goto setup_error;
        }
        goto format_device;
    }

    /* Create log file */
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/test%lu.bin", SD_MOUNT, file_counter_g.sd_files);
    ESP_LOGI(TAG_SD, "Creating file %s", log_name);

    FILE *f = fopen(log_name, "wb");
    if (!f) {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        goto cleanup;
    }

    /* Write header */
    file_header_t sd_header = {
        .name_check = 0xABCD1234,
        .timestamp  = (uint32_t)esp_timer_get_time(),
    };

    static uint8_t write_buffer[SD_BUFFER_SIZE];
    uint16_t       buffer_offset = 0;
    save_t         save_data;
    TickType_t     last_sync = xTaskGetTickCount();

    /* Flight data save loop */
    while (true) {
        // ================================= REVISÃO =================================
        // Read data from queue
        if (xQueueReceive(xSDQueue, &save_data, pdMS_TO_TICKS(QUEUE_STOP_THRESHOLD_MS)) == pdTRUE) {
            // If buffer is full, write to file
            if (buffer_offset + sizeof(save_t) > SD_BUFFER_SIZE) {
                size_t w = fwrite(write_buffer, 1, buffer_offset, f);
                if (w != buffer_offset)
                    ESP_LOGE(TAG_SD, "Failed to write data to file");
                else
                    ESP_LOGD(TAG_SD, "Data written to SD card");
                buffer_offset = 0; // Reset buffer index for next batch
                taskYIELD();       // Yield to allow other tasks to run

                if (xTaskGetTickCount() - last_sync >= pdMS_TO_TICKS(5000)) // Flushes every 5s
                {
                    fflush(f);
                    fsync(fileno(f));
                    last_sync = xTaskGetTickCount();
                    ESP_LOGD(TAG_SD, "File flushed");
                    taskYIELD(); // Yield to allow other tasks to run
                }
            }
            // Copy remaining data to buffer
            memcpy(&write_buffer[buffer_offset], &save_data, sizeof(save_t));
            buffer_offset += sizeof(save_t);
        }

        // Check if landing
        bool landing = (atomic_load(&data_g.flight_state) >= STATE_LANDING);
        if (landing)
            break;
    }

    if (buffer_offset > 0) // Write remaining data to file
    {
        size_t w = fwrite(write_buffer, 1, buffer_offset, f);
        if (w != buffer_offset)
            ESP_LOGE(TAG_SD, "Failed to write remaining data to file");
        else
            ESP_LOGD(TAG_SD, "Remaining data written to SD card");

        buffer_offset = 0;
    }

    uint32_t b4_samples = 0;

    // Write before launch queue data to file
    while (xQueueReceive(xB4LaunchQueue, &save_data, 0) == pdTRUE) {
        if (buffer_offset + sizeof(save_t) > SD_BUFFER_SIZE) {
            size_t w = fwrite(write_buffer, 1, buffer_offset, f);
            if (w != buffer_offset)
                ESP_LOGE(TAG_SD, "Failed to write before launch data to file");
            buffer_offset = 0;
        }
        memcpy(&write_buffer[buffer_offset], &save_data, sizeof(save_t));
        buffer_offset += sizeof(save_t);
        b4_samples++;
    }

    if (buffer_offset > 0) {
        size_t w = fwrite(write_buffer, 1, buffer_offset, f);
        if (w != buffer_offset)
            ESP_LOGE(TAG_SD, "Failed to write remaining before launch data to file");
        buffer_offset = 0;
    }

    if (b4_samples > 0)
        ESP_LOGI(TAG_SD, "Before launch data written to SD card (%lu samples)", b4_samples);
    // ================================= REVISÃO =================================

    ESP_LOGW(TAG_SD, "Landed, closing file and unmounting SD card");

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    taskYIELD(); // Yield to allow other tasks to runs
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG_SD, "File closed");

cleanup:
    esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
    ESP_LOGI(TAG_SD, "Card unmounted");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_SD_DONE); // Signal that SD task is done
    vTaskDelete(NULL);

setup_error:
    ESP_LOGE(TAG_SD, "SD init failed: %s", esp_err_to_name(err));

    if (sd_mounted) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
        ESP_LOGI(TAG_SD, "Card unmounted");
    }

    status_event_t evt = EVT_SETUP_FAILED;
    xQueueSend(xEventQueue, &evt, portMAX_DELAY);
    vTaskDelete(NULL);

format_device:
    ESP_LOGW(TAG_SD, "SD card formatted");

    esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
    ESP_LOGI(TAG_SD, "Card unmounted");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_SD_DONE); // Signal that SD task is done
    vTaskDelete(NULL);
}

void task_lfs(void *pvParameters) {
    esp_err_t err;

    bool lfs_mounted = false;

    /* Settings for initializing LittleFS */
    esp_vfs_littlefs_conf_t littlefs_cfg = {
        .base_path              = "/littlefs",
        .partition_label        = "littlefs",
        .format_if_mount_failed = true,
        .dont_mount             = false,
    };

    /* LittleFS initialization */
    ESP_LOGI(TAG_LFS, "Initializing LittleFS");
    err = esp_vfs_littlefs_register(&littlefs_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LFS, "Failed to mount LittleFS: %s", esp_err_to_name(err));
        goto setup_error;
    }

    size_t lfs_size = 0;
    size_t lfs_used = 0;
    ESP_LOGI(TAG_LFS, "Filesystem mounted");
    err = esp_littlefs_info(littlefs_cfg.partition_label, &lfs_size, &lfs_used);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LFS, "Failed to get LittleFS partition information");
        goto setup_error;
    }
    lfs_mounted = true;

    ESP_LOGI(TAG_LFS, "Partition size: %d/%d (%.2f%%)", lfs_used, lfs_size, (float)(lfs_used / lfs_size * 100.0f));

    /* LittleFS format mode */
    if (file_counter_g.format == true) {
        ESP_LOGW(TAG_LFS, "Format mode enabled, formatting LittleFS");
        err = esp_littlefs_format(littlefs_cfg.partition_label);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_LFS, "Failed to format LittleFS: %s", esp_err_to_name(err));
            goto setup_error;
        }
        goto format_device;
    }

    /* Create log file */
    char log_name[FILENAME_LENGTH];
    snprintf(log_name, FILENAME_LENGTH, "%s/flight%lu.bin", littlefs_cfg.base_path, file_counter_g.lfs_files);
    ESP_LOGI(TAG_LFS, "Created file %s", log_name);

    FILE *f = fopen(log_name, "wb");
    if (!f) {
        ESP_LOGE(TAG_LFS, "Failed to open file for writing");
        goto cleanup;
    }

    /* Write header */
    file_header_t lfs_header = {
        .name_check = 0xABCD5678,
        .timestamp  = (uint32_t)esp_timer_get_time(),
    };

    static uint8_t buffer[LFS_BUFFER_SIZE];
    uint16_t       buffer_offset = 0;
    save_t         save_data;
    bool           _lfs_full;

    while (true) {
        _lfs_full = atomic_load_explicit(&lfs_full, memory_order_relaxed);
        // Read data from queue
        if (xQueueReceive(xLittleFSQueue, &save_data, pdMS_TO_TICKS(QUEUE_STOP_THRESHOLD_MS)) == pdTRUE) {
            // If buffer is full, write to file
            if (buffer_offset + sizeof(save_t) > LFS_BUFFER_SIZE) {
                if (!_lfs_full) {
                    if (lfs_used + buffer_offset > LFS_MAX_FLASH * lfs_size) // Check if there's space before writing
                    {
                        ESP_LOGW(TAG_LFS, "Flash memory almost full.");
                        _lfs_full = true;
                        atomic_store_explicit(&lfs_full, true, memory_order_relaxed);
                    } else {
                        size_t w = fwrite(buffer, 1, buffer_offset, f);
                        if (w != buffer_offset)
                            ESP_LOGE(TAG_LFS, "Failed to write data to file");
                        else
                            ESP_LOGD(TAG_LFS, "Data written to LittleFS");
                        lfs_used += sizeof(buffer); // Update used space tracker
                        taskYIELD();                // Yield to allow other tasks to run
                    }
                }
                buffer_offset = 0; // Reset buffer index for next batch (or drop if full)
            }
            // Copy remaining data to buffer if we still have space available
            if (!_lfs_full) {
                memcpy(&buffer[buffer_offset], &save_data, sizeof(save_t));
                buffer_offset += sizeof(save_t);
            }
        }

        // Check if landed
        bool landed = (atomic_load(&data_g.flight_state) >= STATE_LANDED);
        if (landed)
            break;
    }

    if (buffer_offset > 0) // Write remaining data to file
    {
        if (_lfs_full || lfs_used + buffer_offset > LFS_MAX_FLASH * lfs_size) // Check if there's space before writing
            ESP_LOGW(TAG_LFS, "Flash memory almost full. Remaining data not written.");
        else {
            size_t w = fwrite(buffer, 1, buffer_offset, f);
            if (w != buffer_offset)
                ESP_LOGE(TAG_LFS, "Failed to write remaining data to file");
            else
                ESP_LOGD(TAG_LFS, "Remaining data written to LittleFS");
            lfs_used += buffer_offset; // Update used space tracker
        }
    }

    ESP_LOGW(TAG_LFS, "Landed, closing file and unmounting LittleFS");

    fclose(f);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG_LFS, "File closed");

cleanup:
    esp_vfs_littlefs_unregister(littlefs_cfg.partition_label);
    ESP_LOGI(TAG_LFS, "LittleFS unmounted");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_LFS_DONE); // Signal that LFS task is done
    vTaskDelete(NULL);

setup_error:
    ESP_LOGE(TAG_LFS, "LFS init failed: %s", esp_err_to_name(err));

    if (lfs_mounted) {
        esp_vfs_littlefs_unregister(littlefs_cfg.partition_label);
        ESP_LOGI(TAG_LFS, "LittleFS unmounted");
    }

    status_event_t evt = EVT_SETUP_FAILED;
    xQueueSend(xEventQueue, &evt, portMAX_DELAY);
    vTaskDelete(NULL);

format_device:
    ESP_LOGW(TAG_LFS, "LittleFS formatted");

    esp_vfs_littlefs_unregister(littlefs_cfg.partition_label);
    ESP_LOGI(TAG_LFS, "LittleFS unmounted");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_LFS_DONE); // Signal that LFS task is done
    vTaskDelete(NULL);
}

void task_nvs(void *pvParameters) {
    nvs_handle_t nvs_handle;

    /* Wait for SD and LFS to finish */
    const EventBits_t save_bits = EVT_SD_DONE | EVT_LFS_DONE;
    xEventGroupWaitBits(xNVSCounterEventGroup, save_bits, pdTRUE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG_NVS, "Starting NVS file counter update");
    nvs_open("storage", NVS_READWRITE, &nvs_handle);

    /* Increment file counter */
    file_counter_g.sd_files += 1;
    file_counter_g.lfs_files += 1;

    /* Update NVS */
    nvs_set_u32(nvs_handle, "sd_counter", file_counter_g.sd_files);
    nvs_set_u32(nvs_handle, "lfs_counter", file_counter_g.lfs_files);
    nvs_commit(nvs_handle);

    nvs_close(nvs_handle);

    ESP_LOGI(TAG_NVS, "NVS file counter updated");

    xEventGroupSetBits(xNVSCounterEventGroup, EVT_NVS_DONE);
    vTaskDelete(NULL);
}