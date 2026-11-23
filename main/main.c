#include "global.h"

static const char *TAG_MAIN = "Main";

#define STATUS_QUEUE_SIZE   20
#define SD_QUEUE_SIZE       80
#define LITTLEFS_QUEUE_SIZE 80
#define LORA_QUEUE_SIZE     1   // Overwrite queue
#define B4LAUNCH_QUEUE_SIZE 650 // Max of 13 samples/s: 650 =~ 5s of flight data

#define SD_RINGBUF_SIZE  4 * 4096
#define LFS_RINGBUF_SIZE 4 * 4096

void app_main(void) {
    ESP_LOGI(TAG_MAIN, "Starting main application");
    esp_log_level_set("spi_master", ESP_LOG_INFO);

    /* Create Queue */
    xEventQueue    = xQueueCreate(STATUS_QUEUE_SIZE, sizeof(status_event_t));
    xSDQueue       = xQueueCreate(SD_QUEUE_SIZE, sizeof(save_t));
    xLittleFSQueue = xQueueCreate(LITTLEFS_QUEUE_SIZE, sizeof(save_t));
    xLoraQueue     = xQueueCreate(LORA_QUEUE_SIZE, sizeof(send_t));
    xB4LaunchQueue = xQueueCreate(B4LAUNCH_QUEUE_SIZE, sizeof(save_t));

    /* Create Ring Buffer */
    xSDRingBuf  = xRingbufferCreate(SD_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    xLFSRingBuf = xRingbufferCreate(LFS_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);

    /* Create Mutex */
    xI2CSem = xSemaphoreCreateMutex();

    /* Create Event Group */
    xInitEventGroup       = xEventGroupCreate();
    xNVSCounterEventGroup = xEventGroupCreate();
    xFormatEventGroup     = xEventGroupCreate();

    /* Setup Tasks */
    xTaskCreatePinnedToCore(task_setup, "Setup", configMINIMAL_STACK_SIZE * 8, NULL, 10, NULL, 1);

    while ((atomic_load(&sys_flags_g) & STATUS_INITIALIZED) == 0) {
        vTaskDelay(100);
    }

    /* Peripheral Tasks */
    xTaskCreatePinnedToCore(task_gps, "GPS", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_bmp, "BMP", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_fusion, "ICM", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_acquire, "ACQUIRE", configMINIMAL_STACK_SIZE * 4, NULL, 10, &xTaskAcquire, 1);
    xTaskCreatePinnedToCore(task_flight_control, "FLIGHT", configMINIMAL_STACK_SIZE * 4, NULL, 10, &xTaskFlightControl, 0);
    xTaskCreatePinnedToCore(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_lfs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &file_counter_g, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_buzzer_led, "BUZZER/LED", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_lora, "LORA", configMINIMAL_STACK_SIZE * 2, NULL, 3, &xTaskLora, 0);
    xTaskCreatePinnedToCore(task_adc, "ADC", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_nvs, "NVS", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
    xTaskCreatePinnedToCore(task_log, "LOG", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 0);
#endif
}