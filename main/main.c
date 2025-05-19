#include "common.h"
#include "acquire.h"
#include "save_send.h"

#define HIGH 1
#define LOW 0

// Tags
static const char *TAG_DEPLOY = "Deploy";

// task_deploy deploys parachutes
void task_deploy(void *pvParameters)
{

    gpio_set_direction(DROGUE_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_direction(MAIN_GPIO, GPIO_MODE_OUTPUT);

    float current_altitude = 0;
    float max_altitude = 0;
    float start_altitude = 0;

    xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);
    start_altitude = current_altitude;

    while (true)
    {
        /*// Manual Deploy Test
        if (gpio_get_level(BUTTON_GPIO) == LOW)
        {
            gpio_set_level(MAIN_GPIO, HIGH);
            gpio_set_level(DROGUE_GPIO, HIGH);
        }
        else
        {
            gpio_set_level(MAIN_GPIO, LOW);
            gpio_set_level(DROGUE_GPIO, LOW);
        }*/

        // Get current altitude
        xQueueReceive(xAltQueue, &current_altitude, portMAX_DELAY);

        // Update max altitude
        if (current_altitude > max_altitude)
        {
            max_altitude = current_altitude;
        }

        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (!(STATUS & DROGUE_DEPLOYED)) // If drogue not deployed
        {
            if (current_altitude < max_altitude - CONFIG_DROGUE_THRESHOLD) // If altitude is DROGUE_THRESHOLD below max altitude
            {
                STATUS |= DROGUE_DEPLOYED;
                xSemaphoreGive(xStatusMutex);
                gpio_set_level(DROGUE_GPIO, HIGH);
                ESP_LOGW(TAG_DEPLOY, "Drogue deployed");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(DROGUE_GPIO, LOW);
            }
            else
                xSemaphoreGive(xStatusMutex);
        }
        else if (!(STATUS & MAIN_DEPLOYED)) // If drogue deployed but main not deployed
        {
            if (current_altitude < start_altitude + CONFIG_MAIN_ALTITUDE) // If altitude is MAIN_ALTITUDE above start altitude
            {
                STATUS |= MAIN_DEPLOYED;
                xSemaphoreGive(xStatusMutex);

                gpio_set_level(MAIN_GPIO, HIGH);
                ESP_LOGW(TAG_DEPLOY, "Main deployed");
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(MAIN_GPIO, LOW);

                // Delete task
                vTaskDelete(NULL);
            }
            else
                xSemaphoreGive(xStatusMutex);
        }
        else
            xSemaphoreGive(xStatusMutex);
    }
}

// task_buzzer_led blinks LED and beeps buzzer to indicate status
void task_buzzer_led(void *pvParameters)
{
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // When initializing, blink LED and beep buzzer 10 times
    for (uint32_t i = 0; i < 10; i++)
    {
        gpio_set_level(LED_GPIO, HIGH);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(LED_GPIO, LOW);
        gpio_set_level(BUZZER_GPIO, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI("Buzzer LED", "Initialized");

    while (true)
    {
        // Use local copy of STATUS because of delays
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        int32_t status_local = STATUS;
        xSemaphoreGive(xStatusMutex);

        // If ARMED, blink LED and beep buzzer three times
        if (status_local & ARMED)
        {
            static uint32_t i = 0;
            while (i++ < 3)
            {
                gpio_set_level(LED_GPIO, HIGH);
                gpio_set_level(BUZZER_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_GPIO, LOW);
                gpio_set_level(BUZZER_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        // If LANDED, blink LED every second
        if (status_local & LANDED)
        {
            gpio_set_level(LED_GPIO, HIGH);
            gpio_set_level(BUZZER_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, LOW);
            gpio_set_level(BUZZER_GPIO, LOW);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // GPIO Initialization

    // test with ALED
    // gpio_reset_pin(ALED_GPIO);
    // gpio_set_direction(ALED_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_direction(RBF_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RBF_GPIO, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    // Enter format mode if button is pressed for 5 seconds
    uint32_t format = pdFALSE;
    if (gpio_get_level(BUTTON_GPIO) == LOW)
    {
        uint64_t time = esp_timer_get_time();
        while (gpio_get_level(BUTTON_GPIO) == LOW)
        {
            if (esp_timer_get_time() - time > 5000000)
            {
                ESP_LOGW("RESET", "Button pressed for 5 seconds. Formatting...");
                format = pdTRUE;
                break;
            }
            vTaskDelay(10);
        }
    }

    // Create Mutexes
    xGPSMutex = xSemaphoreCreateMutex();
    xStatusMutex = xSemaphoreCreateMutex();

    // Create Queues
    xAltQueue = xQueueCreate(2, sizeof(float));
    xLittleFSQueue = xQueueCreate(2, sizeof(data_t));
    xSDQueue = xQueueCreate(2, sizeof(data_t));
    xLoraQueue = xQueueCreate(2, sizeof(data_t));

    // Initialize NVS to store file counters
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS
    ESP_LOGI("nvs", "Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t nvs_handle;
    nvs_open("storage", NVS_READWRITE, &nvs_handle);
    int32_t sd_num = 0;
    int32_t lfs_num = 0;
    nvs_get_i32(nvs_handle, "sd_counter", &sd_num);
    nvs_get_i32(nvs_handle, "lfs_counter", &lfs_num);

    // Increment file counters
    if (sd_num < CONFIG_MAX_SD_FILES)
        sd_num++;
    else
        sd_num = 0;
    if (lfs_num < CONFIG_MAX_LFS_FILES)
        lfs_num++;
    else
        lfs_num = 0;

    if (format == pdTRUE)
    {
        sd_num = 0;
        lfs_num = 0;
    }

    // Save file counters
    nvs_set_i32(nvs_handle, "sd_counter", sd_num);
    nvs_set_i32(nvs_handle, "lfs_counter", lfs_num);
    nvs_commit(nvs_handle);

    // Close NVS
    nvs_close(nvs_handle);

    // Create file counter structs for tasks
    file_counter_t counter_sd = {
        .file_num = sd_num,
        .format = format
    };
    file_counter_t counter_lfs = {
        .file_num = lfs_num,
        .format = format
    };

    // If format is true, format SD and LittleFS, then restart
    if (format == pdTRUE)
    {
        xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 8, &counter_sd, 5, NULL);
        xTaskCreate(task_littlefs, "LittleFS", configMINIMAL_STACK_SIZE * 8, &counter_lfs, 5, NULL);

        vTaskDelay(pdMS_TO_TICKS(30000));
        esp_restart();
    }

    // If RBF is off at startup, set SAFE_MODE
    if (gpio_get_level(RBF_GPIO) == LOW)
    {
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        STATUS |= SAFE_MODE;
        xSemaphoreGive(xStatusMutex);
    }

    // Start tasks
    xTaskCreate(task_acquire, "Acquire", configMINIMAL_STACK_SIZE * 4, NULL, 4, NULL);
    xTaskCreate(task_sd, "SD", configMINIMAL_STACK_SIZE * 4, &counter_sd, 5, NULL);
    xTaskCreate(task_littlefs, "LittleFS", configMINIMAL_STACK_SIZE * 4, &counter_lfs, 5, NULL);
    xTaskCreate(task_lora, "Lora", configMINIMAL_STACK_SIZE * 4, NULL, 6, &xTaskLora);
    xTaskCreate(task_buzzer_led, "Buzzer LED", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);

    while (true)
    {
        // Logic for arming parachute deployment
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        if (!(STATUS & ARMED)) // If not armed
        {
            if (!(STATUS & SAFE_MODE) && gpio_get_level(RBF_GPIO) == LOW) // If not in safe mode and RBF is off
            {
                xTaskCreate(task_deploy, "Deploy", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL); // Start deploy task
                STATUS |= ARMED;                                                                 // Set ARMED
                // test with ALED
                // gpio_set_level(ALED_GPIO, HIGH);
            }
        }
        xSemaphoreGive(xStatusMutex);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}