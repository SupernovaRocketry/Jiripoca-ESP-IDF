#include "global.h"

static const char *TAG_LORA = "LoRa";

/* LORA CONFIG */
#define LORA_BAUDRATE        9600
#define LORA_RATE_MS         200 // 5Hz
#define LORA_AUX_TIMEOUT_MS  1200
#define LORA_UART_TIMEOUT_MS 50
#define LORA_WAIT_AUX_LOW_MS 5

static void IRAM_ATTR lora_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(xTaskLora, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

static bool lora_wait_aux_high(void) {
    ulTaskNotifyTake(pdTRUE, 0); // Drain stale count

    if (gpio_get_level(LORA_AUX))
        return true;

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LORA_AUX_TIMEOUT_MS)) != 0)
        return true;

    ESP_LOGW(TAG_LORA, "AUX timeout");
    return false;
}

static bool lora_send_packet(const send_t *pkt) {
    if (pkt == NULL)
        return false;

    const uint8_t *buf   = (const uint8_t *)pkt;
    const size_t   total = sizeof(send_t);

    // Waits for LoRa to be ready
    if (!lora_wait_aux_high()) {
        ESP_LOGW(TAG_LORA, "LoRa busy before TX");
        return false;
    }

    int written = uart_write_bytes(LORA_UART_NUM, (const char *)buf, total); // Writes packet to UART at once
    if (written != total) {
        ESP_LOGE(TAG_LORA, "UART write failed (%d/%d)", written, total);
        return false;
    }

    // Wait write_bytes and AUX LOW
    uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(LORA_UART_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(LORA_WAIT_AUX_LOW_MS));

    // Waits for LoRa to finish transmission
    if (!lora_wait_aux_high()) {
        ESP_LOGW(TAG_LORA, "TX not confirmed by AUX");
        return false;
    }

    return true;
}

static esp_err_t e220_set_config(void) {
    const uint8_t config_cmd[] = {
        0xC2, // temporary register
        0x00, // starting address
        0x08, // length
        0xFF, // ADDH
        0xFF, // ADDL, no address filtering
        0x60, // REG0 (0b01100000: 9600 baud, 8N1, 2.4k ADR)
        0xC0, // REG1 (0b11000000: 32 bytes sub-packet, disable RSSI Ambient noise, 22dBm)
        0x41, // REG2 (850.125 + CH*1M = 915.125Mhz)
        0x00, // REG3 (0b00000000: disable RSSI byte, transparent transmission mode, disable LBT, WOR cycle not
              // applicable)
        0x00, // CRYPT_H (encryption key MSB)
        0x00, // CRYPT_L (encryption key LSB)
    };

    // Flush UART to clear any residual data
    uart_flush(LORA_UART_NUM);

    // Wait for AUX before writing config
    vTaskDelay(pdMS_TO_TICKS(50));
    lora_wait_aux_high();

    int written = uart_write_bytes(LORA_UART_NUM, (const char *)config_cmd, sizeof(config_cmd));
    if (written != sizeof(config_cmd)) {
        ESP_LOGE(TAG_LORA, "UART write failed (%d/%d)", written, sizeof(config_cmd));
        return ESP_FAIL;
    }

    // Wait write_bytes and AUX LOW
    uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(LORA_UART_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(LORA_WAIT_AUX_LOW_MS));

    // Check response
    uint8_t response[sizeof(config_cmd)];
    lora_wait_aux_high(); // Wait for AUX before reading response
    uart_read_bytes(LORA_UART_NUM, response, sizeof(config_cmd), pdMS_TO_TICKS(100));
    if (response[0] != 0xC1) {
        ESP_LOGE(TAG_LORA, "Failed to set LoRa configuration, response 0x%02X", response[0]);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t lora_init(void) {
    esp_err_t err;

    const uart_config_t lora_uart_cfg = {
        .baud_rate = LORA_BAUDRATE,       // E220 default baudrate is 9600
        .data_bits = UART_DATA_8_BITS,    // 8
        .stop_bits = UART_STOP_BITS_1,    // 1
        .parity    = UART_PARITY_DISABLE, // N
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &lora_uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_TX, LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Lora AUX GPIO config
    gpio_set_direction(LORA_AUX, GPIO_MODE_INPUT);
    gpio_pullup_en(LORA_AUX); // Enable pull-up on AUX pin
    gpio_set_intr_type(LORA_AUX, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add(LORA_AUX, lora_isr_handler, NULL);

    // Set M0 and M1 to 1 for configuration mode
    gpio_set_direction(LORA_M0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_M1, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_M0, 1);
    gpio_set_level(LORA_M1, 1);
    vTaskDelay(pdMS_TO_TICKS(LORA_WAIT_AUX_LOW_MS));
    lora_wait_aux_high();

    err = e220_set_config();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LORA, "LoRa initialization failed.");
        return ESP_FAIL;
    }

    // Set M0 and M1 to 0 for normal mode
    gpio_set_level(LORA_M0, 0);
    gpio_set_level(LORA_M1, 0);
    vTaskDelay(pdMS_TO_TICKS(LORA_WAIT_AUX_LOW_MS));
    lora_wait_aux_high();

    ESP_LOGI(TAG_LORA, "LoRa UART initialized (baud %d)", LORA_BAUDRATE);
    return ESP_OK;
}

void task_lora(void *pvParameters) {
    esp_err_t  err;
    send_t     send_data = {0};
    TickType_t xLastWakeTime;

    err = lora_init();
    if (err != ESP_OK) {
        goto setup_error;
    }

    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        xQueuePeek(xLoraQueue, &send_data, 0); // Non-blocking peek, will use last data if queue hasn't been updated yet

        if (!lora_send_packet(&send_data))
            ESP_LOGW(TAG_LORA, "Failed to send LoRa packet — discarded");

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LORA_RATE_MS));
    }

cleanup:
    gpio_intr_disable(LORA_AUX);
    gpio_isr_handler_remove(LORA_AUX);

    vTaskDelete(NULL);

setup_error:
    {
        status_event_t evt = EVT_SETUP_FAILED;
        xQueueSend(xEventQueue, &evt, portMAX_DELAY);
    }
    goto cleanup;
}