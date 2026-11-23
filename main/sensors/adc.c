#include "global.h"

static const char *TAG_ADC = "ADC";

static void adc_init(adc_oneshot_unit_handle_t *adc_unit_handle, adc_cali_handle_t *adc_cali_handle) {
    adc_oneshot_unit_init_cfg_t unit_config = {.unit_id = VIN_ADC_UNIT, .ulp_mode = ADC_ULP_MODE_DISABLE};
    adc_oneshot_new_unit(&unit_config, adc_unit_handle);

    // Configure the ADC channel
    adc_oneshot_chan_cfg_t channel_config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_unit_handle, VIN_ADC_CHANNEL, &channel_config));

    // Configure calibration (raw ADC value in mV)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = VIN_ADC_UNIT, .chan = VIN_ADC_CHANNEL, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, adc_cali_handle));
}

void task_adc(void *pvParameters) {
    adc_oneshot_unit_handle_t adc_unit_handle;
    adc_cali_handle_t         adc_cali_handle;
    adc_init(&adc_unit_handle, &adc_cali_handle); // Initialize ADC

    while (true) {
        int       raw, voltage_mv;
        esp_err_t err = (adc_oneshot_read(adc_unit_handle, VIN_ADC_CHANNEL, &raw));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_ADC, "ADC read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
            continue;
        }

        err = (adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_ADC, "ADC calibration failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
            continue;
        }

        // Calculate actual battery voltage based on voltage divider ratio
        portENTER_CRITICAL(&xADCMutex);
        battery_voltage_g = (uint8_t)round((voltage_mv * V_DIV_RATIO)); //  Apply divider ratio, save as V * 10
        portEXIT_CRITICAL(&xADCMutex);

        xTaskNotify(xTaskAcquire, ADC_BIT, eSetBits);

        vTaskDelay(pdMS_TO_TICKS(10000)); // Read every 10 seconds
    }
}