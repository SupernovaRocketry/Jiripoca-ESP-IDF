#ifndef SAVE_SEND_H
#define SAVE_SEND_H

#include "common.h"

#include "sys/stat.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "unistd.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "esp_littlefs.h"
#include "driver/uart.h"

#define MAX_USED 0.8 // Maximum percentage of flash to be used by littlefs
#define E220_BAUD_RATE 115200

#endif

void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
void task_lora(void *pvParameters);