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

#ifdef ENABLE_SX1276
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_intr_alloc.h"
#include "sx127x.h"

#ifdef CONFIG_SX127X_BW_7800
#define SX127X_BW 0b00000000
#elif CONFIG_SX127X_BW_10400
#define SX127X_BW 0b00010000
#elif CONFIG_SX127X_BW_15600
#define SX127X_BW 0b00100000
#elif CONFIG_SX127X_BW_20800
#define SX127X_BW 0b00110000
#elif CONFIG_SX127X_BW_31200
#define SX127X_BW 0b01000000
#elif CONFIG_SX127X_BW_41700
#define SX127X_BW 0b01010000
#elif CONFIG_SX127X_BW_62500
#define SX127X_BW 0b01100000
#elif CONFIG_SX127X_BW_125000
#define SX127X_BW 0b01110000
#elif CONFIG_SX127X_BW_250000
#define SX127X_BW 0b10000000
#elif CONFIG_SX127X_BW_500000
#define SX127X_BW 0b10010000
#else
#define SX127X_BW 0b01110000
#endif

#ifdef CONFIG_SX127X_SF_6
#define SX127X_SF 0b01100000
#elif CONFIG_SX127X_SF_7
#define SX127X_SF 0b01110000
#elif CONFIG_SX127X_SF_8
#define SX127X_SF 0b10000000
#elif CONFIG_SX127X_SF_9
#define SX127X_SF 0b10010000
#elif CONFIG_SX127X_SF_10
#define SX127X_SF 0b10100000
#elif CONFIG_SX127X_SF_11
#define SX127X_SF 0b10110000
#elif CONFIG_SX127X_SF_12
#define SX127X_SF 0b11000000
#else
#define SX127X_SF 0b10010000
#endif
#endif

#ifdef ENABLE_E220
#include "driver/uart.h"
#define E220_BAUD_RATE 115200
#endif

#endif

void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
#ifdef ENABLE_LORA
void task_lora(void *pvParameters);
#endif