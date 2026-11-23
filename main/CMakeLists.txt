cmake_minimum_required(VERSION 4.0)

list(APPEND SOURCES main.c global.c acquire.c system.c flight_control.c save.c send.c 
                    sensors/adc.c sensors/bmp.c sensors/gps.c sensors/icm.c)

list(APPEND REQUIRES esp_adc esp_ringbuf esp_timer esp_driver_i2c esp_driver_uart 
                        nvs_flash sdmmc fatfs esp_bmp390 esp_littlefs-master 
                        icm20948-master minmea-master vqf-main eskf)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)