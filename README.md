# _Jiripoca ESP-IDF_

This is firmware for Supernova Rocketry's flight computers based on ESP-IDF. The goal of this project is to create a universal flight computer firmware that can be used in different rocketry projects.

Supported flight computer models: 

- ASR3000 V2 (ESP32-WROOM-32)

Used peripherals:

 - GPS (NMEA 0183)
 - IMU (MPU6050)
 - Barometer (BMP280)
 - SD Card (SPI)
 - LoRa (SX1276)
 - LED (GPIO)
 - Buzzer (GPIO)
 - Remove before flight switch (GPIO)

Features:

 - Data logging to SD Card
 - Data logging to ESP32's flash memory (LittleFS)
 - Data transmission via LoRa
 - Format mode (format SD Card and flash memory)

Planned features:

 - ASR3000 V3 support (ESP32-S3-WROOM-1)
 - Kalman filter
 - Flight data download via WiFi

## Prerequisites
This project requires ESP-IDF v5.1.2 or newer, with the VSCode environment. Please refer to the [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for setting up the development environment.

## Components
This project uses the following components:

 - MPU6050 (from [esp-idf-lib](https://esp-idf-lib.readthedocs.io/en/latest/index.html))
 - BMP280 (from [esp-idf-lib](https://esp-idf-lib.readthedocs.io/en/latest/index.html))

It also uses the following managed components:

 - [LittleFS](https://components.espressif.com/components/joltwallet/littlefs)
 - [sx127x](https://components.espressif.com/components/dernasherbrezon/sx127x)

## Configuration

Before building the project, you must run menuconfig to configure the project. You can do this by running the following command in the project directory: `idf.py menuconfig` or clicking the gear icon.

`menuconfig` includes several options for configuring the project, including GPIO pins, LoRa settings, logging settings and parachute deployment parameters.

Partition tables must also be selected. The `/partition` directory contains partition tables for different sizes of flash memory. Select the appropriate partition table for your module.

There may be additional configuration options in the header files.

## Flight data download from flash memory

In order to download flight data files from the flash memory, you must first dump the flash memory to a binary file. This can be done using the `parttool.py` script in the ESP-IDF. The following command can be used to dump the flash memory to a binary file:

```
python [ESP-IDF PATH]/components/partition_table/parttool.py --port "[COM PORT NUMBER]" read_partition --partition-type=data --partition-subtype=spiffs --output "littlefs.bin"
```

You can then use the [LittleFS Disk Image Viewer](https://tniessen.github.io/littlefs-disk-img-viewer/) to view the contents of the binary file. The default block size is **4096** and block count is **Disk Image Size/4096**.

After extracting the flight data files, you may use the included `decode.py` script to convert the binary files to CSV files.