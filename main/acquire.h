#ifndef ACQUIRE_H
#define ACQUIRE_H
#endif

#define G 9.80665

#define FLYING_THRESHOLD 2 * G // Acceleration threshold to consider rocket flying
#define CUTOFF_THRESHOLD 2 * G // Acceleration threshold to consider motor cutoff
#define LANDED_THRESHOLD 2     // Altitude threshold to consider rocket landed

// MPU6050
#ifdef CONFIG_MPU6050_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

void task_acquire(void *pvParameters);