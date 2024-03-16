#ifndef ACQUIRE_H
#define ACQUIRE_H
#endif

#define G 9.80665

// MPU6050
#ifdef CONFIG_MPU6050_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

void task_acquire(void *pvParameters);