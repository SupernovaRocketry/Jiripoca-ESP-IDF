#ifndef SAVE_SEND_H
#define SAVE_SEND_H

#define MAX_USED 0.8 // Maximum percentage of flash to be used by littlefs

#endif

void task_sd(void *pvParameters);
void task_littlefs(void *pvParameters);
void task_lora(void *pvParameters);