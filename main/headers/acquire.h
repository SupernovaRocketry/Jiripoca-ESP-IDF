#ifndef ACQUIRE_H
#define ACQUIRE_H

#include "common.h"

#define G 9.80665

#define FLYING_THRESHOLD 10 * G // Acceleration threshold to consider rocket flying
#define CUTOFF_THRESHOLD 3 * G // Acceleration threshold to consider motor cutoff
#define LANDED_THRESHOLD 2     // Altitude threshold to consider rocket landed

#endif

void task_acquire(void *pvParameters);