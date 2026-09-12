#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t handle);
void vTaskDelayUntil(TickType_t *previous, TickType_t period);
#endif
