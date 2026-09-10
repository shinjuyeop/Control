#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stddef.h>
#include <stdint.h>
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define configTICK_RATE_HZ 1000u
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
void test_critical_enter(void);
void test_critical_exit(void);
#define taskENTER_CRITICAL() test_critical_enter()
#define taskEXIT_CRITICAL() test_critical_exit()
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
#endif
