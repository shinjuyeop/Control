#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DISPLAY_UPDATE_MS 50U
#define DISPLAY_RECOVERY_RETRY_MS 1000U
#define DISPLAY_GEAR_GESTURE_WINDOW_MS 1000U

extern volatile uint32_t DISPLAY_RECOVERY_COUNT;
extern volatile uint32_t DISPLAY_LAST_RECOVERY_ERROR;

void Dashboard_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_APP_H */
