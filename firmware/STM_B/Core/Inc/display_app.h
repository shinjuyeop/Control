#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DISPLAY_UPDATE_MS 100U

void Dashboard_DrawStaticLayout(void);
void Dashboard_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_APP_H */
