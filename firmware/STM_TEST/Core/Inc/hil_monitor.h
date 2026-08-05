#ifndef HIL_MONITOR_H
#define HIL_MONITOR_H

#include "main.h"

void HilMonitor_Init(void);
void HilMonitor_Task(void);
void HilMonitor_TIMCaptureFromISR(TIM_HandleTypeDef *htim);

#endif /* HIL_MONITOR_H */
