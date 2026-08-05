#ifndef HIL_KEYA_H
#define HIL_KEYA_H

#include "main.h"
#include <stdint.h>

#define HIL_KEYA_NODE_ID       1u
#define HIL_KEYA_COMMAND_ID    (0x06000000UL + HIL_KEYA_NODE_ID)
#define HIL_KEYA_FEEDBACK_ID   (0x05800000UL + HIL_KEYA_NODE_ID)
#define HIL_KEYA_HEARTBEAT_ID  (0x07000000UL + HIL_KEYA_NODE_ID)
#define HIL_KEYA_HEARTBEAT_MS  1000u

void HilKEYA_Init(void);
void HilKEYA_Task(void);
void HilKEYA_RxCallbackFromISR(FDCAN_HandleTypeDef *hfdcan);

#endif /* HIL_KEYA_H */
