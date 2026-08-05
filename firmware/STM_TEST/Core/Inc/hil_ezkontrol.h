#ifndef HIL_EZKONTROL_H
#define HIL_EZKONTROL_H

#include "main.h"
#include <stdint.h>

#define HIL_EZ_COMMAND_ID       0x0C01EFD0UL
#define HIL_EZ_STATUS_PART1_ID  0x1801D0EFUL
#define HIL_EZ_STATUS_PART2_ID  0x1802D0EFUL
#define HIL_EZ_PERIOD_MS        50u

typedef enum {
  EZ_STATE_WAIT_HANDSHAKE = 0,
  EZ_STATE_ACTIVE,
  EZ_STATE_FAULT
} HilEZState_t;

void HilEZKontrol_Init(void);
void HilEZKontrol_Task(void);
void HilEZKontrol_RxCallbackFromISR(FDCAN_HandleTypeDef *hfdcan);

#endif /* HIL_EZKONTROL_H */
