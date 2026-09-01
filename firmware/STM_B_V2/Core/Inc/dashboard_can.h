#ifndef DASHBOARD_CAN_H
#define DASHBOARD_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DASHBOARD_CAN_SLOT_PERIOD_MS 10U
#define DASHBOARD_CAN_SLOTS_PER_SNAPSHOT 5U

extern volatile uint32_t DASHBOARD_CAN_TX_COUNT;
extern volatile uint32_t DASHBOARD_CAN_TX_ERROR_COUNT;
extern volatile uint32_t DASHBOARD_CAN_TX_FIFO_FULL_COUNT;
extern volatile uint32_t DASHBOARD_CAN_START_ERROR_COUNT;
extern volatile uint32_t DASHBOARD_CAN_LAST_HAL_STATUS;
extern volatile uint32_t DASHBOARD_CAN_SNAPSHOT_COUNT;
extern volatile uint8_t DASHBOARD_CAN_ALIVE;

void DashboardCan_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DASHBOARD_CAN_H */
