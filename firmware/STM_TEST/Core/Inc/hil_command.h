#ifndef HIL_COMMAND_H
#define HIL_COMMAND_H

#include <stdint.h>

void HilCommand_Init(void);
void HilCommand_Task(void);
void HilCommand_RxEventFromISR(uint16_t size);
void HilCommand_UartErrorFromISR(void);

#endif /* HIL_COMMAND_H */
