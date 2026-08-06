#include "hil_vehicle_io.h"

#include "hil_logic.h"
#include "hil_state.h"
#include "main.h"
#include "cmsis_os2.h"

#define HIL_VEHICLE_IO_PERIOD_MS 10u

extern DAC_HandleTypeDef hdac1;

void HilVehicleIO_Init(void)
{
  (void)HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  (void)HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                        HIL_PEDAL_ADC_MIN);
}

void HilVehicleIO_Task(void)
{
  uint32_t next = osKernelGetTickCount();
  for (;;) {
    HilState_t state;
    if (HilState_GetSnapshot(&state, 5u)) {
      HAL_GPIO_WritePin(HIL_MODE_OUT_GPIO_Port, HIL_MODE_OUT_Pin,
                        state.auto_mode ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(HIL_GEAR_OUT_GPIO_Port, HIL_GEAR_OUT_Pin,
                        state.reverse ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(HIL_ESTOP_DASH_OUT_GPIO_Port, HIL_ESTOP_DASH_OUT_Pin,
                        state.estop_dashboard ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(HIL_ESTOP_SIDE_OUT_GPIO_Port, HIL_ESTOP_SIDE_OUT_Pin,
                        state.estop_side ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(HIL_ESTOP_RF_OUT_GPIO_Port, HIL_ESTOP_RF_OUT_Pin,
                        state.estop_rf ? GPIO_PIN_RESET : GPIO_PIN_SET);
      (void)HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                            HilLogic_PedalPercentToDac(state.pedal_percent));
    }
    next += HIL_VEHICLE_IO_PERIOD_MS;
    (void)osDelayUntil(next);
  }
}
