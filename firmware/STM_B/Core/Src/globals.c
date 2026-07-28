/*
---- CODE DESCRIPTION ----
* 전역변수 초기값 선언
*/

#include "globals.h"

 volatile float    CURRENT_SPEED_FROM_A             = 0.0f;
 volatile uint8_t  GEAR_FROM_A                      = 0;
 volatile uint8_t  MODE_FROM_A                      = 0;
 volatile uint8_t  VEHICLE_FRAME_VALID_FROM_A       = 0;
 volatile uint32_t LAST_VEHICLE_RX_MS               = 0;
 volatile uint32_t VEHICLE_RX_COUNT                  = 0;
 volatile uint32_t VEHICLE_CRC_ERROR_COUNT           = 0;
 volatile uint32_t VEHICLE_SEQUENCE_ERROR_COUNT      = 0;
 
 volatile uint8_t COMMAND_BRAKE_REQUEST = 0; // /vehicle/command.brake_engage
 volatile uint8_t ESTOP_DASHBOARD       = 0; // PC0 active-low / write: EStop
 volatile uint8_t ESTOP_SIDE            = 0; // PC1 active-low / write: EStop
 volatile uint8_t ESTOP_RF              = 0; // PB4 active-low / write: EStop
 volatile uint8_t ESTOP_STATUS          = 0; // 물리 E-stop 통합 / write: EStop
 volatile uint8_t SAFETY_STOP_REQUEST   = 0; // 물리 E-stop | 명령 브레이크
 volatile uint8_t BRAKE_REQUEST         = 0; // 브레이크 요청 / write: EStop
 volatile uint8_t BRAKE_COMMAND_APPLIED = 0; // 적용 방향 PWM 명령 / write: Brake
