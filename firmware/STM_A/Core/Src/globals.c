/* globals.c — STM_A 전역변수 정의
 * 위치: STM_A/Core/Src/globals.c  (신규 생성)
 */
 #include "globals.h"

 volatile float   COMMAND_SPEED   = 0.0f;
 volatile float   CURRENT_SPEED   = 0.0f;
 
 volatile uint8_t AEB             = 0;
 volatile uint8_t ESTOP_STATUS    = 1;   /* 초기: 안전 우선 */
 volatile uint8_t ESTOP_DASHBOARD = 0;
 volatile uint8_t ESTOP_SIDE      = 0;
 volatile uint8_t ESTOP_FROM_B    = 0;
 volatile uint8_t ESTOP_TO_B      = 0;
 
 volatile uint8_t GEAR            = 0;
 volatile uint8_t MODE            = 0;
 volatile uint8_t PEDAL           = 0;