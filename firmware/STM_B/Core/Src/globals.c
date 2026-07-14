/*
---- CODE DESCRIPTION ----
* 전역변수 초기값 선언
*/

#include <stdint.h>

 volatile float   CURRENT_SPEED = 0.0f; // 현재 속도(m/s) / write : uROS / read : Display
 volatile float   COMMAND_SPEED = 0.0f; // 목표 속도(m/s) / write : uROS / read : (미사용)
 
 volatile uint8_t AEB                   = 0; // Host AEB / write: uROS, read: EStop
 volatile uint8_t ESTOP_DASHBOARD       = 0; // PC0 active-low / write: EStop
 volatile uint8_t ESTOP_SIDE            = 0; // PC1 active-low / write: EStop
 volatile uint8_t ESTOP_RF              = 0; // PB4 active-low / write: EStop
 volatile uint8_t ESTOP_STATUS          = 0; // 물리 E-stop 통합 / write: EStop
 volatile uint8_t SAFETY_STOP_REQUEST   = 0; // 물리 E-stop | AEB / write: EStop
 volatile uint8_t BRAKE_REQUEST         = 0; // 브레이크 요청 / write: EStop
 volatile uint8_t BRAKE_COMMAND_APPLIED = 0; // 적용 방향 PWM 명령 / write: Brake
 
 volatile uint8_t GEAR          = 0;    // 0 : 전진 || 1 : 후진 / write : uROS / read : Display
 volatile uint8_t MODE          = 0;    // 0 : 수동 || 1 : 자율 / write : uROS / read : Display
