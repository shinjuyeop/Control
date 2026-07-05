/*
---- CODE DESCRIPTION ----
* 전역변수 초기값 선언
 */
 #include "globals.h"

 volatile float   COMMAND_SPEED   = 0.0f;   // 목표 속도(m/s) / write : uROS / read : Control
 volatile float   CURRENT_SPEED   = 0.0f;   // 현재 속도 (모터 피드백) / write : MotorRx / read : uROS
 
 volatile uint8_t AEB             = 0;      // AEB(판단) / write : uROS / read : Inputs
 volatile uint8_t ESTOP_STATUS    = 1;      // 통합 ESTOP / write : Inputs / read : Control + uROS
 volatile uint8_t ESTOP_DASHBOARD = 0;      // 계기판 ESTOP / write : Inputs / read : Inputs
 volatile uint8_t ESTOP_SIDE      = 0;      // 측면 ESTOP / write : Inputs / read : Inputs
 volatile uint8_t ESTOP_FROM_B    = 0;      // RF ESTOP(STM B) / write : BRx / read : Inputs
 volatile uint8_t ESTOP_TO_B      = 0;      // B로 송신할 물리버튼값 / write : Inputs / read : BTx
 
 volatile uint8_t GEAR            = 0;      // 0 : 전진 || 1 : 후진 / write : Inputs / read : Control, uROS
 volatile uint8_t MODE            = 0;      // 0 : 수동 || 1 : 자율 / write : Inputs / read : Control, uROS
 volatile uint8_t PEDAL           = 0;      // 페달(스르틀, Analog) 0~100(%) / write : Inputs / read : Control