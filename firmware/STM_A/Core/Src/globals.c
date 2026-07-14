/*
---- CODE DESCRIPTION ----
* 전역변수 초기값 선언
 */
 #include "globals.h"

 volatile float   COMMAND_SPEED   = 0.0f;   // 목표 속도(m/s) / write : uROS / read : Control
 volatile float   CURRENT_SPEED   = 0.0f;   // 현재 속도 (모터 피드백) / write : MotorRx / read : uROS
 
 volatile uint8_t AEB                          = 0; // Host AEB / write: uROS
 volatile uint8_t ESTOP_DASHBOARD_FROM_B       = 0; // write: BRx
 volatile uint8_t ESTOP_SIDE_FROM_B            = 0; // write: BRx
 volatile uint8_t ESTOP_RF_FROM_B              = 0; // write: BRx
 volatile uint8_t AEB_FROM_B                   = 0; // write: BRx
 volatile uint8_t BRAKE_REQUEST_FROM_B         = 0; // write: BRx
 volatile uint8_t BRAKE_COMMAND_APPLIED_FROM_B = 0; // write: BRx, diagnostic only
 volatile uint8_t DRIVE_STOP_REQUEST            = 1; // fail-safe until first valid Safety frame
 
 volatile uint8_t GEAR            = 0;      // 0 : 전진 || 1 : 후진 / write : Inputs / read : Control, uROS
 volatile uint8_t MODE            = 0;      // 0 : 수동 || 1 : 자율 / write : Inputs / read : Control, uROS
 volatile uint8_t PEDAL           = 0;      // 페달(스르틀, Analog) 0~100(%) / write : Inputs / read : Control
