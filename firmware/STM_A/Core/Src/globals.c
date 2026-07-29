/*
---- CODE DESCRIPTION ----
* 전역변수 초기값 선언
 */
 #include "globals.h"

 volatile float   COMMAND_SPEED   = 0.0f;   // 목표 속도(m/s) / write : uROS / read : Control
 volatile float   CURRENT_SPEED   = 0.0f;   // 현재 속도 (모터 피드백) / write : MotorRx / read : uROS
 
 volatile uint8_t  BRAKE_REQUEST_FROM_B          = 0; // write: BRx
 volatile uint8_t  SAFETY_FRAME_VALID_FROM_B     = 0; // write: BRx
 volatile uint8_t  STM_B_COMM_TIMEOUT            = 1; // fail-safe until first valid frame
 volatile uint8_t  DRIVE_STOP_REQUEST             = 1; // final motor stop request
 volatile uint32_t LAST_SAFETY_RX_MS             = 0;
 volatile uint32_t SAFETY_RX_COUNT                = 0;
 volatile uint32_t SAFETY_CRC_ERROR_COUNT         = 0;
 volatile uint32_t SAFETY_SEQUENCE_ERROR_COUNT    = 0;
 
 volatile uint8_t GEAR            = 0;      // 0 : 전진 || 1 : 후진 / write : Inputs / read : Control, uROS
 volatile uint8_t MODE            = 0;      // 0 : 수동 || 1 : 자율 / write : Inputs / read : Control, uROS
 volatile uint8_t PEDAL           = 0;      // 페달(스르틀, Analog) 0~100(%) / write : Inputs / read : Control
