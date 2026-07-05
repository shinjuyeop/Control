/*
---- CODE DESCRIPTION ----
* 전역변수 초기값 선언
*/

 volatile float   CURRENT_SPEED = 0.0f; // 현재 속도(m/s) / write : uROS / read : VESS, Display
 volatile float   COMMAND_SPEED = 0.0f; // 목표 속도(m/s) / write : uROS / read : (미사용)
 
 volatile uint8_t AEB           = 0;    // AEB(판단) / write : uROS / read : EStop
 volatile uint8_t ESTOP_RF      = 0;    // RF 리모콘 토글 / write : EStop / read : EStop, ATx
 volatile uint8_t ESTOP_FROM_A  = 0;    // 물리버튼 ESTOP(STM A) / write : ARx / read : EStop
 volatile uint8_t ESTOP_STATUS  = 1;    // 통합 ESTOP / write : EStop / read : Brake, Display
 
 volatile uint8_t GEAR          = 0;    // 0 : 전진 || 1 : 후진 / write : uROS / read : Display
 volatile uint8_t MODE          = 0;    // 0 : 수동 || 1 : 자율 / write : uROS / read : Display