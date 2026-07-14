/* globals.h — STM_A 전역변수 선언
 * 위치: STM_A/Core/Inc/globals.h  (신규 생성)
 */
 #pragma once
 #include <stdint.h>
 
 /* 속도 */
 extern volatile float   COMMAND_SPEED;   /* PC→STM  /cmd/speed   m/s */
 extern volatile float   CURRENT_SPEED;   /* 모터피드백 /current/speed m/s */
 
 /* 안전: 물리 E-stop은 STM_B Safety frame에서만 수신한다. */
 extern volatile uint8_t AEB;                          /* Host /aeb */
 extern volatile uint8_t ESTOP_DASHBOARD_FROM_B;       /* Safety bit0 */
 extern volatile uint8_t ESTOP_SIDE_FROM_B;            /* Safety bit1 */
 extern volatile uint8_t ESTOP_RF_FROM_B;              /* Safety bit2 */
 extern volatile uint8_t AEB_FROM_B;                   /* Safety bit3 */
 extern volatile uint8_t BRAKE_REQUEST_FROM_B;         /* Safety bit4 */
 extern volatile uint8_t BRAKE_COMMAND_APPLIED_FROM_B; /* Safety bit5 */
 extern volatile uint8_t DRIVE_STOP_REQUEST;            /* 최종 구동 정지 요청 */
 
 /* 주행 */
 extern volatile uint8_t GEAR;            /* 0:전진  1:후진            */
 extern volatile uint8_t MODE;            /* 0:수동  1:자율            */
 extern volatile uint8_t PEDAL;           /* 0~100 %                   */
