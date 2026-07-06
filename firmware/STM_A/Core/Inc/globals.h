/* globals.h — STM_A 전역변수 선언
 * 위치: STM_A/Core/Inc/globals.h  (신규 생성)
 */
 #pragma once
 #include <stdint.h>
 
 /* 속도 */
 extern volatile float   COMMAND_SPEED;   /* PC→STM  /cmd/speed   m/s */
 extern volatile float   CURRENT_SPEED;   /* 모터피드백 /current/speed m/s */
 
 /* 안전 */
 extern volatile uint8_t AEB;             /* PC→STM  /aeb              */
 extern volatile uint8_t ESTOP_STATUS;    /* 최종 통합 ESTOP           */
 extern volatile uint8_t ESTOP_DASHBOARD; /* 계기판 버튼 (PB0)         */
 extern volatile uint8_t ESTOP_SIDE;      /* 측면 버튼   (PB1)         */
 extern volatile uint8_t ESTOP_FROM_B;    /* B로부터 수신 (RF)         */
 extern volatile uint8_t ESTOP_TO_B;      /* B로 송신 (물리버튼 2개)   */
 
 /* 주행 */
 extern volatile uint8_t GEAR;            /* 0:전진  1:후진            */
 extern volatile uint8_t MODE;            /* 0:수동  1:자율            */
 extern volatile uint8_t PEDAL;           /* 0~100 %                   */