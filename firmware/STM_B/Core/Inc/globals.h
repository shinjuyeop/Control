/* globals.h — STM_B 전역변수 선언
 * 위치: STM_B/Core/Inc/globals.h  (신규 생성)
 */
 #pragma once
 #include <stdint.h>
 
 /* 속도 (STM_A uROS에서 수신) */
 extern volatile float   CURRENT_SPEED;   /* /current/speed m/s */
 extern volatile float   COMMAND_SPEED;   /* /cmd/speed     m/s */
 
 /* 안전 */
 extern volatile uint8_t AEB;             /* /aeb                      */
 extern volatile uint8_t ESTOP_RF;        /* RF 리모콘 토글 상태        */
 extern volatile uint8_t ESTOP_FROM_A;    /* A로부터 수신 (물리버튼2개) */
 extern volatile uint8_t ESTOP_STATUS;    /* 최종 통합                  */
 
 /* 주행 상태 (uROS 수신) */
 extern volatile uint8_t GEAR;            /* 0:전진 1:후진 */
 extern volatile uint8_t MODE;            /* 0:수동 1:자율 */