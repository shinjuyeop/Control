/* globals.h — STM_B 전역변수 선언
 * 위치: STM_B/Core/Inc/globals.h  (신규 생성)
 */
 #pragma once
 #include <stdint.h>
 
 /* 속도 (STM_A uROS에서 수신) */
 extern volatile float   CURRENT_SPEED;   /* /current/speed m/s */
 extern volatile float   COMMAND_SPEED;   /* /cmd/speed     m/s */
 
 /* 안전: EStop 태스크가 물리 입력과 Host AEB를 통합한다. */
 extern volatile uint8_t AEB;                   /* Host /aeb 입력 */
 extern volatile uint8_t ESTOP_DASHBOARD;       /* PC0, active-low */
 extern volatile uint8_t ESTOP_SIDE;            /* PC1, active-low */
 extern volatile uint8_t ESTOP_RF;              /* PB4, active-low */
 extern volatile uint8_t ESTOP_STATUS;          /* 물리 E-stop 통합 */
 extern volatile uint8_t SAFETY_STOP_REQUEST;   /* 물리 E-stop | AEB */
 extern volatile uint8_t BRAKE_REQUEST;         /* 브레이크 적용 요청 */
 extern volatile uint8_t BRAKE_COMMAND_APPLIED; /* 적용 방향 PWM 명령 상태 */
 
 /* 주행 상태 (uROS 수신) */
 extern volatile uint8_t GEAR;            /* 0:전진 1:후진 */
 extern volatile uint8_t MODE;            /* 0:수동 1:자율 */
