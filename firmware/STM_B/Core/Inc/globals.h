/* globals.h — STM_B 전역변수 선언
 * 위치: STM_B/Core/Inc/globals.h  (신규 생성)
 */
 #pragma once
 #include <stdint.h>
 
 /* STM_A Vehicle Status UART 수신 상태 */
 extern volatile float    CURRENT_SPEED_FROM_A;
 extern volatile uint8_t  GEAR_FROM_A;
 extern volatile uint8_t  MODE_FROM_A;
 extern volatile uint8_t  VEHICLE_FRAME_VALID_FROM_A;
 extern volatile uint32_t LAST_VEHICLE_RX_MS;
 extern volatile uint32_t VEHICLE_RX_COUNT;
 extern volatile uint32_t VEHICLE_CRC_ERROR_COUNT;
 extern volatile uint32_t VEHICLE_SEQUENCE_ERROR_COUNT;
 
 /* 안전: EStop 태스크가 물리 입력과 AUTO 모드 Host AEB를 통합한다. */
 extern volatile uint8_t AEB;                   /* Host /aeb 입력 */
 extern volatile uint8_t ESTOP_DASHBOARD;       /* PC0, active-low */
 extern volatile uint8_t ESTOP_SIDE;            /* PC1, active-low */
 extern volatile uint8_t ESTOP_RF;              /* PB4, active-low */
 extern volatile uint8_t ESTOP_STATUS;          /* 물리 E-stop 통합 */
 extern volatile uint8_t MICROROS_COMM_FAULT;    /* Agent/entity 통신 이상 */
 extern volatile uint8_t BRAKE_REQUEST;         /* 브레이크 적용 요청 */
 extern volatile uint8_t BRAKE_COMMAND_APPLIED; /* 적용 방향 PWM 명령 상태 */
