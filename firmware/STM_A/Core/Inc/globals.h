/* globals.h — STM_A 전역변수 선언
 * 위치: STM_A/Core/Inc/globals.h  (신규 생성)
 */
 #pragma once
 #include <stdint.h>
 
 /* 속도 */
 extern volatile float   COMMAND_SPEED;   /* PC→STM  /cmd/speed   m/s */
 extern volatile float   CURRENT_SPEED;   /* 모터피드백 /current/speed m/s */
 
 /* 안전: STM_B는 통합 BRAKE_REQUEST만 전달한다. */
 extern volatile uint8_t  BRAKE_REQUEST_FROM_B;         /* Safety payload bit0 */
 extern volatile uint8_t  SAFETY_FRAME_VALID_FROM_B;    /* 정상 frame + timeout 상태 */
 extern volatile uint8_t  STM_B_COMM_TIMEOUT;           /* 100 ms Safety timeout */
 extern volatile uint8_t  DRIVE_STOP_REQUEST;            /* 최종 구동 정지 요청 */
 extern volatile uint32_t LAST_SAFETY_RX_MS;
 extern volatile uint32_t SAFETY_RX_COUNT;
 extern volatile uint32_t SAFETY_CRC_ERROR_COUNT;
 extern volatile uint32_t SAFETY_SEQUENCE_ERROR_COUNT;
 
 /* 주행 */
 extern volatile uint8_t GEAR;            /* 0:전진  1:후진            */
 extern volatile uint8_t MODE;            /* 0:수동  1:자율            */
 extern volatile uint8_t PEDAL;           /* 0~100 %                   */
