#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DISPLAY_UPDATE_MS 50U
#define DISPLAY_RECOVERY_RETRY_MS 1000U
#define DISPLAY_GEAR_GESTURE_WINDOW_MS 1000U

/* 모터 상전류(phase_current_da, 0.1A 단위)의 방향이 반전되거나 한 프레임
 * 사이에 급격히 변하면, 그 di/dt가 커패시티브 커플링으로 그라운드 레벨을
 * 흔들어 ST7796S 컨트롤러 상태를 깨뜨린다(화면 하얘짐). STM_B에는 그라운드
 * 레벨을 직접 잴 아날로그 입력이 없으므로, 원인인 상전류 변화를 STM_A가
 * 보내는 detail 프레임에서 감지해 트리거로 쓴다. */

/* 부호 반전으로 인정하기 위한 양쪽 최소 크기. |I| >= 이 값(3.0A)이어야
 * 실제 방향 전환으로 보고, 그 사이 구간은 이전 부호를 유지한다(0 부근
 * 노이즈 오검출 방지 히스테리시스). */
#define DISPLAY_PHASE_REVERSAL_DA 30

/* 연속 프레임 간 |Δphase_current|가 이 값(40.0A)을 넘으면 부호 반전이
 * 아니어도 그라운드를 흔들 만한 급변으로 본다. */
#define DISPLAY_PHASE_SWING_DA 400

/* 마지막 교란 이후 이 시간이 지나면 재초기화한다. 교란이 이어지면 마지막
 * 이벤트 기준으로 대기 시간이 다시 시작된다(과도 구간 중 재초기화 방지). */
#define DISPLAY_PHASE_SETTLE_MS 250U

extern volatile uint32_t DISPLAY_RECOVERY_COUNT;
extern volatile uint32_t DISPLAY_LAST_RECOVERY_ERROR;

void Dashboard_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_APP_H */
