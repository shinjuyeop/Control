/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "stm32g4xx_nucleo.h"

void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_IN_Pin       GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin       GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin          GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port    GPIOF
#define RCC_OSC_OUT_Pin         GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port   GPIOF

/* 스위치 (PA8: 자율/수동, PA9: 전진/후진) */
#define SW_MODE_Pin         GPIO_PIN_8
#define SW_MODE_GPIO_Port   GPIOA
#define SW_GEAR_Pin         GPIO_PIN_9
#define SW_GEAR_GPIO_Port   GPIOA

/* EStop 물리버튼 (PB0: 계기판, PB1: 측면) — USER CODE 추가 */
#define ESTOP_DASH_Pin          GPIO_PIN_0
#define ESTOP_DASH_GPIO_Port    GPIOB
#define ESTOP_DASH_EXTI_IRQn    EXTI0_IRQn
#define ESTOP_SIDE_Pin          GPIO_PIN_1
#define ESTOP_SIDE_GPIO_Port    GPIOB
#define ESTOP_SIDE_EXTI_IRQn    EXTI1_IRQn

#define T_SWDIO_Pin         GPIO_PIN_13
#define T_SWDIO_GPIO_Port   GPIOA
#define T_SWCLK_Pin         GPIO_PIN_14
#define T_SWCLK_GPIO_Port   GPIOA
#define T_SWO_Pin           GPIO_PIN_3
#define T_SWO_GPIO_Port     GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */