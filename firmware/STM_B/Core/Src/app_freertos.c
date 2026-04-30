/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Brake */
osThreadId_t BrakeHandle;
const osThreadAttr_t Brake_attributes = {
  .name = "Brake",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for EStop */
osThreadId_t EStopHandle;
const osThreadAttr_t EStop_attributes = {
  .name = "EStop",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for uROS */
osThreadId_t uROSHandle;
const osThreadAttr_t uROS_attributes = {
  .name = "uROS",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1280 * 4
};
/* Definitions for VESS */
osThreadId_t VESSHandle;
const osThreadAttr_t VESS_attributes = {
  .name = "VESS",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for ARx */
osThreadId_t ARxHandle;
const osThreadAttr_t ARx_attributes = {
  .name = "ARx",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for ATx */
osThreadId_t ATxHandle;
const osThreadAttr_t ATx_attributes = {
  .name = "ATx",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBrake(void *argument);
void StartEStop(void *argument);
void StartuROS(void *argument);
void StartVESS(void *argument);
void StartDisplay(void *argument);
void StartARx(void *argument);
void StartATx(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Brake */
  BrakeHandle = osThreadNew(StartBrake, NULL, &Brake_attributes);

  /* creation of EStop */
  EStopHandle = osThreadNew(StartEStop, NULL, &EStop_attributes);

  /* creation of uROS */
  uROSHandle = osThreadNew(StartuROS, NULL, &uROS_attributes);

  /* creation of VESS */
  VESSHandle = osThreadNew(StartVESS, NULL, &VESS_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(StartDisplay, NULL, &Display_attributes);

  /* creation of ARx */
  ARxHandle = osThreadNew(StartARx, NULL, &ARx_attributes);

  /* creation of ATx */
  ATxHandle = osThreadNew(StartATx, NULL, &ATx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBrake */
/**
  * @brief  Function implementing the Brake thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBrake */
void StartBrake(void *argument)
{
  /* USER CODE BEGIN StartBrake */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBrake */
}

/* USER CODE BEGIN Header_StartEStop */
/**
* @brief Function implementing the EStop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEStop */
void StartEStop(void *argument)
{
  /* USER CODE BEGIN StartEStop */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartEStop */
}

/* USER CODE BEGIN Header_StartuROS */
/**
* @brief Function implementing the uROS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartuROS */
void StartuROS(void *argument)
{
  /* USER CODE BEGIN StartuROS */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartuROS */
}

/* USER CODE BEGIN Header_StartVESS */
/**
* @brief Function implementing the VESS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVESS */
void StartVESS(void *argument)
{
  /* USER CODE BEGIN StartVESS */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartVESS */
}

/* USER CODE BEGIN Header_StartDisplay */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplay */
void StartDisplay(void *argument)
{
  /* USER CODE BEGIN StartDisplay */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDisplay */
}

/* USER CODE BEGIN Header_StartARx */
/**
* @brief Function implementing the ARx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartARx */
void StartARx(void *argument)
{
  /* USER CODE BEGIN StartARx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartARx */
}

/* USER CODE BEGIN Header_StartATx */
/**
* @brief Function implementing the ATx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartATx */
void StartATx(void *argument)
{
  /* USER CODE BEGIN StartATx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartATx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

