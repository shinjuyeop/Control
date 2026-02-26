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
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for Inputs */
osThreadId_t InputsHandle;
const osThreadAttr_t Inputs_attributes = {
  .name = "Inputs",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 256 * 4
};
/* Definitions for MotorRx */
osThreadId_t MotorRxHandle;
const osThreadAttr_t MotorRx_attributes = {
  .name = "MotorRx",
  .priority = (osPriority_t) osPriorityHigh2,
  .stack_size = 256 * 4
};
/* Definitions for uROS */
osThreadId_t uROSHandle;
const osThreadAttr_t uROS_attributes = {
  .name = "uROS",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1536 * 4
};
/* Definitions for BRx */
osThreadId_t BRxHandle;
const osThreadAttr_t BRx_attributes = {
  .name = "BRx",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for BTx */
osThreadId_t BTxHandle;
const osThreadAttr_t BTx_attributes = {
  .name = "BTx",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControl(void *argument);
void StartInputs(void *argument);
void StartMotorRx(void *argument);
void StartuROS(void *argument);
void StartBRx(void *argument);
void StartBTx(void *argument);

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
  /* creation of Control */
  ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);

  /* creation of Inputs */
  InputsHandle = osThreadNew(StartInputs, NULL, &Inputs_attributes);

  /* creation of MotorRx */
  MotorRxHandle = osThreadNew(StartMotorRx, NULL, &MotorRx_attributes);

  /* creation of uROS */
  uROSHandle = osThreadNew(StartuROS, NULL, &uROS_attributes);

  /* creation of BRx */
  BRxHandle = osThreadNew(StartBRx, NULL, &BRx_attributes);

  /* creation of BTx */
  BTxHandle = osThreadNew(StartBTx, NULL, &BTx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControl */
/**
  * @brief  Function implementing the Control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControl */
void StartControl(void *argument)
{
  /* USER CODE BEGIN StartControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartControl */
}

/* USER CODE BEGIN Header_StartInputs */
/**
* @brief Function implementing the Inputs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputs */
void StartInputs(void *argument)
{
  /* USER CODE BEGIN StartInputs */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInputs */
}

/* USER CODE BEGIN Header_StartMotorRx */
/**
* @brief Function implementing the MotorRx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorRx */
void StartMotorRx(void *argument)
{
  /* USER CODE BEGIN StartMotorRx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorRx */
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

/* USER CODE BEGIN Header_StartBRx */
/**
* @brief Function implementing the BRx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBRx */
void StartBRx(void *argument)
{
  /* USER CODE BEGIN StartBRx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBRx */
}

/* USER CODE BEGIN Header_StartBTx */
/**
* @brief Function implementing the BTx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBTx */
void StartBTx(void *argument)
{
  /* USER CODE BEGIN StartBTx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBTx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

