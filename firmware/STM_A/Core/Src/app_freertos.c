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
#include "usart.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include "rmw_microros/rmw_microros.h"

// dma_transport.c 함수 전방 선언 (STM_B 방식과 동일)
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);/* USER CODE END Includes */

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
volatile uint8_t g_estop_from_b = 0;   // BRx가 쓰고, uROS가 읽음
static uint8_t a_rx_buf[4];
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
  extern UART_HandleTypeDef hlpuart1;

  // STM_B와 동일한 transport 설정 (LPUART1 → PC로 agent 연결)
  rmw_uros_set_custom_transport(
      true, (void *)&hlpuart1,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read
  );

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rcl_publisher_t publisher;
  std_msgs__msg__Bool msg;

  // Agent 연결 대기
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      osDelay(500);
  }

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "stmA_node", "", &support);
  rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "stma/estop_status"
  );

  for(;;)
  {
    msg.data = (g_estop_from_b != 0);
    rcl_publish(&publisher, &msg, NULL);
    osDelay(100);  // 10Hz publish
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
  extern UART_HandleTypeDef huart1;

  // DMA Circular 수신 시작 (한 번만)
  HAL_UART_Receive_DMA(&huart1, a_rx_buf, 4);

  for(;;)
  {
    if(a_rx_buf[0] == 0xAA && a_rx_buf[1] == 0x01)
    {
      uint8_t chk = a_rx_buf[0] ^ a_rx_buf[1] ^ a_rx_buf[2];
      if(chk == a_rx_buf[3])
      {
        g_estop_from_b = a_rx_buf[2];
      }
    }
    osDelay(10);
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

