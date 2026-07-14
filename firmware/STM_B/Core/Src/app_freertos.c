/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : STM_B FreeRTOS 태스크 전체 구현
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
#include <math.h>
#include <stdio.h>
#include "globals.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>
#include "rmw_microros/rmw_microros.h"

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                               const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport,
                              uint8_t *buf, size_t len,
                              int timeout, uint8_t *err);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANG_BRAKE 35 // 제동 각도 [deg] - 실차 기구 검증 후 조정 필요 
#define ANG_RELEASE 70
#define BRAKE_STEP_MS 20
#define ESTOP_EVENT_DASH (1u << 0)
#define ESTOP_EVENT_SIDE (1u << 1)
#define ESTOP_EVENT_RF   (1u << 2)
#define ESTOP_EVENT_MASK (ESTOP_EVENT_DASH | ESTOP_EVENT_SIDE | ESTOP_EVENT_RF)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* micro-ROS 구독 메시지 버퍼 */
static std_msgs__msg__Float32 s_cmd_msg;
static std_msgs__msg__Float32 s_cur_msg;
static std_msgs__msg__UInt8   s_aeb_msg;
static std_msgs__msg__UInt8   s_gear_msg;
static std_msgs__msg__UInt8   s_mode_msg;

/* 구독 콜백: 수신값을 전역변수에 반영 */
static void cb_cmd (const void *m){ COMMAND_SPEED = ((const std_msgs__msg__Float32 *)m)->data; }
static void cb_cur (const void *m){ CURRENT_SPEED = ((const std_msgs__msg__Float32 *)m)->data; }
static void cb_aeb (const void *m){ AEB  = ((const std_msgs__msg__UInt8 *)m)->data; }
static void cb_gear(const void *m){ GEAR = ((const std_msgs__msg__UInt8 *)m)->data; }
static void cb_mode(const void *m){ MODE = ((const std_msgs__msg__UInt8 *)m)->data; }

static uint8_t read_active_low(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

static void update_safety_state(void)
{
  /* 세 물리 E-stop 입력은 pull-up active-low이다. */
  ESTOP_DASHBOARD = read_active_low(ESTOP_DASH_GPIO_Port, ESTOP_DASH_Pin);
  ESTOP_SIDE = read_active_low(ESTOP_SIDE_GPIO_Port, ESTOP_SIDE_Pin);
  ESTOP_RF = read_active_low(ESTOP_RF_GPIO_Port, ESTOP_RF_Pin);

  ESTOP_STATUS = ESTOP_DASHBOARD | ESTOP_SIDE | ESTOP_RF;
  SAFETY_STOP_REQUEST = ESTOP_STATUS | ((AEB != 0u) ? 1u : 0u);
  BRAKE_REQUEST = SAFETY_STOP_REQUEST;
}

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
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  int cur = ANG_RELEASE, tgt = ANG_RELEASE;
  uint32_t last = 0;
  for (;;) {
    uint8_t brake_requested = (BRAKE_REQUEST != 0u) ? 1u : 0u;
    BRAKE_COMMAND_APPLIED = brake_requested;
    tgt = (brake_requested != 0u) ? ANG_BRAKE : ANG_RELEASE;
    uint32_t now = HAL_GetTick();
    if (now - last >= BRAKE_STEP_MS) {
      last = now;
      if      (cur < tgt) cur++;
      else if (cur > tgt) cur--;
      uint32_t pulse = 1000u + (uint32_t)((cur * 1000) / 180);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
    }
    osDelay(5);
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
  /* 태스크 시작 시 인터럽트를 기다리지 않고 현재 핀 상태부터 반영한다. */
  update_safety_state();
  for (;;) {
    (void)osThreadFlagsWait(ESTOP_EVENT_MASK, osFlagsWaitAny, 10u);
    /* timeout 주기에도 다시 읽어 누락된 edge와 Host AEB 변경을 반영한다. */
    update_safety_state();
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
  extern UART_HandleTypeDef hlpuart1;
  rmw_uros_set_custom_transport(true,(void*)&hlpuart1,
      cubemx_transport_open,cubemx_transport_close,
      cubemx_transport_write,cubemx_transport_read);

  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t sup; rcl_node_t node;
  rcl_subscription_t sub_cmd,sub_cur,sub_aeb,sub_gear,sub_mode;
  rcl_publisher_t pub_estop, pub_rf, pub_safety_stop;

  while (rmw_uros_ping_agent(100,1) != RMW_RET_OK) osDelay(500);

  rclc_support_init(&sup,0,NULL,&alloc);
  rclc_node_init_default(&node,"stm_b_node","",&sup);

  rclc_subscription_init_default(&sub_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),"/cmd/speed");
  rclc_subscription_init_default(&sub_cur, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),"/current/speed");
  rclc_subscription_init_default(&sub_aeb, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt8),"/aeb");
  rclc_subscription_init_default(&sub_gear,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt8),"/gear");
  rclc_subscription_init_default(&sub_mode,&node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt8),"/mode");

  rclc_publisher_init_default(&pub_estop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt8),"/stm_b/estop/status");
  rclc_publisher_init_default(&pub_rf, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt8),"/stm_b/estop/rf");
  rclc_publisher_init_default(&pub_safety_stop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt8),"/stm_b/safety/stop_request");

  rclc_executor_t exec;
  rclc_executor_init(&exec,&sup.context,5,&alloc);
  rclc_executor_add_subscription(&exec,&sub_cmd, &s_cmd_msg, &cb_cmd, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_cur, &s_cur_msg, &cb_cur, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_aeb, &s_aeb_msg, &cb_aeb, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_gear,&s_gear_msg,&cb_gear,ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_mode,&s_mode_msg,&cb_mode,ON_NEW_DATA);

  for(;;){
    rclc_executor_spin_some(&exec,RCL_MS_TO_NS(10));

    std_msgs__msg__UInt8 u;
    u.data = ESTOP_STATUS; rcl_publish(&pub_estop, &u, NULL);
    u.data = ESTOP_RF;     rcl_publish(&pub_rf, &u, NULL);
    u.data = SAFETY_STOP_REQUEST; rcl_publish(&pub_safety_stop, &u, NULL);

    osDelay(10);
  }
  /* USER CODE END StartuROS */
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
  /* TODO: LCD_Init() 호출 후 아래 스니펫으로 채울 것
     char buf[20];
     snprintf(buf,sizeof(buf),"%.1f km/h",CURRENT_SPEED*3.6f);
     LCD_DrawString(10,10,buf,...);
     LCD_DrawString(10,40,(GEAR==1)?"REV":"FWD",...);
     LCD_DrawString(10,60,(MODE==1)?"AUTO":"MANUAL",...);
     if(ESTOP_STATUS) LCD_DrawString(10,90,"!! ESTOP !!",...);
     else             LCD_DrawString(10,90,"  NORMAL   ",...);
  */
  for (;;) { osDelay(100); }
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
  /* STM_A -> STM_B E-stop 송신은 제거되었다. 태스크 슬롯만 안전하게 유지한다. */
  for (;;) {
    osDelay(1000);
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
  extern UART_HandleTypeDef huart1;
  for (;;) {
    uint8_t pkt[4];
    uint8_t safety_bits = 0u;

    safety_bits |= (uint8_t)((ESTOP_DASHBOARD != 0u) ? (1u << 0) : 0u);
    safety_bits |= (uint8_t)((ESTOP_SIDE != 0u) ? (1u << 1) : 0u);
    safety_bits |= (uint8_t)((ESTOP_RF != 0u) ? (1u << 2) : 0u);
    safety_bits |= (uint8_t)((AEB != 0u) ? (1u << 3) : 0u);
    safety_bits |= (uint8_t)((BRAKE_REQUEST != 0u) ? (1u << 4) : 0u);
    safety_bits |= (uint8_t)((BRAKE_COMMAND_APPLIED != 0u) ? (1u << 5) : 0u);

    pkt[0] = 0xAAu;
    pkt[1] = 0x02u;
    pkt[2] = safety_bits;
    pkt[3] = pkt[0] ^ pkt[1] ^ pkt[2];
    HAL_UART_Transmit(&huart1, pkt, 4, 10);
    osDelay(20);
  }
  /* USER CODE END StartATx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t event_flag;

  if (GPIO_Pin == ESTOP_DASH_Pin) {
    event_flag = ESTOP_EVENT_DASH;
  } else if (GPIO_Pin == ESTOP_SIDE_Pin) {
    event_flag = ESTOP_EVENT_SIDE;
  } else if (GPIO_Pin == ESTOP_RF_Pin) {
    event_flag = ESTOP_EVENT_RF;
  } else {
    return;
  }

  /* ISR에서는 상태를 계산하지 않고 EStop 태스크만 깨운다. */
  if ((EStopHandle != NULL) &&
      (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
    (void)osThreadFlagsSet(EStopHandle, event_flag);
  }
}

/* USER CODE END Application */

