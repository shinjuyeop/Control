/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : STM_B FreeRTOS 태스크 전체 구현
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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

/* USER CODE BEGIN Variables */
static volatile uint8_t s_rf_toggle = 0;
static uint8_t s_arx_buf[4];
/* USER CODE END Variables */

/* ── 태스크 속성 ── */
osThreadId_t BrakeHandle;
const osThreadAttr_t Brake_attributes = {
  .name = "Brake", .priority = (osPriority_t)osPriorityRealtime, .stack_size = 512*4 };
osThreadId_t EStopHandle;
const osThreadAttr_t EStop_attributes = {
  .name = "EStop", .priority = (osPriority_t)osPriorityHigh,     .stack_size = 256*4 };
osThreadId_t uROSHandle;
const osThreadAttr_t uROS_attributes = {
  .name = "uROS",  .priority = (osPriority_t)osPriorityNormal,   .stack_size = 1280*4 };
osThreadId_t VESSHandle;
const osThreadAttr_t VESS_attributes = {
  .name = "VESS",  .priority = (osPriority_t)osPriorityBelowNormal, .stack_size = 256*4 };
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display", .priority = (osPriority_t)osPriorityLow,    .stack_size = 512*4 };
osThreadId_t ARxHandle;
const osThreadAttr_t ARx_attributes = {
  .name = "ARx",   .priority = (osPriority_t)osPriorityAboveNormal, .stack_size = 256*4 };
osThreadId_t ATxHandle;
const osThreadAttr_t ATx_attributes = {
  .name = "ATx",   .priority = (osPriority_t)osPriorityHigh,     .stack_size = 256*4 };

void StartBrake(void *argument);   void StartEStop(void *argument);
void StartuROS(void *argument);    void StartVESS(void *argument);
void StartDisplay(void *argument); void StartARx(void *argument);
void StartATx(void *argument);

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  BrakeHandle   = osThreadNew(StartBrake,   NULL, &Brake_attributes);
  EStopHandle   = osThreadNew(StartEStop,   NULL, &EStop_attributes);
  uROSHandle    = osThreadNew(StartuROS,    NULL, &uROS_attributes);
  VESSHandle    = osThreadNew(StartVESS,    NULL, &VESS_attributes);
  DisplayHandle = osThreadNew(StartDisplay, NULL, &Display_attributes);
  ARxHandle     = osThreadNew(StartARx,     NULL, &ARx_attributes);
  ATxHandle     = osThreadNew(StartATx,     NULL, &ATx_attributes);
}

/* RF EXTI 콜백 (ISR) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* USER CODE BEGIN EXTI */
  /* Pinmap 기준 RF_Remote = PB4. main.h의 ESTOP_BTN_Pin을 PB4로 맞출 것 */
  if (GPIO_Pin == ESTOP_BTN_Pin)
    s_rf_toggle ^= 1u;
  /* USER CODE END EXTI */
}

/* EStop */
void StartEStop(void *argument) {
  /* USER CODE BEGIN StartEStop */
  for (;;) {
    ESTOP_RF     = s_rf_toggle;
    ESTOP_STATUS = ESTOP_RF | AEB | ESTOP_FROM_A;
    osDelay(10);
  }
  /* USER CODE END StartEStop */
}

/* Brake */
#define ANG_BRAKE    35
#define ANG_RELEASE  70
#define BRAKE_STEP_MS 20

void StartBrake(void *argument) {
  /* USER CODE BEGIN StartBrake */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  int cur = ANG_RELEASE, tgt = ANG_RELEASE;
  uint32_t last = 0;
  for (;;) {
    tgt = (ESTOP_STATUS == 1) ? ANG_BRAKE : ANG_RELEASE;
    uint32_t now = HAL_GetTick();
    if (now - last >= BRAKE_STEP_MS) {
      last = now;
      if      (cur < tgt) cur++;
      else if (cur > tgt) cur--;
      uint32_t pulse = 1000u + (uint32_t)((cur * 1000) / 180);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    }
    osDelay(5);
  }
  /* USER CODE END StartBrake */
}

/* ARx */
void StartARx(void *argument) {
  /* USER CODE BEGIN StartARx */
  extern UART_HandleTypeDef huart1;
  HAL_UART_Receive_DMA(&huart1, s_arx_buf, 4);
  for (;;) {
    if (s_arx_buf[0] == 0xAAu && s_arx_buf[1] == 0x01u) {
      uint8_t chk = s_arx_buf[0] ^ s_arx_buf[1] ^ s_arx_buf[2];
      if (chk == s_arx_buf[3]) ESTOP_FROM_A = s_arx_buf[2];
    }
    osDelay(10);
  }
  /* USER CODE END StartARx */
}

/* ATx */
void StartATx(void *argument) {
  /* USER CODE BEGIN StartATx */
  extern UART_HandleTypeDef huart1;
  for (;;) {
    uint8_t pkt[4];
    pkt[0] = 0xAAu; pkt[1] = 0x01u;
    pkt[2] = ESTOP_RF;
    pkt[3] = pkt[0] ^ pkt[1] ^ pkt[2];
    HAL_UART_Transmit(&huart1, pkt, 4, 10);
    osDelay(20);
  }
  /* USER CODE END StartATx */
}

/* uROS */
static std_msgs__msg__Float32 s_cmd_msg, s_cur_msg;
static std_msgs__msg__UInt8   s_aeb_msg, s_gear_msg, s_mode_msg;
static void cb_cmd (const void *m){COMMAND_SPEED=((const std_msgs__msg__Float32*)m)->data;}
static void cb_cur (const void *m){CURRENT_SPEED=((const std_msgs__msg__Float32*)m)->data;}
static void cb_aeb (const void *m){AEB =((const std_msgs__msg__UInt8*)m)->data;}
static void cb_gear(const void *m){GEAR=((const std_msgs__msg__UInt8*)m)->data;}
static void cb_mode(const void *m){MODE=((const std_msgs__msg__UInt8*)m)->data;}

void StartuROS(void *argument) {
  /* USER CODE BEGIN StartuROS */
  extern UART_HandleTypeDef hlpuart1;
  rmw_uros_set_custom_transport(true,(void*)&hlpuart1,
      cubemx_transport_open,cubemx_transport_close,
      cubemx_transport_write,cubemx_transport_read);

  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t sup; rcl_node_t node;
  rcl_subscription_t sub_cmd,sub_cur,sub_aeb,sub_gear,sub_mode;

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

  rclc_executor_t exec;
  rclc_executor_init(&exec,&sup.context,5,&alloc);
  rclc_executor_add_subscription(&exec,&sub_cmd, &s_cmd_msg, &cb_cmd, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_cur, &s_cur_msg, &cb_cur, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_aeb, &s_aeb_msg, &cb_aeb, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_gear,&s_gear_msg,&cb_gear,ON_NEW_DATA);
  rclc_executor_add_subscription(&exec,&sub_mode,&s_mode_msg,&cb_mode,ON_NEW_DATA);

  for(;;){ rclc_executor_spin_some(&exec,RCL_MS_TO_NS(10)); osDelay(10); }
  /* USER CODE END StartuROS */
}

/* VESS */
void StartVESS(void *argument) {
  /* USER CODE BEGIN StartVESS */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  for (;;) {
    float kmh = CURRENT_SPEED * 3.6f;
    if (kmh < 0.0f) kmh = 0.0f;
    if (kmh > 100.0f) kmh = 100.0f;
    uint32_t pulse = 1600u + (uint32_t)((kmh/100.0f)*400.0f);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    osDelay(100);
  }
  /* USER CODE END StartVESS */
}

/* Display — LCD 드라이버 연동 후 내부 구현 */
void StartDisplay(void *argument) {
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

/* USER CODE BEGIN Application */
/* USER CODE END Application */