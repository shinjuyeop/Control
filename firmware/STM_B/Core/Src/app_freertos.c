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
#include "display_app.h"
#include "globals.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <kaiev26_msgs/msg/aeb_state.h>
#include <kaiev26_msgs/msg/brake_actuator_status.h>
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
typedef struct {
  uint8_t data[16];
  uint8_t index;
  uint8_t expected_size;
} uart_frame_parser_t;
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
#define UART_SOF1 0xAAu
#define UART_SOF2 0x55u
#define UART_MSG_A_TO_B_VEHICLE_STATUS 0x01u
#define UART_MSG_B_TO_A_SAFETY_STATUS  0x02u
#define UART_VEHICLE_PAYLOAD_LEN 3u
#define UART_SAFETY_PAYLOAD_LEN  1u
#define UART_RX_DMA_SIZE 64u
#define UART_RX_RING_SIZE 128u
#define VEHICLE_COMM_TIMEOUT_MS 200u
#define SAFETY_TX_PERIOD_MS 20u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCL_PUBLISH_BEST_EFFORT(pub, msg) do { \
  rcl_ret_t publish_result = rcl_publish((pub), (msg), NULL); \
  (void)publish_result; \
} while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* micro-ROS 구독 메시지 버퍼 */
/* micro-ROS 구독 메시지의 string 필드에는 정적 버퍼가 필요하다 */
static kaiev26_msgs__msg__AEBState s_aeb_msg;
static char s_aeb_frame_id[16];
static char s_aeb_state_str[32];
static char s_aeb_reason_str[64];

/* Host에서는 AEB만 받는다. Vehicle Status는 STM_A UART에서 수신한다. */
static void cb_aeb (const void *m){
  AEB = ((const kaiev26_msgs__msg__AEBState *)m)->active ? 1u : 0u;
}

static uint8_t s_usart1_rx_dma[UART_RX_DMA_SIZE];
static uint8_t s_usart1_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t s_usart1_rx_head = 0u;
static volatile uint16_t s_usart1_rx_tail = 0u;
static volatile uint8_t s_usart1_rx_restart_requested = 0u;
static uint16_t s_usart1_rx_dma_pos = 0u;
static uart_frame_parser_t s_vehicle_parser;
static uint8_t s_vehicle_last_sequence = 0u;
static uint8_t s_vehicle_sequence_initialized = 0u;

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
static uint8_t uart_crc8(const uint8_t *data, uint8_t length);
static void usart1_rx_ring_push(uint8_t byte);
static uint8_t usart1_rx_ring_pop(uint8_t *byte);
static void reset_vehicle_parser(uint8_t last_byte);
static void process_vehicle_rx_byte(uint8_t byte);
static HAL_StatusTypeDef restart_usart1_rx(UART_HandleTypeDef *huart);
/* USER CODE END FunctionPrototypes */

void StartBrake(void *argument);
void StartEStop(void *argument);
void StartuROS(void *argument);
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
  rcl_subscription_t sub_aeb;
  // 통합 /brake/status 하나가 개별 상태 토픽들을 대체한다
  rcl_publisher_t pub_brake_status;

  while (rmw_uros_ping_agent(100,1) != RMW_RET_OK) osDelay(500);

  rclc_support_init(&sup,0,NULL,&alloc);
  rclc_node_init_default(&node,"stm_b_node","",&sup);

  s_aeb_msg.header.frame_id.data = s_aeb_frame_id;
  s_aeb_msg.header.frame_id.size = 0u;
  s_aeb_msg.header.frame_id.capacity = sizeof(s_aeb_frame_id);
  s_aeb_msg.state.data = s_aeb_state_str;
  s_aeb_msg.state.size = 0u;
  s_aeb_msg.state.capacity = sizeof(s_aeb_state_str);
  s_aeb_msg.reason.data = s_aeb_reason_str;
  s_aeb_msg.reason.size = 0u;
  s_aeb_msg.reason.capacity = sizeof(s_aeb_reason_str);

  rclc_subscription_init_default(&sub_aeb, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs,msg,AEBState),
      "/safety/aeb_state");

  rclc_publisher_init_default(&pub_brake_status, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs,msg,BrakeActuatorStatus),
      "/brake/status");

  rclc_executor_t exec;
  rclc_executor_init(&exec,&sup.context,1,&alloc);
  rclc_executor_add_subscription(&exec,&sub_aeb, &s_aeb_msg, &cb_aeb, ON_NEW_DATA);

  static kaiev26_msgs__msg__BrakeActuatorStatus s_status_msg;
  static char s_status_frame_id[4];
  s_status_msg.header.frame_id.data = s_status_frame_id;
  s_status_msg.header.frame_id.size = 0u;
  s_status_msg.header.frame_id.capacity = sizeof(s_status_frame_id);

  uint32_t now = HAL_GetTick();
  uint32_t last_status = now - 50u;
  uint8_t previous_bits = 0xFFu;

  for(;;){
    rclc_executor_spin_some(&exec,RCL_MS_TO_NS(1));
    now = HAL_GetTick();

    uint8_t reason_bits = 0u;
    if (ESTOP_STATUS != 0u) {
      reason_bits |= kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_ESTOP;
    }
    if (AEB != 0u) {
      reason_bits |= kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_AEB;
    }
    if (((uint32_t)(now - last_status) >= 50u) || (reason_bits != previous_bits)) {
      s_status_msg.brake_commanded = (BRAKE_COMMAND_APPLIED != 0u);
      s_status_msg.stop_reason_bits = reason_bits;
      /* 브레이크 서보에 피드백 신호선이 없어 actual은 항구적 미지원 */
      s_status_msg.actual_valid = false;
      s_status_msg.command_age_ms = 0.0f;
      RCL_PUBLISH_BEST_EFFORT(&pub_brake_status, &s_status_msg);
      previous_bits = reason_bits;
      last_status = now;
    }

    osDelay(5);
  }
  /* USER CODE END StartuROS */
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
  extern UART_HandleTypeDef huart1;

  while (restart_usart1_rx(&huart1) != HAL_OK) {
    osDelay(10);
  }
  uint32_t last_restart_ms = HAL_GetTick();

  for (;;) {
    uint8_t byte;
    while (usart1_rx_ring_pop(&byte) != 0u) {
      process_vehicle_rx_byte(byte);
    }

    uint32_t now = HAL_GetTick();
    if ((VEHICLE_FRAME_VALID_FROM_A != 0u) &&
        ((uint32_t)(now - LAST_VEHICLE_RX_MS) >= VEHICLE_COMM_TIMEOUT_MS)) {
      VEHICLE_FRAME_VALID_FROM_A = 0u;
    }

    if ((s_usart1_rx_restart_requested != 0u) ||
        (((VEHICLE_FRAME_VALID_FROM_A == 0u) &&
          ((uint32_t)(now - last_restart_ms) >= VEHICLE_COMM_TIMEOUT_MS)))) {
      if (restart_usart1_rx(&huart1) == HAL_OK) {
        s_usart1_rx_restart_requested = 0u;
        last_restart_ms = now;
      }
    }
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
  extern UART_HandleTypeDef huart1;
  uint8_t sequence = 0u;

  for (;;) {
    uint8_t frame[7];
    uint8_t stop_bits = (SAFETY_STOP_REQUEST != 0u) ? 0x01u : 0x00u;

    frame[0] = UART_SOF1;
    frame[1] = UART_SOF2;
    frame[2] = UART_MSG_B_TO_A_SAFETY_STATUS;
    frame[3] = UART_SAFETY_PAYLOAD_LEN;
    frame[4] = sequence++;
    frame[5] = stop_bits;
    frame[6] = uart_crc8(&frame[2], 4u);

    (void)HAL_UART_Transmit(&huart1, frame, sizeof(frame), 10u);
    osDelay(SAFETY_TX_PERIOD_MS);
  }
  /* USER CODE END StartATx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static uint8_t uart_crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0x00u;
  for (uint8_t i = 0u; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
      crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                          : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void usart1_rx_ring_push(uint8_t byte)
{
  uint16_t next = (uint16_t)(s_usart1_rx_head + 1u);
  if (next >= UART_RX_RING_SIZE) next = 0u;
  if (next != s_usart1_rx_tail) {
    s_usart1_rx_ring[s_usart1_rx_head] = byte;
    s_usart1_rx_head = next;
  }
}

static uint8_t usart1_rx_ring_pop(uint8_t *byte)
{
  uint8_t available = 0u;
  taskENTER_CRITICAL();
  if (s_usart1_rx_tail != s_usart1_rx_head) {
    *byte = s_usart1_rx_ring[s_usart1_rx_tail];
    s_usart1_rx_tail++;
    if (s_usart1_rx_tail >= UART_RX_RING_SIZE) s_usart1_rx_tail = 0u;
    available = 1u;
  }
  taskEXIT_CRITICAL();
  return available;
}

static HAL_StatusTypeDef restart_usart1_rx(UART_HandleTypeDef *huart)
{
  (void)HAL_UART_AbortReceive(huart);
  taskENTER_CRITICAL();
  s_usart1_rx_head = 0u;
  s_usart1_rx_tail = 0u;
  s_usart1_rx_dma_pos = 0u;
  reset_vehicle_parser(0u);
  taskEXIT_CRITICAL();
  return HAL_UARTEx_ReceiveToIdle_DMA(huart, s_usart1_rx_dma,
                                     UART_RX_DMA_SIZE);
}

static void reset_vehicle_parser(uint8_t last_byte)
{
  s_vehicle_parser.index = 0u;
  s_vehicle_parser.expected_size = 0u;
  if (last_byte == UART_SOF1) {
    s_vehicle_parser.data[0] = UART_SOF1;
    s_vehicle_parser.index = 1u;
  }
}

static void process_vehicle_rx_byte(uint8_t byte)
{
  uart_frame_parser_t *parser = &s_vehicle_parser;

  if (parser->index == 0u) {
    if (byte == UART_SOF1) {
      parser->data[0] = byte;
      parser->index = 1u;
    }
    return;
  }

  if (parser->index == 1u) {
    if (byte == UART_SOF2) {
      parser->data[1] = byte;
      parser->index = 2u;
    } else {
      reset_vehicle_parser(byte);
    }
    return;
  }

  if (parser->index >= sizeof(parser->data)) {
    reset_vehicle_parser(byte);
    return;
  }
  parser->data[parser->index++] = byte;

  if (parser->index == 4u) {
    if ((parser->data[2] != UART_MSG_A_TO_B_VEHICLE_STATUS) ||
        (parser->data[3] != UART_VEHICLE_PAYLOAD_LEN)) {
      reset_vehicle_parser(byte);
      return;
    }
    parser->expected_size = (uint8_t)(6u + parser->data[3]);
  }

  if ((parser->expected_size != 0u) &&
      (parser->index == parser->expected_size)) {
    uint8_t payload_len = parser->data[3];
    uint8_t received_crc = parser->data[5u + payload_len];
    uint8_t calculated_crc = uart_crc8(&parser->data[2],
                                       (uint8_t)(3u + payload_len));

    if (received_crc != calculated_crc) {
      VEHICLE_CRC_ERROR_COUNT++;
    } else {
      uint8_t sequence = parser->data[4];
      if ((s_vehicle_sequence_initialized != 0u) &&
          (sequence != (uint8_t)(s_vehicle_last_sequence + 1u))) {
        VEHICLE_SEQUENCE_ERROR_COUNT++;
      }
      s_vehicle_last_sequence = sequence;
      s_vehicle_sequence_initialized = 1u;

      uint8_t status = parser->data[5];
      uint16_t speed_encoded = (uint16_t)parser->data[6] |
                               ((uint16_t)parser->data[7] << 8);
      int16_t speed_x100 = (int16_t)speed_encoded;

      taskENTER_CRITICAL();
      MODE_FROM_A = status & 0x01u;
      GEAR_FROM_A = (status >> 1) & 0x01u;
      CURRENT_SPEED_FROM_A = (float)speed_x100 / 100.0f;
      LAST_VEHICLE_RX_MS = HAL_GetTick();
      VEHICLE_FRAME_VALID_FROM_A = 1u;
      VEHICLE_RX_COUNT++;
      taskEXIT_CRITICAL();
    }
    reset_vehicle_parser(byte);
  }
}

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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART1) return;

  uint16_t position = Size;
  if (position > UART_RX_DMA_SIZE) position = UART_RX_DMA_SIZE;

  if (position > s_usart1_rx_dma_pos) {
    for (uint16_t i = s_usart1_rx_dma_pos; i < position; ++i) {
      usart1_rx_ring_push(s_usart1_rx_dma[i]);
    }
  } else if (position < s_usart1_rx_dma_pos) {
    for (uint16_t i = s_usart1_rx_dma_pos; i < UART_RX_DMA_SIZE; ++i) {
      usart1_rx_ring_push(s_usart1_rx_dma[i]);
    }
    for (uint16_t i = 0u; i < position; ++i) {
      usart1_rx_ring_push(s_usart1_rx_dma[i]);
    }
  }

  s_usart1_rx_dma_pos = position;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    s_usart1_rx_restart_requested = 1u;
  }
}

/* USER CODE END Application */

