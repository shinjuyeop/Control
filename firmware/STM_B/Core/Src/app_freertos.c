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
#include <string.h>
#include "board_uart_protocol.h"
#include "runtime_periodic.h"
#include "runtime_stats_ros.h"
#include "dashboard_can.h"
#include "dashboard_can_protocol.h"
#include "globals.h"
#include "lv_battery_protocol.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <kaiev26_msgs/msg/actuator_command.h>
#include <kaiev26_msgs/msg/brake_actuator_status.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/u_int8_multi_array.h>
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
  uint8_t data[BOARD_UART_MAX_FRAME_SIZE];
  uint8_t index;
  uint8_t expected_size;
} uart_frame_parser_t;

typedef enum {
  ROS_STATE_WAIT_AGENT = 0,
  ROS_STATE_CREATE_ENTITIES,
  ROS_STATE_RUNNING,
  ROS_STATE_DESTROY_ENTITIES
} ros_connection_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Happymodel Super400 400g 서보 데이터시트 PWM 안전범위 (하드웨어 절대 한계) */
#define BRAKE_SERVO_PWM_MIN_US  800u
#define BRAKE_SERVO_PWM_MAX_US  2200u

/* 실차 캘리브레이션 직선: (45도,1249us) / (80도,1443us) 두 점에서 역산.
 * 기울기 194/35 us/deg, 절편(0도) 1000us로 두 점 모두 정확히 재현됨.
 * 실측 캘리브레이션 점이 바뀌면 이 두 값만 갱신하면 된다. */
#define BRAKE_SERVO_US_PER_DEG        (194.0f / 35.0f)
#define BRAKE_SERVO_ZERO_DEG_PULSE_US 1000.0f

/* 각도(도) -> pulse(us) 자동 변환. 각도만 바꾸면 pulse는 자동 계산된다. */
#define BRAKE_SERVO_DEG_TO_PULSE_US(deg_) \
  ((uint16_t)(BRAKE_SERVO_ZERO_DEG_PULSE_US + (BRAKE_SERVO_US_PER_DEG) * (deg_) + 0.5f))

/* ---- 여기 두 각도만 바꾸면 engage/release pulse가 자동으로 재계산된다 ---- */
#define BRAKE_ENGAGED_ANGLE_DEG   55.0f
#define BRAKE_RELEASED_ANGLE_DEG  120.0f

#define BRAKE_ENGAGED_PULSE_US   BRAKE_SERVO_DEG_TO_PULSE_US(BRAKE_ENGAGED_ANGLE_DEG)
#define BRAKE_RELEASED_PULSE_US  BRAKE_SERVO_DEG_TO_PULSE_US(BRAKE_RELEASED_ANGLE_DEG)

/* 서보 데이터시트 PWM 절대범위(800~2200us)를 벗어나면 컴파일 자체가 실패하도록 강제 */
_Static_assert(BRAKE_ENGAGED_PULSE_US >= BRAKE_SERVO_PWM_MIN_US &&
  BRAKE_ENGAGED_PULSE_US <= BRAKE_SERVO_PWM_MAX_US,
  "BRAKE_ENGAGED_ANGLE_DEG가 서보 PWM 안전범위(800~2200us)를 벗어남");
_Static_assert(BRAKE_RELEASED_PULSE_US >= BRAKE_SERVO_PWM_MIN_US &&
  BRAKE_RELEASED_PULSE_US <= BRAKE_SERVO_PWM_MAX_US,
  "BRAKE_RELEASED_ANGLE_DEG가 서보 PWM 안전범위(800~2200us)를 벗어남");

#define BRAKE_PULSE_TRAVEL_US \
  (BRAKE_RELEASED_PULSE_US - BRAKE_ENGAGED_PULSE_US)
#define BRAKE_RAMP_UPDATE_COUNT 35u
#define BRAKE_STEP_MS 20u
#define ESTOP_EVENT_DASH (1u << 0)
#define ESTOP_EVENT_SIDE (1u << 1)
#define ESTOP_EVENT_RF   (1u << 2)
#define ESTOP_EVENT_MASK (ESTOP_EVENT_DASH | ESTOP_EVENT_SIDE | ESTOP_EVENT_RF)
#define UART_RX_DMA_SIZE 64u
#define UART_RX_RING_SIZE 128u
#define VEHICLE_COMM_TIMEOUT_MS 200u
#define DRIVE_DETAIL_COMM_TIMEOUT_MS 300u
#define SAFETY_TX_PERIOD_MS 20u
#define VEHICLE_MODE_AUTO 1u
#define AGENT_PING_PERIOD_MS 500u
#define AGENT_PING_TIMEOUT_MS 100u
#define BRAKE_STATUS_PERIOD_MS 50u
#define ROS_OPERATION_FAILURE_LIMIT 3u
#define BATTERY_UART_RX_RING_SIZE 256u
#define BATTERY_STATUS_TIMEOUT_MS 1000u
#define BATTERY_STATUS_PERIOD_MS 100u
#define BATTERY_COMMAND_TX_TIMEOUT_MS 20u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* micro-ROS 구독 메시지 버퍼 */
/* micro-ROS 구독 메시지의 string 필드에는 정적 버퍼가 필요하다 */
static kaiev26_msgs__msg__ActuatorCommand s_cmd_msg;
static char s_cmd_frame_id[16];
static sensor_msgs__msg__NavSatFix s_gnss_msg;
static char s_gnss_frame_id[16];
static std_msgs__msg__UInt8MultiArray s_dashboard_auto_msg;
static uint8_t s_dashboard_auto_data[DASHBOARD_AUTO_ROS_PAYLOAD_SIZE];
static std_msgs__msg__UInt8MultiArray s_battery_command_msg;
static uint8_t s_battery_command_data[LV_BATTERY_COMMAND_PAYLOAD_SIZE];
static std_msgs__msg__UInt8MultiArray s_battery_link_status_msg;
static uint8_t s_battery_link_status_data[LV_BATTERY_STATUS_PAYLOAD_SIZE];
static std_msgs__msg__UInt8MultiArray s_battery_link_ack_msg;
static uint8_t s_battery_link_ack_data[LV_BATTERY_ACK_PAYLOAD_SIZE];
static kaiev26_msgs__msg__BrakeActuatorStatus s_status_msg;
static char s_status_frame_id[4];
static sensor_msgs__msg__BatteryState s_battery_msg;
static char s_battery_frame_id[16];
static char s_battery_location[16];
static char s_battery_serial[1];
static volatile uint32_t s_last_command_tick = 0u;
static volatile uint8_t s_command_received = 0u;

/* entity별 생성 상태를 추적해 부분 초기화 실패와 반복 reconnect를 정리한다. */
static RuntimeStatsRos s_runtime_ros;
static rcl_allocator_t s_ros_allocator;
static rclc_support_t s_ros_support;
static rcl_node_t s_ros_node;
static rcl_subscription_t s_ros_sub_cmd;
static rcl_subscription_t s_ros_sub_gnss;
static rcl_subscription_t s_ros_sub_dashboard_auto;
static rcl_subscription_t s_ros_sub_battery_command;
static rcl_publisher_t s_ros_pub_brake_status;
static rcl_publisher_t s_ros_pub_battery_state;
static rcl_publisher_t s_ros_pub_battery_link_status;
static rcl_publisher_t s_ros_pub_battery_link_ack;
static rclc_executor_t s_ros_executor;
static uint8_t s_ros_support_initialized = 0u;
static uint8_t s_ros_node_initialized = 0u;
static uint8_t s_ros_sub_cmd_initialized = 0u;
static uint8_t s_ros_sub_gnss_initialized = 0u;
static uint8_t s_ros_sub_dashboard_auto_initialized = 0u;
static uint8_t s_ros_sub_battery_command_initialized = 0u;
static uint8_t s_ros_publisher_initialized = 0u;
static uint8_t s_ros_battery_publisher_initialized = 0u;
static uint8_t s_ros_battery_link_status_initialized = 0u;
static uint8_t s_ros_battery_link_ack_initialized = 0u;
static uint8_t s_ros_executor_initialized = 0u;

/* Host에서는 최종 브레이크 명령만 받는다. Vehicle Status는 STM_A UART에서 수신한다. */
static void cb_cmd(const void *m)
{
  const uint32_t now = HAL_GetTick();
  taskENTER_CRITICAL();
  COMMAND_BRAKE_REQUEST =
      ((const kaiev26_msgs__msg__ActuatorCommand *)m)->brake_engage ? 1u : 0u;
  s_last_command_tick = now;
  s_command_received = 1u;
  taskEXIT_CRITICAL();
}

static void cb_gnss(const void *m)
{
  const sensor_msgs__msg__NavSatFix *fix =
      (const sensor_msgs__msg__NavSatFix *)m;
  DashboardCan_ObserveGnssFix(fix->status.status);
}

static void cb_dashboard_auto(const void *m)
{
  const std_msgs__msg__UInt8MultiArray *summary =
      (const std_msgs__msg__UInt8MultiArray *)m;
  DashboardCan_ObserveAutoStatus(summary->data.data, summary->data.size);
}

static volatile uint32_t s_battery_command_tx_count = 0u;
static volatile uint32_t s_battery_command_tx_error_count = 0u;
static uint32_t s_battery_command_frame_sequence = 0u;

static void cb_battery_command(const void *m)
{
  const std_msgs__msg__UInt8MultiArray *message =
      (const std_msgs__msg__UInt8MultiArray *)m;
  LvBatteryCommand command;
  if (message->data.size != LV_BATTERY_COMMAND_PAYLOAD_SIZE ||
      !LvBattery_DecodeCommandPayload(message->data.data, &command)) {
    s_battery_command_tx_error_count++;
    return;
  }
  uint8_t frame[LV_BATTERY_MAX_FRAME_SIZE];
  const size_t frame_size = LvBattery_EncodeCommandFrame(
      &command, s_battery_command_frame_sequence++, frame);
  if (frame_size == 0u ||
      HAL_UART_Transmit(&huart4, frame, (uint16_t)frame_size,
                        BATTERY_COMMAND_TX_TIMEOUT_MS) != HAL_OK) {
    s_battery_command_tx_error_count++;
    return;
  }
  s_battery_command_tx_count++;
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
static uint8_t s_detail_last_sequence = 0u;
static uint8_t s_detail_sequence_initialized = 0u;

static uint8_t s_uart4_rx_byte;
static uint8_t s_uart4_rx_ring[BATTERY_UART_RX_RING_SIZE];
static volatile uint16_t s_uart4_rx_head = 0u;
static volatile uint16_t s_uart4_rx_tail = 0u;
static volatile uint8_t s_uart4_rx_restart_requested = 0u;
static volatile uint32_t s_uart4_rx_overflow_count = 0u;
static LvBatteryParser s_battery_parser;
static LvBatteryStatus s_battery_status;
static volatile uint32_t s_last_battery_rx_tick = 0u;
static volatile uint32_t s_battery_rx_count = 0u;
static volatile uint32_t s_battery_crc_error_count = 0u;
static volatile uint32_t s_battery_format_error_count = 0u;
static volatile uint32_t s_battery_sequence_error_count = 0u;
static uint32_t s_battery_last_sequence = 0u;
static uint8_t s_battery_sequence_initialized = 0u;
static volatile uint8_t s_battery_status_received = 0u;
static LvBatteryAck s_battery_ack;
static volatile uint32_t s_battery_ack_count = 0u;

static uint8_t read_active_low(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

static void update_safety_state(void)
{
  uint8_t command_brake;
  uint8_t mode;

  /* 세 물리 E-stop 입력은 pull-up active-low이다. */
  ESTOP_DASHBOARD = read_active_low(ESTOP_DASH_GPIO_Port, ESTOP_DASH_Pin);
  ESTOP_SIDE = read_active_low(ESTOP_SIDE_GPIO_Port, ESTOP_SIDE_Pin);
  ESTOP_RF = read_active_low(ESTOP_RF_GPIO_Port, ESTOP_RF_Pin);

  /* 서로 다른 태스크가 쓰는 안전 입력을 한 시점의 값으로 읽는다. */
  taskENTER_CRITICAL();
  command_brake = COMMAND_BRAKE_REQUEST;
  mode = MODE_FROM_A;
  taskEXIT_CRITICAL();

  ESTOP_STATUS = ESTOP_DASHBOARD | ESTOP_SIDE | ESTOP_RF;
  BRAKE_REQUEST =
      ESTOP_STATUS |
      (((mode == VEHICLE_MODE_AUTO) && (command_brake != 0u)) ? 1u : 0u);
}

/* USER CODE END Variables */
/* Definitions for Brake */
osThreadId_t BrakeHandle;
const osThreadAttr_t Brake_attributes = {
  .name = "Brake",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 512 * 4
};
/* Definitions for EStop */
osThreadId_t EStopHandle;
const osThreadAttr_t EStop_attributes = {
  .name = "EStop",
  .priority = (osPriority_t) osPriorityRealtime,
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
/* Definitions for PerfMon */
osThreadId_t PerfMonHandle;
const osThreadAttr_t PerfMon_attributes = {
  .name = "PerfMon",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void usart1_rx_ring_push(uint8_t byte);
static uint8_t usart1_rx_ring_pop(uint8_t *byte);
static void reset_vehicle_parser(uint8_t last_byte);
static void process_vehicle_rx_byte(uint8_t byte);
static HAL_StatusTypeDef restart_usart1_rx(UART_HandleTypeDef *huart);
static void uart4_rx_ring_push(uint8_t byte);
static uint8_t uart4_rx_ring_pop(uint8_t *byte);
static HAL_StatusTypeDef restart_uart4_rx(UART_HandleTypeDef *huart);
static void process_battery_rx_byte(uint8_t byte);
static void init_ros_message_buffers(void);
static uint8_t create_ros_entities(void);
static void destroy_ros_entities(void);
static rcl_ret_t publish_brake_status(uint8_t reason_bits);
static rcl_ret_t publish_battery_state(void);
static rcl_ret_t publish_battery_link_status(void);
static rcl_ret_t publish_battery_link_ack(void);
static uint8_t ros_operation_failed(rcl_ret_t result);
/* USER CODE END FunctionPrototypes */

void StartBrake(void *argument);
void StartEStop(void *argument);
void StartuROS(void *argument);
void StartDisplay(void *argument);
void StartARx(void *argument);
void StartATx(void *argument);
void StartPerfMon(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  RuntimeMetrics_Init();
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

  /* creation of PerfMon */
  PerfMonHandle = osThreadNew(StartPerfMon, NULL, &PerfMon_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  RuntimeMetrics_Register(RM_BRAKE, (TaskHandle_t)BrakeHandle);
  RuntimeMetrics_Register(RM_ESTOP, (TaskHandle_t)EStopHandle);
  RuntimeMetrics_Register(RM_UROS, (TaskHandle_t)uROSHandle);
  RuntimeMetrics_Register(RM_DISPLAY, (TaskHandle_t)DisplayHandle);
  RuntimeMetrics_Register(RM_RX, (TaskHandle_t)ARxHandle);
  RuntimeMetrics_Register(RM_TX, (TaskHandle_t)ATxHandle);
  RuntimeMetrics_Register(RM_PERFMON, (TaskHandle_t)PerfMonHandle);

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
  uint16_t current_pulse_us = BRAKE_ENGAGED_PULSE_US;
  uint16_t target_pulse_us = BRAKE_ENGAGED_PULSE_US;
  uint16_t ramp_remainder = 0u;
  uint32_t last = HAL_GetTick();

  (void)argument;
  BRAKE_COMMAND_APPLIED = 1u;
  __HAL_TIM_SET_COMPARE(
      &htim2, TIM_CHANNEL_2, BRAKE_ENGAGED_PULSE_US);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  for (;;) {
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_BRAKE);
    uint8_t brake_requested = (BRAKE_REQUEST != 0u) ? 1u : 0u;
    BRAKE_COMMAND_APPLIED = brake_requested;
    uint16_t requested_pulse_us =
        (brake_requested != 0u) ?
        BRAKE_ENGAGED_PULSE_US : BRAKE_RELEASED_PULSE_US;

    if (requested_pulse_us != target_pulse_us) {
      target_pulse_us = requested_pulse_us;
      ramp_remainder = 0u;
    }

    uint32_t now = HAL_GetTick();
    if (now - last >= BRAKE_STEP_MS) {
      last = now;
      if (current_pulse_us != target_pulse_us) {
        uint16_t step_us;
        ramp_remainder =
            (uint16_t)(ramp_remainder + BRAKE_PULSE_TRAVEL_US);
        step_us = (uint16_t)(ramp_remainder / BRAKE_RAMP_UPDATE_COUNT);
        ramp_remainder =
            (uint16_t)(ramp_remainder % BRAKE_RAMP_UPDATE_COUNT);

        if (current_pulse_us < target_pulse_us) {
          uint16_t remaining =
              (uint16_t)(target_pulse_us - current_pulse_us);
          current_pulse_us += (step_us < remaining) ? step_us : remaining;
        } else {
          uint16_t remaining =
              (uint16_t)(current_pulse_us - target_pulse_us);
          current_pulse_us -= (step_us < remaining) ? step_us : remaining;
        }
        __HAL_TIM_SET_COMPARE(
            &htim2, TIM_CHANNEL_2, current_pulse_us);
      }
    }
    RuntimeMetrics_End(RM_BRAKE, activation);
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
    uint32_t dispatch_cycle = RuntimeMetrics_Now();
    RuntimeMetrics_EStopDispatch(dispatch_cycle);
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_ESTOP);
    /* timeout 주기에도 다시 읽어 누락된 edge와 명령 브레이크 변경을 반영한다. */
    update_safety_state();
    RuntimeMetrics_End(RM_ESTOP, activation);
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
  ros_connection_state_t state = ROS_STATE_WAIT_AGENT;
  uint32_t last_agent_ping;
  uint32_t last_status;
  uint32_t last_battery_status;
  uint8_t previous_bits = 0xFFu;
  uint8_t spin_failures = 0u;
  uint8_t publish_failures = 0u;
  uint8_t battery_publish_failures = 0u;
  uint32_t published_ack_count = 0u;
  uint32_t published_link_status_count = 0u;

  (void)argument;
  rmw_uros_set_custom_transport(true,(void*)&hlpuart1,
      cubemx_transport_open,cubemx_transport_close,
      cubemx_transport_write,cubemx_transport_read);
  init_ros_message_buffers();

  last_agent_ping = HAL_GetTick() - AGENT_PING_PERIOD_MS;
  last_status = HAL_GetTick();
  last_battery_status = HAL_GetTick();

  for (;;) {
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_UROS);
    uint32_t delay_ticks = 0u;
    uint32_t now = HAL_GetTick();

    switch (state) {
      case ROS_STATE_WAIT_AGENT:
        if ((uint32_t)(now - last_agent_ping) >= AGENT_PING_PERIOD_MS) {
          last_agent_ping = now;
          if (rmw_uros_ping_agent(
                  AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            state = ROS_STATE_CREATE_ENTITIES;
          }
        }
        delay_ticks = 10u;
        break;

      case ROS_STATE_CREATE_ENTITIES:
        if (create_ros_entities() != 0u) {
          spin_failures = 0u;
          publish_failures = 0u;
          battery_publish_failures = 0u;
          published_ack_count = 0u;
          published_link_status_count = 0u;
          previous_bits = 0xFFu;
          last_status = now - BRAKE_STATUS_PERIOD_MS;
          last_battery_status = now - BATTERY_STATUS_PERIOD_MS;
          last_agent_ping = now;
          state = ROS_STATE_RUNNING;
        } else {
          destroy_ros_entities();
          last_agent_ping = now;
          state = ROS_STATE_WAIT_AGENT;
        }
        break;

      case ROS_STATE_RUNNING: {
        rcl_ret_t spin_result =
            rclc_executor_spin_some(&s_ros_executor, RCL_MS_TO_NS(1));
        if (ros_operation_failed(spin_result) != 0u) {
          if (spin_failures < UINT8_MAX) spin_failures++;
        } else {
          spin_failures = 0u;
        }

        uint8_t reason_bits = 0u;
        if (ESTOP_STATUS != 0u) {
          reason_bits |=
              kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_ESTOP;
        }
        if ((MODE_FROM_A == VEHICLE_MODE_AUTO) &&
            (COMMAND_BRAKE_REQUEST != 0u)) {
          reason_bits |=
              kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_COMMAND_BRAKE;
        }
        if (((uint32_t)(now - last_status) >= BRAKE_STATUS_PERIOD_MS) ||
            (reason_bits != previous_bits)) {
          rcl_ret_t publish_result = publish_brake_status(reason_bits);
          if (ros_operation_failed(publish_result) != 0u) {
            if (publish_failures < UINT8_MAX) publish_failures++;
          } else {
            publish_failures = 0u;
          }
          previous_bits = reason_bits;
          last_status = now;
        }

        if ((uint32_t)(now - last_battery_status) >=
            BATTERY_STATUS_PERIOD_MS) {
          rcl_ret_t battery_publish_result = publish_battery_state();
          uint32_t rx_count;
          taskENTER_CRITICAL();
          rx_count = s_battery_rx_count;
          taskEXIT_CRITICAL();
          if (battery_publish_result == RCL_RET_OK &&
              rx_count != published_link_status_count) {
            battery_publish_result = publish_battery_link_status();
            if (battery_publish_result == RCL_RET_OK)
              published_link_status_count = rx_count;
          }
          if (ros_operation_failed(battery_publish_result) != 0u) {
            if (battery_publish_failures < UINT8_MAX) {
              battery_publish_failures++;
            }
          } else {
            battery_publish_failures = 0u;
          }
          last_battery_status = now;
        }

        uint32_t ack_count;
        taskENTER_CRITICAL();
        ack_count = s_battery_ack_count;
        taskEXIT_CRITICAL();
        if (ack_count != published_ack_count) {
          rcl_ret_t ack_result = publish_battery_link_ack();
          if (ros_operation_failed(ack_result) != 0u) {
            if (battery_publish_failures < UINT8_MAX)
              battery_publish_failures++;
          } else {
            published_ack_count = ack_count;
          }
        }

        if ((spin_failures >= ROS_OPERATION_FAILURE_LIMIT) ||
            (publish_failures >= ROS_OPERATION_FAILURE_LIMIT) ||
            (battery_publish_failures >= ROS_OPERATION_FAILURE_LIMIT)) {
          state = ROS_STATE_DESTROY_ENTITIES;
          break;
        }

        if ((uint32_t)(now - last_agent_ping) >= AGENT_PING_PERIOD_MS) {
          last_agent_ping = now;
          if (rmw_uros_ping_agent(
                  AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
            state = ROS_STATE_DESTROY_ENTITIES;
            break;
          }
        }
        RuntimeStatsRos_PublishIfDue(&s_runtime_ros);
        delay_ticks = 5u;
        break;
      }

      case ROS_STATE_DESTROY_ENTITIES:
      default:
        destroy_ros_entities();
        last_agent_ping = HAL_GetTick();
        state = ROS_STATE_WAIT_AGENT;
        break;
    }
    RuntimeMetrics_End(RM_UROS, activation);
    if (delay_ticks != 0u) osDelay(delay_ticks);
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
  DashboardCan_Task(argument);
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
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart4;

  while (restart_usart1_rx(&huart1) != HAL_OK) {
    osDelay(10);
  }
  LvBatteryParser_Init(&s_battery_parser);
  while (restart_uart4_rx(&huart4) != HAL_OK) {
    osDelay(10);
  }
  uint32_t last_restart_ms = HAL_GetTick();

  for (;;) {
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_RX);
    uint8_t byte;
    while (usart1_rx_ring_pop(&byte) != 0u) {
      process_vehicle_rx_byte(byte);
    }
    while (uart4_rx_ring_pop(&byte) != 0u) {
      process_battery_rx_byte(byte);
    }

    uint32_t now = HAL_GetTick();
    if ((VEHICLE_FRAME_VALID_FROM_A != 0u) &&
        ((uint32_t)(now - LAST_VEHICLE_RX_MS) >= VEHICLE_COMM_TIMEOUT_MS)) {
      VEHICLE_FRAME_VALID_FROM_A = 0u;
    }
    if ((DRIVE_DETAIL_FRAME_VALID_FROM_A != 0u) &&
        ((uint32_t)(now - LAST_DRIVE_DETAIL_RX_MS) >=
         DRIVE_DETAIL_COMM_TIMEOUT_MS)) {
      DRIVE_DETAIL_FRAME_VALID_FROM_A = 0u;
    }

    if ((s_usart1_rx_restart_requested != 0u) ||
        (((VEHICLE_FRAME_VALID_FROM_A == 0u) &&
          ((uint32_t)(now - last_restart_ms) >= VEHICLE_COMM_TIMEOUT_MS)))) {
      if (restart_usart1_rx(&huart1) == HAL_OK) {
        s_usart1_rx_restart_requested = 0u;
        last_restart_ms = now;
      }
    }
    if (s_uart4_rx_restart_requested != 0u) {
      if (restart_uart4_rx(&huart4) == HAL_OK) {
        s_uart4_rx_restart_requested = 0u;
      }
    }
    RuntimeMetrics_End(RM_RX, activation);
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

  RuntimePeriodic periodic = RuntimePeriodic_Init(pdMS_TO_TICKS(SAFETY_TX_PERIOD_MS));

  for (;;) {
    RuntimeActivation activation = RuntimePeriodic_Begin(RM_TX, &periodic);
    uint8_t frame[BOARD_UART_MAX_FRAME_SIZE];
    uint8_t payload[BOARD_UART_SAFETY_PAYLOAD_LEN];
    uint8_t stop_bits = (BRAKE_REQUEST != 0u) ? 0x01u : 0x00u;
    payload[0] = stop_bits;
    const size_t frame_size = BoardUart_EncodeFrame(
        BOARD_UART_MSG_B_TO_A_SAFETY_STATUS,
        sequence++,
        payload,
        BOARD_UART_SAFETY_PAYLOAD_LEN,
        frame,
        sizeof(frame));
    if (frame_size != 0u) {
      (void)HAL_UART_Transmit(
          &huart1, frame, (uint16_t)frame_size, 10u);
    }
    RuntimePeriodic_Wait(RM_TX, &periodic, activation);
  }
  /* USER CODE END StartATx */
}

/* USER CODE BEGIN Header_StartPerfMon */
/**
* @brief Function implementing the PerfMon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPerfMon */
void StartPerfMon(void *argument)
{
  /* USER CODE BEGIN StartPerfMon */
  RuntimePeriodic periodic = RuntimePeriodic_Init(pdMS_TO_TICKS(1000u));
  for (;;) {
    RuntimeActivation activation = RuntimePeriodic_Begin(RM_PERFMON, &periodic);
    RuntimeMetrics_Aggregate();
    RuntimePeriodic_Wait(RM_PERFMON, &periodic, activation);
  }
  /* USER CODE END StartPerfMon */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void init_ros_message_buffers(void)
{
  memset(&s_cmd_msg, 0, sizeof(s_cmd_msg));
  memset(&s_gnss_msg, 0, sizeof(s_gnss_msg));
  memset(&s_dashboard_auto_msg, 0, sizeof(s_dashboard_auto_msg));
  memset(&s_battery_command_msg, 0, sizeof(s_battery_command_msg));
  memset(&s_battery_link_status_msg, 0, sizeof(s_battery_link_status_msg));
  memset(&s_battery_link_ack_msg, 0, sizeof(s_battery_link_ack_msg));
  memset(&s_status_msg, 0, sizeof(s_status_msg));
  memset(&s_battery_msg, 0, sizeof(s_battery_msg));

  /* 수신 string은 reconnect 횟수와 무관하게 수명이 고정된 버퍼를 사용한다. */
  s_cmd_msg.header.frame_id.data = s_cmd_frame_id;
  s_cmd_msg.header.frame_id.capacity = sizeof(s_cmd_frame_id);
  s_gnss_msg.header.frame_id.data = s_gnss_frame_id;
  s_gnss_msg.header.frame_id.capacity = sizeof(s_gnss_frame_id);
  s_dashboard_auto_msg.data.data = s_dashboard_auto_data;
  s_dashboard_auto_msg.data.capacity = sizeof(s_dashboard_auto_data);
  s_battery_command_msg.data.data = s_battery_command_data;
  s_battery_command_msg.data.capacity = sizeof(s_battery_command_data);
  s_battery_link_status_msg.data.data = s_battery_link_status_data;
  s_battery_link_status_msg.data.capacity = sizeof(s_battery_link_status_data);
  s_battery_link_ack_msg.data.data = s_battery_link_ack_data;
  s_battery_link_ack_msg.data.capacity = sizeof(s_battery_link_ack_data);
  s_status_msg.header.frame_id.data = s_status_frame_id;
  s_status_msg.header.frame_id.capacity = sizeof(s_status_frame_id);
  s_battery_msg.header.frame_id.data = s_battery_frame_id;
  s_battery_msg.header.frame_id.capacity = sizeof(s_battery_frame_id);
  s_battery_msg.location.data = s_battery_location;
  s_battery_msg.location.capacity = sizeof(s_battery_location);
  s_battery_msg.serial_number.data = s_battery_serial;
  s_battery_msg.serial_number.capacity = sizeof(s_battery_serial);
  s_battery_msg.cell_voltage.data = NULL;
  s_battery_msg.cell_voltage.size = 0u;
  s_battery_msg.cell_voltage.capacity = 0u;
  s_battery_msg.cell_temperature.data = NULL;
  s_battery_msg.cell_temperature.size = 0u;
  s_battery_msg.cell_temperature.capacity = 0u;

  memcpy(s_battery_frame_id, "lv_battery", sizeof("lv_battery"));
  s_battery_msg.header.frame_id.size = sizeof("lv_battery") - 1u;
  memcpy(s_battery_location, "LV pack", sizeof("LV pack"));
  s_battery_msg.location.size = sizeof("LV pack") - 1u;
  s_battery_serial[0] = '\0';
  s_battery_msg.serial_number.size = 0u;
}

static uint8_t create_ros_entities(void)
{
  s_ros_allocator = rcl_get_default_allocator();
  memset(&s_ros_support, 0, sizeof(s_ros_support));
  s_ros_node = rcl_get_zero_initialized_node();
  s_ros_sub_cmd = rcl_get_zero_initialized_subscription();
  s_ros_sub_gnss = rcl_get_zero_initialized_subscription();
  s_ros_sub_dashboard_auto = rcl_get_zero_initialized_subscription();
  s_ros_sub_battery_command = rcl_get_zero_initialized_subscription();
  s_ros_pub_brake_status = rcl_get_zero_initialized_publisher();
  s_ros_pub_battery_state = rcl_get_zero_initialized_publisher();
  s_ros_pub_battery_link_status = rcl_get_zero_initialized_publisher();
  s_ros_pub_battery_link_ack = rcl_get_zero_initialized_publisher();
  s_ros_executor = rclc_executor_get_zero_initialized_executor();

  s_cmd_msg.header.frame_id.size = 0u;
  s_gnss_msg.header.frame_id.size = 0u;
  s_dashboard_auto_msg.data.size = 0u;
  s_battery_command_msg.data.size = 0u;
  s_battery_link_status_msg.data.size = 0u;
  s_battery_link_ack_msg.data.size = 0u;
  s_status_msg.header.frame_id.size = 0u;
  s_battery_msg.header.frame_id.size = sizeof("lv_battery") - 1u;
  s_battery_msg.location.size = sizeof("LV pack") - 1u;
  s_battery_msg.serial_number.size = 0u;

  if (rclc_support_init(
          &s_ros_support, 0, NULL, &s_ros_allocator) != RCL_RET_OK) {
    return 0u;
  }
  s_ros_support_initialized = 1u;

  if (rclc_node_init_default(
          &s_ros_node, "stm_b_node", "", &s_ros_support) != RCL_RET_OK) {
    return 0u;
  }
  s_ros_node_initialized = 1u;

  if (rclc_subscription_init_default(
          &s_ros_sub_cmd, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs, msg, ActuatorCommand),
          "/vehicle/command") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_sub_cmd_initialized = 1u;

  if (rclc_subscription_init_best_effort(
          &s_ros_sub_gnss, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
          "/gnss/fix") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_sub_gnss_initialized = 1u;

  if (rclc_subscription_init_default(
          &s_ros_sub_dashboard_auto, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
          "/dashboard/auto_status") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_sub_dashboard_auto_initialized = 1u;

  if (rclc_subscription_init_default(
          &s_ros_sub_battery_command, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
          "/lv_battery/link/command") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_sub_battery_command_initialized = 1u;

  if (rclc_publisher_init_default(
          &s_ros_pub_brake_status, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              kaiev26_msgs, msg, BrakeActuatorStatus),
          "/brake/status") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_publisher_initialized = 1u;

  if (rclc_publisher_init_default(
          &s_ros_pub_battery_state, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
          "/lv_battery/state") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_battery_publisher_initialized = 1u;

  if (rclc_publisher_init_default(
          &s_ros_pub_battery_link_status, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
          "/lv_battery/link/status") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_battery_link_status_initialized = 1u;

  if (rclc_publisher_init_default(
          &s_ros_pub_battery_link_ack, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
          "/lv_battery/link/ack") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_battery_link_ack_initialized = 1u;

  if (rclc_executor_init(
          &s_ros_executor, &s_ros_support.context, 4,
          &s_ros_allocator) != RCL_RET_OK) {
    return 0u;
  }
  s_ros_executor_initialized = 1u;

  if (rclc_executor_add_subscription(
          &s_ros_executor, &s_ros_sub_cmd, &s_cmd_msg, &cb_cmd,
          ON_NEW_DATA) != RCL_RET_OK) {
    return 0u;
  }
  if (rclc_executor_add_subscription(
          &s_ros_executor, &s_ros_sub_battery_command,
          &s_battery_command_msg, &cb_battery_command,
          ON_NEW_DATA) != RCL_RET_OK) {
    return 0u;
  }
  if (rclc_executor_add_subscription(
          &s_ros_executor, &s_ros_sub_gnss, &s_gnss_msg, &cb_gnss,
          ON_NEW_DATA) != RCL_RET_OK) {
    return 0u;
  }
  if (rclc_executor_add_subscription(
          &s_ros_executor, &s_ros_sub_dashboard_auto,
          &s_dashboard_auto_msg, &cb_dashboard_auto,
          ON_NEW_DATA) != RCL_RET_OK) {
    return 0u;
  }
  RuntimeStatsRos_Init(&s_runtime_ros, &s_ros_node,
      kaiev26_msgs__msg__RuntimeStats__BOARD_STM_B);
  return 1u;
}

static void destroy_ros_entities(void)
{
  /* Agent가 사라진 뒤 XRCE destroy 응답을 기다리며 안전 태스크를 지연하지 않는다. */
  if (s_ros_support_initialized != 0u) {
    rmw_context_t *rmw_context =
        rcl_context_get_rmw_context(&s_ros_support.context);
    if (rmw_context != NULL) {
      (void)rmw_uros_set_context_entity_destroy_session_timeout(
          rmw_context, 0);
    }
  }
  if (s_ros_executor_initialized != 0u) {
    (void)rclc_executor_fini(&s_ros_executor);
    s_ros_executor_initialized = 0u;
  }
  RuntimeStatsRos_Fini(&s_runtime_ros, &s_ros_node);
  if (s_ros_battery_link_ack_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_publisher_fini(&s_ros_pub_battery_link_ack, &s_ros_node);
    (void)fini_result;
    s_ros_battery_link_ack_initialized = 0u;
  }
  if (s_ros_battery_link_status_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_publisher_fini(&s_ros_pub_battery_link_status, &s_ros_node);
    (void)fini_result;
    s_ros_battery_link_status_initialized = 0u;
  }
  if (s_ros_battery_publisher_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_publisher_fini(&s_ros_pub_battery_state, &s_ros_node);
    (void)fini_result;
    s_ros_battery_publisher_initialized = 0u;
  }
  if (s_ros_publisher_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_publisher_fini(&s_ros_pub_brake_status, &s_ros_node);
    (void)fini_result;
    s_ros_publisher_initialized = 0u;
  }
  if (s_ros_sub_gnss_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_subscription_fini(&s_ros_sub_gnss, &s_ros_node);
    (void)fini_result;
    s_ros_sub_gnss_initialized = 0u;
  }
  if (s_ros_sub_dashboard_auto_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_subscription_fini(&s_ros_sub_dashboard_auto, &s_ros_node);
    (void)fini_result;
    s_ros_sub_dashboard_auto_initialized = 0u;
  }
  if (s_ros_sub_battery_command_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_subscription_fini(&s_ros_sub_battery_command, &s_ros_node);
    (void)fini_result;
    s_ros_sub_battery_command_initialized = 0u;
  }
  if (s_ros_sub_cmd_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_subscription_fini(&s_ros_sub_cmd, &s_ros_node);
    (void)fini_result;
    s_ros_sub_cmd_initialized = 0u;
  }
  if (s_ros_node_initialized != 0u) {
    const rcl_ret_t fini_result = rcl_node_fini(&s_ros_node);
    (void)fini_result;
    s_ros_node_initialized = 0u;
  }
  if (s_ros_support_initialized != 0u) {
    (void)rclc_support_fini(&s_ros_support);
    s_ros_support_initialized = 0u;
  }
  rcl_reset_error();
}

static rcl_ret_t publish_brake_status(uint8_t reason_bits)
{
  uint8_t command_received;
  uint32_t last_command_tick;

  taskENTER_CRITICAL();
  command_received = s_command_received;
  last_command_tick = s_last_command_tick;
  taskEXIT_CRITICAL();

  s_status_msg.brake_commanded = (BRAKE_COMMAND_APPLIED != 0u);
  s_status_msg.stop_reason_bits = reason_bits;
  /* 브레이크 서보에 피드백 신호선이 없어 actual은 항구적 미지원이다. */
  s_status_msg.actual_valid = false;
  s_status_msg.command_age_ms = (command_received != 0u) ?
      (float)((uint32_t)(HAL_GetTick() - last_command_tick)) : INFINITY;
  return rcl_publish(&s_ros_pub_brake_status, &s_status_msg, NULL);
}

static rcl_ret_t publish_battery_state(void)
{
  LvBatteryStatus status;
  uint32_t last_rx_tick;
  uint8_t received;
  const uint32_t now = HAL_GetTick();

  taskENTER_CRITICAL();
  status = s_battery_status;
  last_rx_tick = s_last_battery_rx_tick;
  received = s_battery_status_received;
  taskEXIT_CRITICAL();

  const uint8_t present =
      ((received != 0u) &&
       ((uint32_t)(now - last_rx_tick) < BATTERY_STATUS_TIMEOUT_MS)) ? 1u : 0u;
  const uint8_t measurement_valid =
      ((present != 0u) &&
       ((status.flags & LV_BATTERY_STATUS_MEASUREMENT_VALID) != 0u)) ? 1u : 0u;
  const uint8_t soc_known =
      ((present != 0u) &&
       ((status.flags & LV_BATTERY_STATUS_SOC_KNOWN) != 0u) &&
       (status.remaining_uAs >= 0) &&
       (status.soc_centi_percent >= 0)) ? 1u : 0u;

  s_battery_msg.present = (present != 0u);
  s_battery_msg.temperature = NAN;
  s_battery_msg.power_supply_health =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  s_battery_msg.power_supply_technology =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIFE;

  if (measurement_valid != 0u) {
    s_battery_msg.voltage = (float)status.bus_voltage_uV / 1000000.0f;
    /* Gauge convention is discharge positive; BatteryState is discharge negative. */
    s_battery_msg.current = -(float)status.current_uA / 1000000.0f;
    if (status.current_uA > 0) {
      s_battery_msg.power_supply_status =
          sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    } else if (status.current_uA < 0) {
      s_battery_msg.power_supply_status =
          sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
    } else {
      s_battery_msg.power_supply_status =
          sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
  } else {
    s_battery_msg.voltage = NAN;
    s_battery_msg.current = NAN;
    s_battery_msg.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  }

  if (present != 0u) {
    s_battery_msg.capacity = (float)status.capacity_mAh / 1000.0f;
    s_battery_msg.design_capacity =
        (float)status.capacity_mAh / 1000.0f;
  } else {
    s_battery_msg.capacity = NAN;
    s_battery_msg.design_capacity = NAN;
  }

  if (soc_known != 0u) {
    s_battery_msg.charge = (float)((double)status.remaining_uAs / 3.6e9);
    s_battery_msg.percentage =
        (float)status.soc_centi_percent / 10000.0f;
  } else {
    s_battery_msg.charge = NAN;
    s_battery_msg.percentage = NAN;
  }

  return rcl_publish(&s_ros_pub_battery_state, &s_battery_msg, NULL);
}

static rcl_ret_t publish_battery_link_status(void)
{
  LvBatteryStatus status;
  uint8_t received;
  taskENTER_CRITICAL();
  status = s_battery_status;
  received = s_battery_status_received;
  taskEXIT_CRITICAL();
  if (received == 0u) return RCL_RET_OK;
  LvBattery_EncodeStatusPayload(&status, s_battery_link_status_data);
  s_battery_link_status_msg.data.size = LV_BATTERY_STATUS_PAYLOAD_SIZE;
  return rcl_publish(
      &s_ros_pub_battery_link_status, &s_battery_link_status_msg, NULL);
}

static rcl_ret_t publish_battery_link_ack(void)
{
  LvBatteryAck ack;
  taskENTER_CRITICAL();
  ack = s_battery_ack;
  taskEXIT_CRITICAL();
  LvBattery_EncodeAckPayload(&ack, s_battery_link_ack_data);
  s_battery_link_ack_msg.data.size = LV_BATTERY_ACK_PAYLOAD_SIZE;
  return rcl_publish(&s_ros_pub_battery_link_ack,
                     &s_battery_link_ack_msg, NULL);
}

static uint8_t ros_operation_failed(rcl_ret_t result)
{
  return ((result != RCL_RET_OK) && (result != RCL_RET_TIMEOUT)) ? 1u : 0u;
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
  if (last_byte == BOARD_UART_SOF1) {
    s_vehicle_parser.data[0] = BOARD_UART_SOF1;
    s_vehicle_parser.index = 1u;
  }
}

static void process_vehicle_rx_byte(uint8_t byte)
{
  uart_frame_parser_t *parser = &s_vehicle_parser;

  if (parser->index == 0u) {
    if (byte == BOARD_UART_SOF1) {
      parser->data[0] = byte;
      parser->index = 1u;
    }
    return;
  }

  if (parser->index == 1u) {
    if (byte == BOARD_UART_SOF2) {
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
    const uint8_t message_type = parser->data[2];
    const uint8_t payload_length = parser->data[3];
    const uint8_t valid_vehicle =
        ((message_type == BOARD_UART_MSG_A_TO_B_VEHICLE_STATUS) &&
         (payload_length == BOARD_UART_VEHICLE_PAYLOAD_LEN)) ? 1u : 0u;
    const uint8_t valid_detail =
        ((message_type == BOARD_UART_MSG_A_TO_B_DRIVE_DETAIL) &&
         (payload_length == BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN)) ? 1u : 0u;
    if ((valid_vehicle == 0u) && (valid_detail == 0u)) {
      reset_vehicle_parser(byte);
      return;
    }
    parser->expected_size = (uint8_t)(6u + parser->data[3]);
  }

  if ((parser->expected_size != 0u) &&
      (parser->index == parser->expected_size)) {
    uint8_t payload_len = parser->data[3];
    uint8_t received_crc = parser->data[5u + payload_len];
    uint8_t calculated_crc = BoardUart_Crc8(
        &parser->data[2], (uint8_t)(3u + payload_len));
    const uint8_t message_type = parser->data[2];

    if (received_crc != calculated_crc) {
      if (message_type == BOARD_UART_MSG_A_TO_B_DRIVE_DETAIL) {
        DRIVE_DETAIL_CRC_ERROR_COUNT++;
      } else {
        VEHICLE_CRC_ERROR_COUNT++;
      }
    } else if (message_type == BOARD_UART_MSG_A_TO_B_VEHICLE_STATUS) {
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
    } else {
      BoardUartDriveDetail_t detail;
      if (BoardUart_DecodeDriveDetailPayload(
              &parser->data[5], payload_len, &detail)) {
        uint8_t sequence = parser->data[4];
        if ((s_detail_sequence_initialized != 0u) &&
            (sequence != (uint8_t)(s_detail_last_sequence + 1u))) {
          DRIVE_DETAIL_SEQUENCE_ERROR_COUNT++;
        }
        s_detail_last_sequence = sequence;
        s_detail_sequence_initialized = 1u;

        taskENTER_CRITICAL();
        DRIVE_DETAIL_FROM_A = detail;
        LAST_DRIVE_DETAIL_RX_MS = HAL_GetTick();
        DRIVE_DETAIL_FRAME_VALID_FROM_A = 1u;
        DRIVE_DETAIL_RX_COUNT++;
        taskEXIT_CRITICAL();
      }
    }
    reset_vehicle_parser(byte);
  }
}

static void uart4_rx_ring_push(uint8_t byte)
{
  uint16_t next = (uint16_t)(s_uart4_rx_head + 1u);
  if (next >= BATTERY_UART_RX_RING_SIZE) next = 0u;
  if (next != s_uart4_rx_tail) {
    s_uart4_rx_ring[s_uart4_rx_head] = byte;
    s_uart4_rx_head = next;
  } else {
    s_uart4_rx_overflow_count++;
  }
}

static uint8_t uart4_rx_ring_pop(uint8_t *byte)
{
  uint8_t available = 0u;
  taskENTER_CRITICAL();
  if (s_uart4_rx_tail != s_uart4_rx_head) {
    *byte = s_uart4_rx_ring[s_uart4_rx_tail];
    s_uart4_rx_tail++;
    if (s_uart4_rx_tail >= BATTERY_UART_RX_RING_SIZE) {
      s_uart4_rx_tail = 0u;
    }
    available = 1u;
  }
  taskEXIT_CRITICAL();
  return available;
}

static HAL_StatusTypeDef restart_uart4_rx(UART_HandleTypeDef *huart)
{
  (void)HAL_UART_AbortReceive(huart);
  taskENTER_CRITICAL();
  s_uart4_rx_head = 0u;
  s_uart4_rx_tail = 0u;
  LvBatteryParser_Init(&s_battery_parser);
  taskEXIT_CRITICAL();
  return HAL_UART_Receive_IT(huart, &s_uart4_rx_byte, 1u);
}

static void process_battery_rx_byte(uint8_t byte)
{
  LvBatteryStatus status;
  LvBatteryAck ack;
  const LvBatteryParseResult result =
      LvBatteryParser_Feed(&s_battery_parser, byte, &status, &ack);

  if (result == LV_BATTERY_PARSE_STATUS) {
    if ((s_battery_sequence_initialized != 0u) &&
        (status.frame_sequence != s_battery_last_sequence + 1u)) {
      s_battery_sequence_error_count++;
    }
    s_battery_last_sequence = status.frame_sequence;
    s_battery_sequence_initialized = 1u;

    taskENTER_CRITICAL();
    s_battery_status = status;
    s_last_battery_rx_tick = HAL_GetTick();
    s_battery_rx_count++;
    s_battery_status_received = 1u;
    taskEXIT_CRITICAL();
    DashboardCan_ObserveLvBattery(&status);
  } else if (result == LV_BATTERY_PARSE_ACK) {
    taskENTER_CRITICAL();
    s_battery_ack = ack;
    s_battery_ack_count++;
    taskEXIT_CRITICAL();
  } else if (result == LV_BATTERY_PARSE_CRC_ERROR) {
    s_battery_crc_error_count++;
  } else if (result == LV_BATTERY_PARSE_FORMAT_ERROR) {
    s_battery_format_error_count++;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  const uint32_t entry_cycle = RuntimeMetrics_Now();
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
    RuntimeMetrics_RecordEStopISR(entry_cycle);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) {
    uart4_rx_ring_push(s_uart4_rx_byte);
    if (HAL_UART_Receive_IT(huart, &s_uart4_rx_byte, 1u) != HAL_OK) {
      s_uart4_rx_restart_requested = 1u;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    s_usart1_rx_restart_requested = 1u;
  } else if (huart->Instance == UART4) {
    s_uart4_rx_restart_requested = 1u;
  }
}

/* USER CODE END Application */
