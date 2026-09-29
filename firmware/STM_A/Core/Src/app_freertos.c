/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : STM_A FreeRTOS 태스크 전체 구현
  *                      베이스(uROS+UART 동작 확인) 구조 유지,
  *                      Control / Inputs 및 FDCAN RX callback
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "command_brake_order.h"
#include "longitudinal_controller.h"
#include "ezkontrol_command.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "board_uart_protocol.h"
#include "runtime_periodic.h"
#include "runtime_stats_ros.h"
#include "globals.h"
#include "usart.h"
#include "adc.h"
#include "fdcan.h"
#include "gpio.h"

#include <std_msgs/msg/float32_multi_array.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <kaiev26_msgs/msg/actuator_command.h>
#include <kaiev26_msgs/msg/drive_actuator_status.h>
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

enum {
  CONTROLLER_CONTROL_MODE_TORQUE = 0u,
  CONTROLLER_CONTROL_MODE_SPEED = 1u
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIRE_DIAM_M   0.47f
#define GEAR_RATIO    4.60f
#define RPM_FWD_MAX   4500
#define RPM_REV_MAX   1500
#define AUTO_SPEED_MAX_MPS \
  ((float)RPM_FWD_MAX * 3.14159f * TIRE_DIAM_M / (60.0f * GEAR_RATIO))
#define MANUAL_TORQUE_LIMIT_A 480.0f
#define PEDAL_ADC_MIN 560
#define PEDAL_ADC_MAX 3000
#define UART_RX_DMA_SIZE 64u
#define UART_RX_RING_SIZE 128u
#define STM_B_COMM_TIMEOUT_MS 100u
#define VEHICLE_TX_PERIOD_MS 20u
#define DRIVE_DETAIL_TX_PERIOD_MS 100u
#define HOST_COMMAND_TIMEOUT_MS 500u
#define MOTOR_FEEDBACK_TIMEOUT_MS 200u
#define MOTOR_RPM_ZERO_HOLD_SAMPLES 2u
#define AGENT_PING_PERIOD_MS 500u
#define AGENT_PING_TIMEOUT_MS 100u
/* 20ms(50Hz). 벤치에서 페달 밟는 속도와 EZkontrol 응답을 놓치지 않을 만큼 촘촘하다.
 * RuntimeStats fixed frame도 같은 LPUART1을 쓰므로 460800 baud로 전송 여유를 확보한다.
 * 여유가 모자라면 주기가 아니라 baud를 올린다. */
#define DRIVE_STATUS_PERIOD_MS 20u
#define MOTOR_STATUS_PART1_ID 0x1801D0EFUL
#define MOTOR_STATUS_PART2_ID 0x1802D0EFUL

/* EZkontrol Part II ERROR bytes 3..5: 원본 24-bit fault mapping. */
#define MOTOR_FAULT_OVERCURRENT             (1UL << 0)
#define MOTOR_FAULT_OVERLOAD                (1UL << 1)
#define MOTOR_FAULT_OVERVOLTAGE             (1UL << 2)
#define MOTOR_FAULT_UNDERVOLTAGE            (1UL << 3)
#define MOTOR_FAULT_CONTROLLER_OVERHEAT     (1UL << 4)
#define MOTOR_FAULT_MOTOR_OVERHEAT          (1UL << 5)
#define MOTOR_FAULT_STALLED                 (1UL << 6)
#define MOTOR_FAULT_OUT_OF_PHASE            (1UL << 7)
#define MOTOR_FAULT_SENSOR                  (1UL << 8)
#define MOTOR_FAULT_AUX_SENSOR              (1UL << 9)
#define MOTOR_FAULT_ENCODER_MISALIGNED      (1UL << 10)
#define MOTOR_FAULT_ANTI_RUNAWAY            (1UL << 11)
#define MOTOR_FAULT_MAIN_ACCELERATOR        (1UL << 12)
#define MOTOR_FAULT_AUX_ACCELERATOR         (1UL << 13)
#define MOTOR_FAULT_PRECHARGE               (1UL << 14)
#define MOTOR_FAULT_DC_CONTACTOR             (1UL << 15)
#define MOTOR_FAULT_POWER_VALVE              (1UL << 16)
#define MOTOR_FAULT_CURRENT_SENSOR           (1UL << 17)
#define MOTOR_FAULT_AUTO_TUNE                (1UL << 18)
#define MOTOR_FAULT_RS485                    (1UL << 19)
#define MOTOR_FAULT_CAN                      (1UL << 20)
#define MOTOR_FAULT_SOFTWARE                 (1UL << 21)
#define MOTOR_FAULT_RESERVED_22              (1UL << 22)
#define MOTOR_FAULT_RESERVED_23              (1UL << 23)
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
/* USART1 Receive-to-Idle DMA -> ISR ring buffer -> BRx parser */
static uint8_t s_usart1_rx_dma[UART_RX_DMA_SIZE];
static uint8_t s_usart1_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t s_usart1_rx_head = 0u;
static volatile uint16_t s_usart1_rx_tail = 0u;
static volatile uint8_t s_usart1_rx_restart_requested = 0u;
static uint16_t s_usart1_rx_dma_pos = 0u;
static uart_frame_parser_t s_safety_parser;
static uint8_t s_safety_last_sequence = 0u;
static uint8_t s_safety_sequence_initialized = 0u;
static uint8_t s_life = 0;
/* 데드밴드 클램핑 전 raw ADC 값. StartInputs가 쓰고 StartuROS가 발행한다.
 * 캘리브레이션(PEDAL_ADC_MIN/MAX)이 바뀌어도 과거 로그를 다시 계산할 수 있게 남긴다. */
static volatile int32_t s_pedal_adc_raw = 0;
static volatile float s_motor_rpm_raw = 0.0f;
static int32_t s_last_nonzero_motor_rpm = 0;
static uint8_t s_motor_rpm_zero_streak = 0u;
static uint8_t s_motor_rpm_hold_armed = 0u;
static volatile uint8_t s_command_received = 0u;
static volatile uint8_t s_command_timeout = 1u;
static volatile uint8_t s_command_brake_engage = 1u;
static volatile uint32_t s_last_command_tick = 0u;
static volatile uint8_t s_motor_handshake_done = 0u;
static volatile uint32_t s_motor_handshake_count = 0u;
static volatile uint8_t s_tractive_commanded = 0u;

/* send_motor_can이 버스에 실제로 실은 명령. 인코딩과 클램핑을 되돌린 값이라
 * 지령과 실측(Part I)을 같은 단위로 나란히 볼 수 있다. */
static volatile float s_commanded_phase_current_a = 0.0f;
static volatile float s_commanded_motor_rpm = 0.0f;

/* CAN ISR가 갱신하고 uROS task가 짧은 critical section에서 snapshot한다. */
static volatile float s_bus_voltage_v = 0.0f;
static volatile float s_bus_current_a = 0.0f;
static volatile float s_phase_current_a = 0.0f;
static volatile uint32_t s_last_part1_rx_ms = 0u;
static volatile uint8_t s_part1_received = 0u;

static volatile float s_controller_temperature_c = 0.0f;
static volatile float s_motor_temperature_c = 0.0f;
static volatile uint8_t s_controller_running = 0u;
static volatile uint8_t s_controller_control_mode = CONTROLLER_CONTROL_MODE_TORQUE;
static volatile uint32_t s_motor_fault_bits = 0u;
static volatile uint32_t s_last_part2_rx_ms = 0u;
static volatile uint8_t s_part2_received = 0u;

/* micro-ROS entities are retained across reconnect attempts. */
static RuntimeStatsRos s_runtime_ros;
static rcl_allocator_t s_ros_allocator;
static rclc_support_t s_ros_support;
static rcl_node_t s_ros_node;
static rcl_subscription_t s_ros_sub_cmd;
static rcl_publisher_t s_ros_pub_drive_status;
static rclc_executor_t s_ros_executor;
static uint8_t s_ros_support_initialized = 0u;
static uint8_t s_ros_node_initialized = 0u;
static uint8_t s_ros_sub_cmd_initialized = 0u;
static uint8_t s_ros_publisher_initialized = 0u;
static uint8_t s_ros_executor_initialized = 0u;

static kaiev26_msgs__msg__ActuatorCommand s_cmd_msg;
static kaiev26_msgs__msg__DriveActuatorStatus s_status_msg;
static char s_cmd_frame_id[16];
static char s_status_frame_id[4];

static LongCalibration s_long_calibration;
static LongController s_long_controller;
static LongRequest s_long_pending;
static uint8_t s_long_pending_valid;

static uint8_t s_long_feedback_valid, s_long_inhibit = LONG_DISABLED;
/* MANUAL pedal command after the configured slew; Control task only. */
static float s_manual_current_a;
static LongGate s_long_gate;
static float s_long_status_data[LONG_STATUS_COUNT];
static float s_long_request_data[LONG_REQUEST_COUNT];
static std_msgs__msg__Float32MultiArray s_long_status_msg, s_long_request_msg;
static rcl_subscription_t s_long_subscription;
static rcl_publisher_t s_long_publisher;
static uint8_t s_long_subscription_initialized, s_long_publisher_initialized;

/* USER CODE END Variables */
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for Inputs */
osThreadId_t InputsHandle;
const osThreadAttr_t Inputs_attributes = {
  .name = "Inputs",
  .priority = (osPriority_t) osPriorityHigh,
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
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 256 * 4
};
/* Definitions for BTx */
osThreadId_t BTxHandle;
const osThreadAttr_t BTx_attributes = {
  .name = "BTx",
  .priority = (osPriority_t) osPriorityLow,
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
static void send_motor_can(int32_t rpm, float curr_a, uint8_t run_command,
                           uint8_t speed_control);
static HAL_StatusTypeDef send_motor_handshake_ack(void);
static void cb_cmd(const void *message);
static void cb_long_config(const void *message);
static void prepare_longitudinal(uint32_t now);
static void publish_longitudinal(void);
static void reset_host_command_state(void);
static void init_ros_message_buffers(void);
static uint8_t create_ros_entities(void);
static void destroy_ros_entities(void);
static uint32_t age_since(uint32_t now, uint32_t stamp);
static void reset_motor_rpm_despiker(void);
static int32_t despike_motor_rpm(int32_t motor_rpm_raw);
static void capture_drive_detail(uint32_t now,
                                 BoardUartDriveDetail_t *detail);
static void publish_drive_status(uint32_t now);
static void parse_motor_status_part1(const uint8_t data[8], uint32_t now);
static void parse_motor_status_part2(const uint8_t data[8], uint32_t now);
static void usart1_rx_ring_push(uint8_t byte);
static uint8_t usart1_rx_ring_pop(uint8_t *byte);
static void reset_safety_parser(uint8_t last_byte);
static void process_safety_rx_byte(uint8_t byte);
static HAL_StatusTypeDef restart_usart1_rx(UART_HandleTypeDef *huart);
/* USER CODE END FunctionPrototypes */

void StartControl(void *argument);
void StartInputs(void *argument);
void StartuROS(void *argument);
void StartBRx(void *argument);
void StartBTx(void *argument);
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

  /* Task: Control — 모터 제어 (osPriorityRealtime) */

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
  s_long_calibration.config = Long_DefaultConfig();
  Long_Reset(&s_long_controller);

  /* creation of Control */
  ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);

  /* creation of Inputs */
  InputsHandle = osThreadNew(StartInputs, NULL, &Inputs_attributes);

  /* creation of uROS */
  uROSHandle = osThreadNew(StartuROS, NULL, &uROS_attributes);

  /* creation of BRx */
  BRxHandle = osThreadNew(StartBRx, NULL, &BRx_attributes);

  /* creation of BTx */
  BTxHandle = osThreadNew(StartBTx, NULL, &BTx_attributes);

  /* creation of PerfMon */
  PerfMonHandle = osThreadNew(StartPerfMon, NULL, &PerfMon_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  RuntimeMetrics_Register(RM_CONTROL, (TaskHandle_t)ControlHandle);
  RuntimeMetrics_Register(RM_INPUTS, (TaskHandle_t)InputsHandle);
  RuntimeMetrics_Register(RM_UROS, (TaskHandle_t)uROSHandle);
  RuntimeMetrics_Register(RM_RX, (TaskHandle_t)BRxHandle);
  RuntimeMetrics_Register(RM_TX, (TaskHandle_t)BTxHandle);
  RuntimeMetrics_Register(RM_PERFMON, (TaskHandle_t)PerfMonHandle);

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

  /* EZkontrol status Part I/II의 두 extended ID만 FIFO0으로 받는다. */
  FDCAN_FilterTypeDef f;
  f.IdType       = FDCAN_EXTENDED_ID;
  f.FilterIndex  = 0;
  f.FilterType   = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = MOTOR_STATUS_PART1_ID;
  f.FilterID2    = MOTOR_STATUS_PART2_ID;
  (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
  (void)HAL_FDCAN_ConfigGlobalFilter(
      &hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
      FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  (void)HAL_FDCAN_Start(&hfdcan1);
  // 수신 인터럽트 ON
  (void)HAL_FDCAN_ActivateNotification(
      &hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  RuntimePeriodic periodic = RuntimePeriodic_Init(pdMS_TO_TICKS(50u));

  for (;;)
  {
    RuntimeActivation activation = RuntimePeriodic_Begin(RM_CONTROL, &periodic);
    prepare_longitudinal(HAL_GetTick());
    if (s_motor_handshake_done == 0u)
    {
      Long_Reset(&s_long_controller);
      s_manual_current_a = 0.0f;
      s_tractive_commanded = 0u;
      RuntimeMetrics_End(RM_CONTROL, activation);
      osDelay(10);
      periodic = RuntimePeriodic_Init(pdMS_TO_TICKS(50u));
      continue;
    }

    uint8_t drive_permitted = (DRIVE_STOP_REQUEST == 0u) ? 1u : 0u;
    if (MODE == 1u)
    {
      const uint32_t now = HAL_GetTick();
      const uint8_t command_received = s_command_received;
      const uint8_t command_timed_out =
          ((command_received == 0u) ||
           (age_since(now, s_last_command_tick) >= HOST_COMMAND_TIMEOUT_MS))
              ? 1u : 0u;
      s_command_timeout = command_timed_out;

      /* AUTO는 새롭고 유효한 host 명령이 있어야만 RUN 가능하다. timeout 시
       * 과거 속도를 즉시 지워 재연결/복구 뒤 과거 명령으로 재출발하지 않는다. */
      if ((command_timed_out != 0u) ||
          (s_command_brake_engage != 0u))
      {
        COMMAND_SPEED = 0.0f;
        drive_permitted = 0u;
      }
    }
    else
    {
      /* MANUAL은 페달 구동이므로 PC command timeout을 적용하지 않는다. */
      s_command_timeout = 0u;
    }

    if (drive_permitted != 0u)
    {
      if (MODE == 1u)
      {
        s_manual_current_a = 0.0f;
        if (s_long_calibration.config.mode == LONG_SPEED) {
          /* Native speed loop with the active preset envelope; driving stays 480 A/4500 RPM. */
          Long_Reset(&s_long_controller);
          float wrpm = COMMAND_SPEED * 60.0f / (3.14159f * TIRE_DIAM_M);
          int32_t mrpm = (int32_t)(wrpm * GEAR_RATIO);
          const LongMotorCommand command = Long_SpeedCommand(&s_long_calibration.config, mrpm);
          send_motor_can(command.rpm, command.current, 1u, 1u);
        } else if (s_long_inhibit == LONG_OK && COMMAND_SPEED > 0.0f) {
          const float ceiling = s_long_calibration.config.rpm_limit * 3.14159f * TIRE_DIAM_M / (60.0f * GEAR_RATIO);
          float current = Long_Update(&s_long_controller, &s_long_calibration.config,
              fminf(COMMAND_SPEED, ceiling), CURRENT_SPEED, 0.05f);
          send_motor_can((int32_t)s_long_calibration.config.rpm_limit, current, 1u, 0u);
        } else {
          Long_Reset(&s_long_controller);
          send_motor_can(0, 0.0f, 0u, 0u);
        }
      }
      else
      {
        /* 수동주행: 페달 0~100%를 상전류로 매핑한다. Torque calibration에서는 적용된
         * 전류 한도·회전수 상한·전류 상승/하강률을 AUTO Torque와 공유하고, Speed
         * calibration에서는 기존 0~480 A 매핑을 그대로 쓴다. 두 경우 모두 페달이
         * 토크 요구이며 속도 PI는 MANUAL에서 동작하지 않는다. */
        Long_Reset(&s_long_controller);
        float torque_current;
        int32_t rpm_limit;
        if (s_long_calibration.config.mode == LONG_TORQUE)
        {
          s_manual_current_a = Long_PedalCommand(&s_long_calibration.config,
              (float)PEDAL, s_manual_current_a, 0.05f);
          torque_current = s_manual_current_a;
          rpm_limit = (int32_t)s_long_calibration.config.rpm_limit;
        }
        else
        {
          s_manual_current_a = 0.0f;
          torque_current = ((float)PEDAL / 100.0f) * MANUAL_TORQUE_LIMIT_A;
          rpm_limit = RPM_FWD_MAX;
        }
        if (GEAR == 1u)
        {
          torque_current = -torque_current;
          rpm_limit = -rpm_limit;
        }
        send_motor_can(rpm_limit, torque_current, 1u, 0u);
      }
    }
    // ESTOP 활성시
    else
    {
      /* 정지 시 RPM/전류뿐 아니라 RUNNING bit도 내려 HALTED를 명령한다.
       * 안전 정지는 페달 slew도 거치지 않는다. */
      Long_Reset(&s_long_controller);
      s_manual_current_a = 0.0f;
      if (s_long_inhibit == LONG_OK) s_long_inhibit = LONG_STOP;
      send_motor_can(0, 0.0f, 0u,
          (MODE == 1u && s_long_calibration.config.mode == LONG_SPEED) ? 1u : 0u);
    }
    // 20Hz / 50ms 주기
    RuntimePeriodic_Wait(RM_CONTROL, &periodic, activation);
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
  RuntimePeriodic periodic = RuntimePeriodic_Init(pdMS_TO_TICKS(50u));

  for (;;)
  {
    RuntimeActivation activation = RuntimePeriodic_Begin(RM_INPUTS, &periodic);

    /* 페달 ADC 8회 평균 */
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 10);  // 변환 완료 대기(최대 10ms)
      sum += HAL_ADC_GetValue(&hadc1);
    }
    int raw = (int)(sum / 8u);
    // 클램핑 전 raw 값은 /drive/status의 pedal_adc_raw로 그대로 나간다
    s_pedal_adc_raw = (int32_t)raw;
    // 데드밴드 클램핑 (무반응 구간, 유격감안)
    if (raw < PEDAL_ADC_MIN) raw = PEDAL_ADC_MIN;
    if (raw > PEDAL_ADC_MAX) raw = PEDAL_ADC_MAX;
    // 0~100% 선형 매핑
    int p = (raw - PEDAL_ADC_MIN) * 100 / (PEDAL_ADC_MAX - PEDAL_ADC_MIN);
    // 떨림 방지를 위한 미세값 보정
    if (p < 2) p = 0;
    PEDAL = (uint8_t)p;

    // Inputs 태스크는 페달과 모드/기어 스위치만 담당한다.
    // SW_MODE_Pin은 내부 pull-up이라 단선/플로팅 시 HIGH가 기본값이 된다.

    // MODE_Pin pull-up default value = 0 (manual)
    MODE = (HAL_GPIO_ReadPin(SW_MODE_GPIO_Port, SW_MODE_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
    // GEAR_Pin pull-up default value = 0 (forward)
    GEAR = (HAL_GPIO_ReadPin(SW_GEAR_GPIO_Port, SW_GEAR_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

    RuntimePeriodic_Wait(RM_INPUTS, &periodic, activation);
  }
  /* USER CODE END StartInputs */
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

  extern UART_HandleTypeDef hlpuart1; // uROS UART
  (void)rmw_uros_set_custom_transport(
      true, (void *)&hlpuart1,
      cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read);
  init_ros_message_buffers();
  reset_host_command_state();

  ros_connection_state_t state = ROS_STATE_WAIT_AGENT;
  uint32_t last_status = HAL_GetTick();
  uint32_t last_long_status = HAL_GetTick();
  uint32_t last_agent_ping = HAL_GetTick() - AGENT_PING_PERIOD_MS;

  for (;;)
  {
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_UROS);
    uint32_t delay_ticks = 0u;
    const uint32_t now = HAL_GetTick();

    switch (state)
    {
      case ROS_STATE_WAIT_AGENT:
        if ((uint32_t)(now - last_agent_ping) >= AGENT_PING_PERIOD_MS) {
          last_agent_ping = now;
          if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            state = ROS_STATE_CREATE_ENTITIES;
          }
        }
        delay_ticks = 10u;
        break;

      case ROS_STATE_CREATE_ENTITIES:
        reset_host_command_state();
        if (create_ros_entities() != 0u) {
          last_status = now - DRIVE_STATUS_PERIOD_MS;
          last_agent_ping = now;
          state = ROS_STATE_RUNNING;
        } else {
          destroy_ros_entities();
          last_agent_ping = now;
          state = ROS_STATE_WAIT_AGENT;
        }
        break;

      case ROS_STATE_RUNNING:
      {
        const rcl_ret_t spin_result =
            rclc_executor_spin_some(&s_ros_executor, RCL_MS_TO_NS(1));
        if ((spin_result != RCL_RET_OK) &&
            (spin_result != RCL_RET_TIMEOUT)) {
          reset_host_command_state();
          state = ROS_STATE_DESTROY_ENTITIES;
          break;
        }

        if ((uint32_t)(now - last_status) >= DRIVE_STATUS_PERIOD_MS) {
          publish_drive_status(now);
          /* Agent ping 등으로 지연된 뒤 catch-up burst를 만들지 않고 status 주기를 다시 맞춘다. */
          last_status = now;
        }

        if ((uint32_t)(now - last_agent_ping) >= AGENT_PING_PERIOD_MS) {
          last_agent_ping = now;
          if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
            reset_host_command_state();
            state = ROS_STATE_DESTROY_ENTITIES;
            break;
          }
        }
        if ((uint32_t)(now - last_long_status) >= 200u) {
          publish_longitudinal();
          last_long_status = now;
        }
        RuntimeStatsRos_PublishIfDue(&s_runtime_ros);
        delay_ticks = 5u;
        break;
      }

      case ROS_STATE_DESTROY_ENTITIES:
      default:
        reset_host_command_state();
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

/* USER CODE BEGIN Header_StartBRx */
/**
* @brief Function implementing the BRx thread.
* @param argument: Not used
* @retval None
*/

/* ========================================================= */ 
/* Task: BRx — STM_B -> A Safety frame 수신 (osPriorityAboveNormal) */
/* ========================================================= */
/* USER CODE END Header_StartBRx */
void StartBRx(void *argument)
{
  /* USER CODE BEGIN StartBRx */
  extern UART_HandleTypeDef huart1;

  while (restart_usart1_rx(&huart1) != HAL_OK) {
    osDelay(10);
  }
  uint32_t last_restart_ms = HAL_GetTick();

  for (;;)
  {
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_RX);
    uint8_t byte = 0u;
    while (usart1_rx_ring_pop(&byte) != 0u) {
      process_safety_rx_byte(byte);
    }

    uint32_t now = HAL_GetTick();
    if ((SAFETY_FRAME_VALID_FROM_B == 0u) ||
        ((uint32_t)(now - LAST_SAFETY_RX_MS) >= STM_B_COMM_TIMEOUT_MS)) {
      SAFETY_FRAME_VALID_FROM_B = 0u;
      STM_B_COMM_TIMEOUT = 1u;
    } else {
      STM_B_COMM_TIMEOUT = 0u;
    }

    DRIVE_STOP_REQUEST = (((MODE == 1u) &&
                           (s_command_brake_engage != 0u)) ||
                          (BRAKE_REQUEST_FROM_B != 0u) ||
                          (SAFETY_FRAME_VALID_FROM_B == 0u) ||
                          (STM_B_COMM_TIMEOUT != 0u)) ? 1u : 0u;

    if ((s_usart1_rx_restart_requested != 0u) ||
        (((SAFETY_FRAME_VALID_FROM_B == 0u) &&
          ((uint32_t)(now - last_restart_ms) >= STM_B_COMM_TIMEOUT_MS)))) {
      if (restart_usart1_rx(&huart1) == HAL_OK) {
        s_usart1_rx_restart_requested = 0u;
        last_restart_ms = now;
      }
    }
    RuntimeMetrics_End(RM_RX, activation);
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

/* ================================================================ */ 
/* Task: BTx — STM_A -> STM_B Vehicle Status frame 송신 */
/* ================================================================ */
/* USER CODE END Header_StartBTx */
void StartBTx(void *argument)
{
  /* USER CODE BEGIN StartBTx */
  extern UART_HandleTypeDef huart1;
  uint8_t vehicle_sequence = 0u;
  uint8_t detail_sequence = 0u;
  uint32_t last_detail_ms = HAL_GetTick() - DRIVE_DETAIL_TX_PERIOD_MS;

  RuntimePeriodic periodic = RuntimePeriodic_Init(pdMS_TO_TICKS(VEHICLE_TX_PERIOD_MS));

  for (;;)
  {
    RuntimeActivation activation = RuntimePeriodic_Begin(RM_TX, &periodic);
    uint8_t frame[BOARD_UART_MAX_FRAME_SIZE];
    uint8_t payload[BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN];
    uint8_t status = 0u;
    size_t frame_size;
    float speed_x100_f = CURRENT_SPEED * 100.0f;
    if (speed_x100_f > 32767.0f) speed_x100_f = 32767.0f;
    if (speed_x100_f < -32768.0f) speed_x100_f = -32768.0f;
    int16_t speed_x100 = (int16_t)lroundf(speed_x100_f);

    status |= (uint8_t)((MODE != 0u) ? (1u << 0) : 0u);
    status |= (uint8_t)((GEAR != 0u) ? (1u << 1) : 0u);

    payload[0] = status;
    payload[1] = (uint8_t)((uint16_t)speed_x100 & 0xFFu);
    payload[2] = (uint8_t)(((uint16_t)speed_x100 >> 8) & 0xFFu);
    frame_size = BoardUart_EncodeFrame(
        BOARD_UART_MSG_A_TO_B_VEHICLE_STATUS,
        vehicle_sequence++,
        payload,
        BOARD_UART_VEHICLE_PAYLOAD_LEN,
        frame,
        sizeof(frame));
    if (frame_size != 0u)
    {
      (void)HAL_UART_Transmit(
          &huart1, frame, (uint16_t)frame_size, 10u);
    }

    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_detail_ms) >= DRIVE_DETAIL_TX_PERIOD_MS)
    {
      BoardUartDriveDetail_t detail;
      capture_drive_detail(now, &detail);
      BoardUart_EncodeDriveDetailPayload(&detail, payload);
      frame_size = BoardUart_EncodeFrame(
          BOARD_UART_MSG_A_TO_B_DRIVE_DETAIL,
          detail_sequence++,
          payload,
          BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN,
          frame,
          sizeof(frame));
      if (frame_size != 0u)
      {
        (void)HAL_UART_Transmit(
            &huart1, frame, (uint16_t)frame_size, 10u);
      }
      last_detail_ms = now;
    }
    RuntimePeriodic_Wait(RM_TX, &periodic, activation);
  }
  /* USER CODE END StartBTx */
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
  reset_safety_parser(0u);
  taskEXIT_CRITICAL();
  return HAL_UARTEx_ReceiveToIdle_DMA(huart, s_usart1_rx_dma,
                                     UART_RX_DMA_SIZE);
}

static void reset_safety_parser(uint8_t last_byte)
{
  s_safety_parser.index = 0u;
  s_safety_parser.expected_size = 0u;
  if (last_byte == BOARD_UART_SOF1) {
    s_safety_parser.data[0] = BOARD_UART_SOF1;
    s_safety_parser.index = 1u;
  }
}

static void process_safety_rx_byte(uint8_t byte)
{
  uart_frame_parser_t *parser = &s_safety_parser;

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
      reset_safety_parser(byte);
    }
    return;
  }

  if (parser->index >= sizeof(parser->data)) {
    reset_safety_parser(byte);
    return;
  }
  parser->data[parser->index++] = byte;

  if (parser->index == 4u) {
    if ((parser->data[2] != BOARD_UART_MSG_B_TO_A_SAFETY_STATUS) ||
        (parser->data[3] != BOARD_UART_SAFETY_PAYLOAD_LEN)) {
      reset_safety_parser(byte);
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

    if (received_crc != calculated_crc) {
      SAFETY_CRC_ERROR_COUNT++;
    } else {
      uint8_t sequence = parser->data[4];
      if ((s_safety_sequence_initialized != 0u) &&
          (sequence != (uint8_t)(s_safety_last_sequence + 1u))) {
        SAFETY_SEQUENCE_ERROR_COUNT++;
      }
      s_safety_last_sequence = sequence;
      s_safety_sequence_initialized = 1u;

      taskENTER_CRITICAL();
      BRAKE_REQUEST_FROM_B = parser->data[5] & 0x01u;
      LAST_SAFETY_RX_MS = HAL_GetTick();
      SAFETY_FRAME_VALID_FROM_B = 1u;
      STM_B_COMM_TIMEOUT = 0u;
      SAFETY_RX_COUNT++;
      taskEXIT_CRITICAL();
    }
    reset_safety_parser(byte);
  }
}

static void reset_host_command_state(void)
{
  taskENTER_CRITICAL();
  COMMAND_SPEED = 0.0f;
  s_command_received = 0u;
  s_command_timeout = 1u;
  s_command_brake_engage = 1u;
  s_last_command_tick = 0u;
  taskEXIT_CRITICAL();
}

static void init_ros_message_buffers(void)
{
  memset(&s_long_status_msg, 0, sizeof(s_long_status_msg));
  memset(&s_long_request_msg, 0, sizeof(s_long_request_msg));
  s_long_status_msg.data.data = s_long_status_data;
  s_long_status_msg.data.size = s_long_status_msg.data.capacity = LONG_STATUS_COUNT;
  s_long_request_msg.data.data = s_long_request_data;
  s_long_request_msg.data.capacity = LONG_REQUEST_COUNT;
  memset(&s_cmd_msg, 0, sizeof(s_cmd_msg));
  memset(&s_status_msg, 0, sizeof(s_status_msg));

  /* micro-ROS string 역직렬화는 동적 할당 대신 수명 고정 버퍼를 사용한다. */
  s_cmd_msg.header.frame_id.data = s_cmd_frame_id;
  s_cmd_msg.header.frame_id.capacity = sizeof(s_cmd_frame_id);
  s_status_msg.header.frame_id.data = s_status_frame_id;
  s_status_msg.header.frame_id.capacity = sizeof(s_status_frame_id);
}

static uint8_t create_ros_entities(void)
{
  s_ros_allocator = rcl_get_default_allocator();
  memset(&s_ros_support, 0, sizeof(s_ros_support));
  s_ros_node = rcl_get_zero_initialized_node();
  s_ros_sub_cmd = rcl_get_zero_initialized_subscription();
  s_ros_pub_drive_status = rcl_get_zero_initialized_publisher();
  s_ros_executor = rclc_executor_get_zero_initialized_executor();
  s_long_subscription = rcl_get_zero_initialized_subscription();
  s_long_publisher = rcl_get_zero_initialized_publisher();

  s_cmd_msg.header.frame_id.size = 0u;
  s_status_msg.header.frame_id.size = 0u;

  if (rclc_support_init(&s_ros_support, 0, NULL, &s_ros_allocator) != RCL_RET_OK) {
    return 0u;
  }
  s_ros_support_initialized = 1u;

  if (rclc_node_init_default(
          &s_ros_node, "stm_a_node", "", &s_ros_support) != RCL_RET_OK) {
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

  if (rclc_publisher_init_default(
          &s_ros_pub_drive_status, &s_ros_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs, msg, DriveActuatorStatus),
          "/drive/status") != RCL_RET_OK) {
    return 0u;
  }
  s_ros_publisher_initialized = 1u;

  if (rclc_subscription_init_default(&s_long_subscription, &s_ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/drive/longitudinal_config") != RCL_RET_OK) return 0u;
  s_long_subscription_initialized = 1u;
  if (rclc_publisher_init_best_effort(&s_long_publisher, &s_ros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/drive/longitudinal_status") != RCL_RET_OK) return 0u;
  s_long_publisher_initialized = 1u;

  if (rclc_executor_init(
          &s_ros_executor, &s_ros_support.context, 2, &s_ros_allocator) != RCL_RET_OK) {
    return 0u;
  }
  s_ros_executor_initialized = 1u;

  if (rclc_executor_add_subscription(
          &s_ros_executor, &s_ros_sub_cmd, &s_cmd_msg, &cb_cmd,
          ON_NEW_DATA) != RCL_RET_OK) {
    return 0u;
  }
  if (rclc_executor_add_subscription(&s_ros_executor, &s_long_subscription,
      &s_long_request_msg, &cb_long_config, ON_NEW_DATA) != RCL_RET_OK) return 0u;
  RuntimeStatsRos_Init(&s_runtime_ros, &s_ros_node,
      kaiev26_msgs__msg__RuntimeStats__BOARD_STM_A);
  return 1u;
}

static void destroy_ros_entities(void)
{
  /* Agent 단절 시 entity destroy가 긴 XRCE timeout으로 Control 안전 복귀를
   * 지연하지 않도록 best-effort, non-blocking cleanup으로 전환한다. */
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
  if (s_long_subscription_initialized) {
    const rcl_ret_t result = rcl_subscription_fini(&s_long_subscription, &s_ros_node);
    (void)result;
    s_long_subscription_initialized = 0u;
  }
  if (s_long_publisher_initialized) {
    const rcl_ret_t result = rcl_publisher_fini(&s_long_publisher, &s_ros_node);
    (void)result;
    s_long_publisher_initialized = 0u;
  }
  taskENTER_CRITICAL();
  s_long_pending_valid = 0u;
  taskEXIT_CRITICAL();
  if (s_ros_publisher_initialized != 0u) {
    const rcl_ret_t fini_result =
        rcl_publisher_fini(&s_ros_pub_drive_status, &s_ros_node);
    (void)fini_result;
    s_ros_publisher_initialized = 0u;
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

/* now는 호출자가 앞서 읽어둔 tick이고, CAN ISR과 uROS 명령 콜백은 그 뒤에도
 * 더 최신 tick을 기록한다. 그래서 stamp가 now보다 클 수 있고, 부호 없는
 * 뺄셈은 그때 ~2^32로 언더플로우해 갓 받은 프레임을 timeout으로 만든다.
 * snapshot보다 새 프레임의 age는 0이다. */
static uint32_t age_since(uint32_t now, uint32_t stamp)
{
  const int32_t delta = (int32_t)(now - stamp);
  return (delta < 0) ? 0u : (uint32_t)delta;
}

static void reset_motor_rpm_despiker(void)
{
  s_last_nonzero_motor_rpm = 0;
  s_motor_rpm_zero_streak = 0u;
  s_motor_rpm_hold_armed = 0u;
}

static int32_t despike_motor_rpm(int32_t motor_rpm_raw)
{
  if (motor_rpm_raw != 0) {
    s_last_nonzero_motor_rpm = motor_rpm_raw;
    s_motor_rpm_zero_streak = 0u;
    s_motor_rpm_hold_armed = 1u;
    return motor_rpm_raw;
  }

  if ((s_motor_rpm_hold_armed != 0u) &&
      (s_motor_rpm_zero_streak < MOTOR_RPM_ZERO_HOLD_SAMPLES)) {
    s_motor_rpm_zero_streak++;
    return s_last_nonzero_motor_rpm;
  }

  reset_motor_rpm_despiker();
  return 0;
}

static int16_t encode_tenths_i16(float value)
{
  float scaled = value * 10.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)lroundf(scaled);
}

static uint16_t encode_tenths_u16(float value)
{
  float scaled = value * 10.0f;
  if (scaled > 65535.0f) scaled = 65535.0f;
  if (scaled < 0.0f) scaled = 0.0f;
  return (uint16_t)lroundf(scaled);
}

static void capture_drive_detail(uint32_t now,
                                 BoardUartDriveDetail_t *detail)
{
  uint32_t part1_rx_ms;
  uint32_t part2_rx_ms;
  uint8_t part1_received;
  uint8_t part2_received;
  uint8_t tractive_commanded;
  uint8_t controller_running;
  uint8_t command_received;
  uint8_t command_timeout;
  uint8_t drive_stop_request;
  float bus_voltage_v;
  float bus_current_a;
  float phase_current_a;
  float controller_temperature_c;
  float motor_temperature_c;
  uint32_t motor_fault_bits;

  taskENTER_CRITICAL();
  detail->pedal_percent = PEDAL;
  part1_rx_ms = s_last_part1_rx_ms;
  part2_rx_ms = s_last_part2_rx_ms;
  part1_received = s_part1_received;
  part2_received = s_part2_received;
  tractive_commanded = s_tractive_commanded;
  controller_running = s_controller_running;
  command_received = s_command_received;
  command_timeout = s_command_timeout;
  drive_stop_request = DRIVE_STOP_REQUEST;
  bus_voltage_v = s_bus_voltage_v;
  bus_current_a = s_bus_current_a;
  phase_current_a = s_phase_current_a;
  controller_temperature_c = s_controller_temperature_c;
  motor_temperature_c = s_motor_temperature_c;
  motor_fault_bits = s_motor_fault_bits;
  taskEXIT_CRITICAL();

  const uint8_t part1_valid =
      ((part1_received != 0u) &&
       (age_since(now, part1_rx_ms) < MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1u : 0u;
  const uint8_t part2_valid =
      ((part2_received != 0u) &&
       (age_since(now, part2_rx_ms) < MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1u : 0u;

  detail->flags = 0u;
  if (part1_valid != 0u) {
    detail->flags |= BOARD_UART_DETAIL_PART1_VALID;
  }
  if (part2_valid != 0u) {
    detail->flags |= BOARD_UART_DETAIL_PART2_VALID;
  }
  if (tractive_commanded != 0u) {
    detail->flags |= BOARD_UART_DETAIL_TRACTIVE_COMMANDED;
  }
  if (controller_running != 0u) {
    detail->flags |= BOARD_UART_DETAIL_TRACTIVE_FEEDBACK;
  }
  if (command_received != 0u) {
    detail->flags |= BOARD_UART_DETAIL_COMMAND_RECEIVED;
  }
  if (command_timeout != 0u) {
    detail->flags |= BOARD_UART_DETAIL_COMMAND_TIMEOUT;
  }
  if (drive_stop_request != 0u) {
    detail->flags |= BOARD_UART_DETAIL_DRIVE_STOP;
  }

  detail->bus_voltage_dv = encode_tenths_u16(bus_voltage_v);
  detail->bus_current_da = encode_tenths_i16(bus_current_a);
  detail->phase_current_da = encode_tenths_i16(phase_current_a);
  detail->controller_temperature_dc =
      encode_tenths_i16(controller_temperature_c);
  detail->motor_temperature_dc = encode_tenths_i16(motor_temperature_c);
  detail->motor_fault_bits = motor_fault_bits;
}

static void publish_drive_status(uint32_t now)
{
  uint8_t pedal_percent;
  int32_t pedal_adc_raw;
  float commanded_phase_current_a;
  float commanded_motor_rpm;
  float motor_rpm;
  float speed_mps;
  float bus_voltage_v;
  float bus_current_a;
  float phase_current_a;
  float controller_temperature_c;
  float motor_temperature_c;
  uint32_t fault_bits;
  uint32_t part1_rx_ms;
  uint32_t part2_rx_ms;
  uint32_t command_rx_ms;
  uint8_t controller_running;
  uint8_t controller_control_mode;
  uint8_t part1_received;
  uint8_t part2_received;
  uint8_t command_received;
  uint8_t command_timeout;
  uint8_t tractive_commanded;

  /* CAN ISR와 Control task가 갱신하는 관련 필드를 한 시점의 snapshot으로 읽는다. */
  taskENTER_CRITICAL();
  pedal_percent = PEDAL;
  pedal_adc_raw = s_pedal_adc_raw;
  commanded_phase_current_a = s_commanded_phase_current_a;
  commanded_motor_rpm = s_commanded_motor_rpm;
  motor_rpm = s_motor_rpm_raw;
  speed_mps = CURRENT_SPEED;
  bus_voltage_v = s_bus_voltage_v;
  bus_current_a = s_bus_current_a;
  phase_current_a = s_phase_current_a;
  controller_temperature_c = s_controller_temperature_c;
  motor_temperature_c = s_motor_temperature_c;
  fault_bits = s_motor_fault_bits;
  controller_running = s_controller_running;
  controller_control_mode = s_controller_control_mode;
  part1_rx_ms = s_last_part1_rx_ms;
  part2_rx_ms = s_last_part2_rx_ms;
  part1_received = s_part1_received;
  part2_received = s_part2_received;
  command_rx_ms = s_last_command_tick;
  command_received = s_command_received;
  command_timeout = s_command_timeout;
  tractive_commanded = s_tractive_commanded;
  taskEXIT_CRITICAL();

  const uint32_t part1_age_ms = age_since(now, part1_rx_ms);
  const uint32_t part2_age_ms = age_since(now, part2_rx_ms);
  const uint8_t part1_valid =
      ((part1_received != 0u) &&
       (part1_age_ms < MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1u : 0u;
  const uint8_t part2_valid =
      ((part2_received != 0u) &&
       (part2_age_ms < MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1u : 0u;

  s_status_msg.mode = MODE;
  s_status_msg.gear = GEAR;
  s_status_msg.pedal_percent = pedal_percent;
  s_status_msg.pedal_adc_raw = pedal_adc_raw;
  s_status_msg.speed_mps = speed_mps;
  s_status_msg.motor_rpm = motor_rpm;
  s_status_msg.tractive_commanded = (tractive_commanded != 0u);
  s_status_msg.tractive_feedback = (controller_running != 0u);
  s_status_msg.tractive_feedback_valid = (part2_valid != 0u);
  s_status_msg.stop_request = (DRIVE_STOP_REQUEST != 0u);
  s_status_msg.fault_bits = fault_bits;
  s_status_msg.fault_bits_valid = (part2_valid != 0u);
  s_status_msg.command_received = (command_received != 0u);
  s_status_msg.command_timeout = (command_timeout != 0u);
  s_status_msg.command_age_ms = (command_received != 0u) ?
      (float)age_since(now, command_rx_ms) : INFINITY;
  s_status_msg.commanded_phase_current_a = commanded_phase_current_a;
  s_status_msg.commanded_motor_rpm = commanded_motor_rpm;
  s_status_msg.bus_voltage_v = bus_voltage_v;
  s_status_msg.bus_current_a = bus_current_a;
  s_status_msg.phase_current_a = phase_current_a;
  s_status_msg.part1_valid = (part1_valid != 0u);
  s_status_msg.part1_age_ms = (part1_received != 0u) ?
      (float)part1_age_ms : INFINITY;
  s_status_msg.controller_temperature_c = controller_temperature_c;
  s_status_msg.motor_temperature_c = motor_temperature_c;
  s_status_msg.controller_control_mode = controller_control_mode;
  s_status_msg.part2_valid = (part2_valid != 0u);
  s_status_msg.part2_age_ms = (part2_received != 0u) ?
      (float)part2_age_ms : INFINITY;
  RCL_PUBLISH_BEST_EFFORT(&s_ros_pub_drive_status, &s_status_msg);
}

static void send_motor_can(int32_t rpm, float curr_a, uint8_t run_command,
                           uint8_t speed_control)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t d[8];
  const int32_t clamped_rpm = EZkontrol_ClampRpm(rpm, RPM_FWD_MAX, RPM_REV_MAX);

  hdr.Identifier = 0x0C01EFD0UL;
  hdr.IdType = FDCAN_EXTENDED_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  EZkontrol_EncodeCommand(clamped_rpm, curr_a, run_command, speed_control, s_life++, d);
  const uint16_t current_encoded = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, d) == HAL_OK) {
    s_tractive_commanded = (run_command != 0u) ? 1u : 0u;
    /* 인코딩을 되돌려 저장한다. 양자화와 클램핑이 반영된 값이라
     * 의도한 값이 아니라 실제로 나간 값이 남는다. */
    s_commanded_phase_current_a = (float)current_encoded * 0.1f - 3200.0f;
    s_commanded_motor_rpm = (float)clamped_rpm;
  } else {
    s_tractive_commanded = 0u;
  }
}

/* EZkontrol 핸드셰이크 응답: 모터드라이버가 0x1801D0EF로 8byte 0x55(핸드셰이크 대기)를
 * 보내는 동안, 0x0C01EFD0으로 8byte 0xAA를 보내야 드라이버가 이후 속도/전류 명령을 실행한다.
 * (EZkontrol CANBUS MCU to VCU V1.0, section 4) */
static HAL_StatusTypeDef send_motor_handshake_ack(void)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t d[8];
  for (uint8_t i = 0u; i < 8u; ++i) d[i] = 0xAAu;

  hdr.Identifier = 0x0C01EFD0UL;
  hdr.IdType = FDCAN_EXTENDED_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, d);
}

/* uROS owns the mailbox write; Control owns config/history. The same critical
 * section protects readback. No flash writes and no auto-replay on reconnect. */
static void cb_long_config(const void *message)
{
  const std_msgs__msg__Float32MultiArray *msg = message;
  LongRequest request;
  if (msg->layout.dim.size != 0u || msg->layout.data_offset != 0u ||
      !Long_DecodeRequest(msg->data.data, msg->data.size, &request)) {
    taskENTER_CRITICAL();
    s_long_calibration.result = LONG_INVALID;
    s_long_calibration.request_id = 0u;
    taskEXIT_CRITICAL();
    return;
  }
  taskENTER_CRITICAL();
  if (s_long_pending_valid) {
    s_long_calibration.result = LONG_BUSY;
    s_long_calibration.request_id = request.id;
  } else if (request.id != s_long_calibration.request_id || s_long_calibration.result != LONG_APPLIED) {
    s_long_pending = request;
    s_long_pending_valid = 1u;
  }
  taskEXIT_CRITICAL();
}

static void prepare_longitudinal(uint32_t now)
{
  taskENTER_CRITICAL();
  s_long_feedback_valid = s_part1_received && s_part2_received &&
      age_since(now, s_last_part1_rx_ms) < MOTOR_FEEDBACK_TIMEOUT_MS &&
      age_since(now, s_last_part2_rx_ms) < MOTOR_FEEDBACK_TIMEOUT_MS;
  if (s_long_pending_valid) {
    s_long_pending_valid = 0u;
    Long_ApplyConfiguration(&s_long_calibration, &s_long_pending,
        Long_CanConfigure(MODE == 0u, s_long_feedback_valid,
          CURRENT_SPEED, PEDAL, s_commanded_phase_current_a, s_phase_current_a));
    if (s_long_calibration.result == LONG_APPLIED) Long_Reset(&s_long_controller);
  }
  bool discard_command = false;
  s_long_inhibit = Long_CheckInhibit(&s_long_controller, &s_long_gate,
      MODE == 1u && s_long_calibration.config.mode == LONG_TORQUE,
      s_long_feedback_valid,
      s_command_received && age_since(now, s_last_command_tick) < HOST_COMMAND_TIMEOUT_MS,
      DRIVE_STOP_REQUEST != 0u || s_command_brake_engage != 0u,
      COMMAND_SPEED, CURRENT_SPEED, &discard_command);
  if (discard_command) {
    s_command_received = 0u;
    COMMAND_SPEED = 0.0f;
  }
  taskEXIT_CRITICAL();
}

static void publish_longitudinal(void)
{
  taskENTER_CRITICAL();
  float *d = s_long_status_data;
  d[0] = LONG_VERSION; d[1] = s_long_calibration.revision; d[2] = s_long_calibration.request_id;
  d[3] = s_long_calibration.result; d[4] = s_long_calibration.source;
  Long_EncodeConfig(&s_long_calibration.config, d + 5);
  d[21] = s_long_controller.error; d[22] = s_long_controller.p;
  d[23] = s_long_controller.i; d[24] = s_long_controller.ff;
  d[25] = s_long_controller.unsaturated; d[26] = s_long_controller.current;
  d[27] = s_long_controller.saturated; d[28] = s_long_controller.anti_windup;
  d[29] = s_long_controller.coasting; d[30] = s_long_feedback_valid;
  d[31] = s_long_inhibit;
  taskEXIT_CRITICAL();
  RCL_PUBLISH_BEST_EFFORT(&s_long_publisher, &s_long_status_msg);
}

static void cb_cmd(const void *message)
{
  const kaiev26_msgs__msg__ActuatorCommand *cmd =
      (const kaiev26_msgs__msg__ActuatorCommand *)message;
  const uint8_t brake_requested = cmd->brake_engage ? 1u : 0u;
  if ((brake_requested == 0u) &&
      !isfinite(cmd->speed_target_mps)) return;

  static int64_t last_brake_stamp_ns = 0;
  if (!CommandBrakeOrder_Accept(&last_brake_stamp_ns, brake_requested,
      cmd->header.stamp.sec, cmd->header.stamp.nanosec)) return;
  const uint32_t now = HAL_GetTick();
  taskENTER_CRITICAL();
  /* AUTO 속도 명령의 부호는 Host가 정하고, CAN 송신 경계가 전진/후진
   * 회전수 한계를 적용한다. MANUAL 방향만 물리 기어 스위치가 정한다. */
  COMMAND_SPEED = (brake_requested != 0u) ? 0.0f :
      fmaxf(-AUTO_SPEED_MAX_MPS,
            fminf(cmd->speed_target_mps, AUTO_SPEED_MAX_MPS));
  s_command_brake_engage = brake_requested;
  s_command_received = 1u;
  s_command_timeout = 0u;
  s_last_command_tick = now;
  taskEXIT_CRITICAL();
}

static void parse_motor_status_part1(const uint8_t data[8], uint32_t now)
{
  /* EZkontrol CANBUS MCU→VCU Part I, little-endian:
   * BUS voltage 0.1 V/bit; BUS/phase current 0.1 A/bit, -3200 A offset;
   * motor speed 1 rpm/bit, -32000 rpm offset. */
  const uint16_t raw_voltage =
      (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
  const uint16_t raw_bus_current =
      (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
  const uint16_t raw_phase_current =
      (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
  const uint16_t raw_rpm =
      (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
  const int32_t motor_rpm_raw = (int32_t)raw_rpm - 32000;

  /* 새 CAN stream에서는 이전 stream의 RPM을 hold하지 않는다. */
  if ((s_part1_received == 0u) ||
      (age_since(now, s_last_part1_rx_ms) >= MOTOR_FEEDBACK_TIMEOUT_MS)) {
    reset_motor_rpm_despiker();
  }
  const int32_t motor_rpm_despiked = despike_motor_rpm(motor_rpm_raw);
  const float wheel_rpm = (float)motor_rpm_despiked / GEAR_RATIO;

  s_bus_voltage_v = (float)raw_voltage * 0.1f;
  s_bus_current_a = (float)raw_bus_current * 0.1f - 3200.0f;
  s_phase_current_a = (float)raw_phase_current * 0.1f - 3200.0f;
  s_motor_rpm_raw = (float)motor_rpm_raw;
  CURRENT_SPEED = wheel_rpm * (3.14159f * TIRE_DIAM_M) / 60.0f;
  s_last_part1_rx_ms = now;
  s_part1_received = 1u;
}

static void parse_motor_status_part2(const uint8_t data[8], uint32_t now)
{
  /* EZkontrol Part II STATUS: bit0 RUN, bit1 0=torque/1=speed.
   * ERROR bytes 3..5 retain the manual's original 24-bit bit ordering:
   * 0 overcurrent, 1 overload, 2 overvoltage, 3 undervoltage,
   * 4 controller overheat, 5 motor overheat, 6 stalled, 7 out-of-phase,
   * 8 motor sensor, 9 AUX sensor, 10 encoder, 11 anti-runaway,
   * 12 main accelerator, 13 AUX accelerator, 14 pre-charge, 15 contactor,
   * 16 power valve, 17 current sensor, 18 auto-tune, 19 RS485,
   * 20 CAN, 21 software, 22..23 reserved. */
  s_controller_temperature_c = (float)data[0] - 40.0f;
  s_motor_temperature_c = (float)data[1] - 40.0f;
  s_controller_running = ((data[2] & 0x01u) != 0u) ? 1u : 0u;
  s_controller_control_mode = ((data[2] & 0x02u) != 0u) ?
      CONTROLLER_CONTROL_MODE_SPEED : CONTROLLER_CONTROL_MODE_TORQUE;
  s_motor_fault_bits =
      ((uint32_t)data[3]) |
      ((uint32_t)data[4] << 8) |
      ((uint32_t)data[5] << 16);
  s_last_part2_rx_ms = now;
  s_part2_received = 1u;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  (void)RxFifo0ITs;

  /* 한 IRQ에서 FIFO0에 쌓인 status frame을 모두 비운다. */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
    if (HAL_FDCAN_GetRxMessage(
            hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
      break;
    }
    if ((rx_header.IdType != FDCAN_EXTENDED_ID) ||
        (rx_header.DataLength != FDCAN_DLC_BYTES_8)) {
      continue;
    }

    const uint32_t now = HAL_GetTick();
    if (rx_header.Identifier == MOTOR_STATUS_PART1_ID) {
      /* 드라이버가 핸드셰이크 대기 중이면 8byte 전부 0x55로 온다. */
      uint8_t is_handshake_request = 1u;
      for (uint8_t i = 0u; i < 8u; ++i) {
        if (rx_data[i] != 0x55u) { is_handshake_request = 0u; break; }
      }
      if (is_handshake_request != 0u) {
        reset_motor_rpm_despiker();
        s_motor_handshake_done = 0u;
        if (send_motor_handshake_ack() == HAL_OK) {
          s_motor_handshake_done = 1u;
          s_motor_handshake_count++;
        }
        continue;
      }
      parse_motor_status_part1(rx_data, now);
    } else if (rx_header.Identifier == MOTOR_STATUS_PART2_ID) {
      parse_motor_status_part2(rx_data, now);
    }
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

