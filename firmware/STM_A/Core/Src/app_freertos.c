/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : STM_A FreeRTOS 태스크 전체 구현
  *                      베이스(uROS+UART 동작 확인) 구조 유지,
  *                      Control / Inputs / MotorRx 로직 추가
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
#include <stdlib.h>
#include "globals.h"
#include "usart.h"
#include "adc.h"
#include "fdcan.h"
#include "gpio.h"

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
  uint8_t data[16];
  uint8_t index;
  uint8_t expected_size;
} uart_frame_parser_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIRE_DIAM_M   0.47f
#define GEAR_RATIO    4.60f
#define RPM_FWD_MAX   4500
#define RPM_REV_MAX   1500
#define AUTO_SPEED_MAX_MPS \
  ((float)RPM_FWD_MAX * 3.14159f * TIRE_DIAM_M / (60.0f * GEAR_RATIO))
#define AUTO_PHASE_LIMIT_A 300.0f
#define MANUAL_TORQUE_LIMIT_A 175.0f
#define PEDAL_ADC_MIN 560
#define PEDAL_ADC_MAX 3100
#define UART_SOF1 0xAAu
#define UART_SOF2 0x55u
#define UART_MSG_A_TO_B_VEHICLE_STATUS 0x01u
#define UART_MSG_B_TO_A_SAFETY_STATUS  0x02u
#define UART_VEHICLE_PAYLOAD_LEN 3u
#define UART_SAFETY_PAYLOAD_LEN  1u
#define UART_RX_DMA_SIZE 64u
#define UART_RX_RING_SIZE 128u
#define STM_B_COMM_TIMEOUT_MS 100u
#define VEHICLE_TX_PERIOD_MS 20u
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
/* TEMP DEBUG: 클램핑 전 raw ADC 값 저장 (StartInputs가 쓰고, StartuROS가 읽어서 발행) */
static volatile int32_t s_pedal_raw_dbg = 0;
static volatile float s_motor_rpm = 0.0f;
static volatile uint8_t s_command_received = 0u;
static volatile uint32_t s_last_command_tick = 0u;
static volatile uint8_t s_motor_handshake_done = 0u;
static volatile uint32_t s_motor_handshake_count = 0u;
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
static void send_motor_can(int32_t rpm, float curr_a, uint8_t run_command,
                           uint8_t speed_control);
// static void send_motor_handshake_ack(void); 7.15 handshake 추가 
static void cb_cmd(const void *message);
static uint8_t uart_crc8(const uint8_t *data, uint8_t length);
static void usart1_rx_ring_push(uint8_t byte);
static uint8_t usart1_rx_ring_pop(uint8_t *byte);
static void reset_safety_parser(uint8_t last_byte);
static void process_safety_rx_byte(uint8_t byte);
static HAL_StatusTypeDef restart_usart1_rx(UART_HandleTypeDef *huart);
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

  /* FDCAN 필터 설정 -> 0x1801D0EF만 FIFO0으로 받도록 설정 */
  FDCAN_FilterTypeDef f;
  f.IdType       = FDCAN_EXTENDED_ID;
  f.FilterIndex  = 0;
  f.FilterType   = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = 0x1801D0EFUL;
  f.FilterID2    = 0x1FFFFFFFUL;  // 마스크 전체 비트 비교
  HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
  HAL_FDCAN_Start(&hfdcan1);
  // 수신 인터럽트 ON
  HAL_FDCAN_ActivateNotification(&hfdcan1,
                                  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  for (;;)
  {
    if (s_motor_handshake_done == 0u)
    {
      osDelay(10);
      continue;
    }

    if (DRIVE_STOP_REQUEST == 0u)
    {
      if (MODE == 1u)
      {
        /* 자율주행: PC 속도 명령을 Speed control로 전달한다. */
        float wrpm = COMMAND_SPEED * 60.0f / (3.14159f * TIRE_DIAM_M);
        int32_t mrpm = (int32_t)(wrpm * GEAR_RATIO);
        if (GEAR == 1u) mrpm = -mrpm;
        send_motor_can(mrpm, AUTO_PHASE_LIMIT_A, 1u, 1u);
      }
      else
      {
        /* 수동주행: 페달 0~100%를 설정 최대 상전류(350A)의 절반까지 매핑한다. */
        float torque_current = ((float)PEDAL / 100.0f) * MANUAL_TORQUE_LIMIT_A;
        int16_t rpm_limit = RPM_FWD_MAX;
        if (GEAR == 1u)
        {
          torque_current = -torque_current;
          rpm_limit = -RPM_REV_MAX;
        }
        send_motor_can(rpm_limit, torque_current, 1u, 0u);
      }
    }
    // ESTOP 활성시
    else
    {
      /* 정지 시 RPM/전류뿐 아니라 RUNNING bit도 내려 HALTED를 명령한다. */
      send_motor_can(0, 0.0f, 0u, (MODE == 1u) ? 1u : 0u);
    }
    // 100Hz 주기
    osDelay(10);
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
  for (;;)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN); // TASK 먹통 확인 위한 LED 디버깅

    /* 페달 ADC 8회 평균 */
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 10);  // 변환 완료 대기(최대 10ms)
      sum += HAL_ADC_GetValue(&hadc1);
    }
    int raw = (int)(sum / 8u);
    // TEMP DEBUG: 클램핑 전 raw 값을 그대로 저장 (디버깅 끝나면 이 줄 삭제)
    s_pedal_raw_dbg = (int32_t)raw;
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

    osDelay(50); // LED 확인 위해 주기 느리게 했음. 
  }
  /* USER CODE END StartInputs */
}

/* USER CODE BEGIN Header_StartMotorRx */
/**
* @brief Function implementing the MotorRx thread.
* @param argument: Not used
* @retval None
*/

/* ========================================================== */
/* Task: MotorRx — Rx는 콜백 처리, 태스크 최소화 (osPriorityHigh2) */
/* 수신은 콜백이 하므로 대기만 수행*/
/* ========================================================== */

/* USER CODE END Header_StartMotorRx */
void StartMotorRx(void *argument)
{
  /* USER CODE BEGIN StartMotorRx */
  for (;;) { osDelay(100); }
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

  /* ============================================ */
  /* Task: uROS — PC 양방향 통신 (osPriorityNormal) */
  /* ============================================ */
  /* 구독 메시지 저장 버퍼. micro-ROS 구독 메시지의 string 필드에는
   * 정적 버퍼를 미리 할당해야 역직렬화가 가능하다. */
  static kaiev26_msgs__msg__ActuatorCommand s_cmd_msg;
  static char s_cmd_frame_id[16];
  s_cmd_msg.header.frame_id.data = s_cmd_frame_id;
  s_cmd_msg.header.frame_id.size = 0u;
  s_cmd_msg.header.frame_id.capacity = sizeof(s_cmd_frame_id);

  extern UART_HandleTypeDef hlpuart1; // uROS UART

  // uROS가 LPUAR1으로 통신하도록 transport 등록
  rmw_uros_set_custom_transport(
      true, (void *)&hlpuart1,
      cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read);

  // 메모리 할당자
  rcl_allocator_t alloc = rcl_get_default_allocator();
  // 지원 구조체 (???)
  rclc_support_t  sup;
  /// ROS2 노드
  rcl_node_t      node;

  // Subscriber 생성
  rcl_subscription_t sub_cmd;
  // Publisher 생성: 통합 /drive/status 하나가 개별 상태 토픽들을 대체한다
  rcl_publisher_t pub_drive_status;
  
  // PC와 연결될 때까지 0.5초 간격으로 ping (???)
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
    osDelay(500);
  
  // 노드 생성 순서 바껴야된다는데 ? 반드시 node 생성 first 이후 support sup 이 빈 상태에서 노드 생성 불가 
  // rclc_support_init(&sup, 0, NULL, &alloc);
  // rclc_node_init_default(&node, "stm_a_node", "", &sup);

  // ✨ [수정] 올바른 순서: Support 초기화 -> Node 초기화
  if (rclc_support_init(&sup, 0, NULL, &alloc) != RCL_RET_OK) {
  // 여기에 걸리면 초기화 실패 (디버깅용 브레이크포인트 지정 추천)
    for(;;); 
  }
  
  if (rclc_node_init_default(&node, "stm_a_node", "", &sup) != RCL_RET_OK) {
    for(;;);
  }

  // Decision의 최종 차량 명령 하나만 구독한다.
  rclc_subscription_init_default(&sub_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs, msg, ActuatorCommand),
      "/vehicle/command");

  // Publisher 초기화
  rclc_publisher_init_default(&pub_drive_status, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs, msg, DriveActuatorStatus),
      "/drive/status");

  // Subscriber 콜백 관리
  rclc_executor_t exec;
  rclc_executor_init(&exec, &sup.context, 1, &alloc);
  rclc_executor_add_subscription(&exec, &sub_cmd,
      &s_cmd_msg, &cb_cmd, ON_NEW_DATA);

  static kaiev26_msgs__msg__DriveActuatorStatus s_status_msg;
  static char s_status_frame_id[4];
  s_status_msg.header.frame_id.data = s_status_frame_id;
  s_status_msg.header.frame_id.size = 0u;
  s_status_msg.header.frame_id.capacity = sizeof(s_status_frame_id);

  uint32_t now = HAL_GetTick();
  uint32_t last_status = now - 50u;

  for (;;)
  {
    rclc_executor_spin_some(&exec, RCL_MS_TO_NS(1));
    now = HAL_GetTick();

    if ((uint32_t)(now - last_status) >= 50u) {
      s_status_msg.mode = MODE;
      s_status_msg.gear = GEAR;
      s_status_msg.speed_mps = CURRENT_SPEED;
      s_status_msg.motor_rpm = s_motor_rpm;
      s_status_msg.tractive_commanded = (DRIVE_STOP_REQUEST == 0u);
      /* 드라이버 RUN feedback은 아직 CAN에서 읽지 않으므로 valid=false */
      s_status_msg.tractive_feedback = false;
      s_status_msg.tractive_feedback_valid = false;
      s_status_msg.stop_request = (DRIVE_STOP_REQUEST != 0u);
      s_status_msg.fault_bits = 0u;
      s_status_msg.fault_bits_valid = false;
      s_status_msg.command_received = (s_command_received != 0u);
      s_status_msg.command_age_ms = (s_command_received != 0u) ?
          (float)((uint32_t)(now - s_last_command_tick)) : 0.0f;
      RCL_PUBLISH_BEST_EFFORT(&pub_drive_status, &s_status_msg);
      last_status += 50u;
    }

    osDelay(5);
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
    uint8_t byte;
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

    DRIVE_STOP_REQUEST = ((STOP_REQUEST_FROM_B != 0u) ||
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
  uint8_t sequence = 0u;

  for (;;)
  {
    uint8_t frame[9];
    uint8_t status = 0u;
    float speed_x100_f = CURRENT_SPEED * 100.0f;
    if (speed_x100_f > 32767.0f) speed_x100_f = 32767.0f;
    if (speed_x100_f < -32768.0f) speed_x100_f = -32768.0f;
    int16_t speed_x100 = (int16_t)lroundf(speed_x100_f);

    status |= (uint8_t)((MODE != 0u) ? (1u << 0) : 0u);
    status |= (uint8_t)((GEAR != 0u) ? (1u << 1) : 0u);

    frame[0] = UART_SOF1;
    frame[1] = UART_SOF2;
    frame[2] = UART_MSG_A_TO_B_VEHICLE_STATUS;
    frame[3] = UART_VEHICLE_PAYLOAD_LEN;
    frame[4] = sequence++;
    frame[5] = status;
    frame[6] = (uint8_t)((uint16_t)speed_x100 & 0xFFu);
    frame[7] = (uint8_t)(((uint16_t)speed_x100 >> 8) & 0xFFu);
    frame[8] = uart_crc8(&frame[2], 6u);

    (void)HAL_UART_Transmit(&huart1, frame, sizeof(frame), 10u);
    osDelay(VEHICLE_TX_PERIOD_MS);
  }
  /* USER CODE END StartBTx */
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
  reset_safety_parser(0u);
  taskEXIT_CRITICAL();
  return HAL_UARTEx_ReceiveToIdle_DMA(huart, s_usart1_rx_dma,
                                     UART_RX_DMA_SIZE);
}

static void reset_safety_parser(uint8_t last_byte)
{
  s_safety_parser.index = 0u;
  s_safety_parser.expected_size = 0u;
  if (last_byte == UART_SOF1) {
    s_safety_parser.data[0] = UART_SOF1;
    s_safety_parser.index = 1u;
  }
}

static void process_safety_rx_byte(uint8_t byte)
{
  uart_frame_parser_t *parser = &s_safety_parser;

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
    if ((parser->data[2] != UART_MSG_B_TO_A_SAFETY_STATUS) ||
        (parser->data[3] != UART_SAFETY_PAYLOAD_LEN)) {
      reset_safety_parser(byte);
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
      STOP_REQUEST_FROM_B = parser->data[5] & 0x01u;
      LAST_SAFETY_RX_MS = HAL_GetTick();
      SAFETY_FRAME_VALID_FROM_B = 1u;
      STM_B_COMM_TIMEOUT = 0u;
      SAFETY_RX_COUNT++;
      taskEXIT_CRITICAL();
    }
    reset_safety_parser(byte);
  }
}

static void send_motor_can(int32_t rpm, float curr_a, uint8_t run_command,
                           uint8_t speed_control)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t d[8];
  double raw_current = (curr_a + 3200.0) / 0.1;
  uint16_t current_encoded = (uint16_t)fmax(0.0, fmin(65535.0, raw_current));
  int32_t clamped_rpm = rpm >= 0
                          ? (int32_t)fmin((float)rpm, (float)RPM_FWD_MAX)
                          : (int32_t)fmax((float)rpm, (float)-RPM_REV_MAX);
  uint16_t speed_encoded = (uint16_t)(clamped_rpm + 32000);

  hdr.Identifier = 0x0C01EFD0UL;
  hdr.IdType = FDCAN_EXTENDED_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  d[0] = (uint8_t)(current_encoded & 0xFFu);
  d[1] = (uint8_t)(current_encoded >> 8);
  d[2] = (uint8_t)(speed_encoded & 0xFFu);
  d[3] = (uint8_t)(speed_encoded >> 8);
  /* bit0: RUNNING, bit1: speed control(0이면 torque control). */
  d[4] = (uint8_t)(((run_command != 0u) ? 0x01u : 0x00u) |
                   ((speed_control != 0u) ? 0x02u : 0x00u));
  d[5] = 0x00u;
  d[6] = 0x00u;
  d[7] = s_life++;

  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, d);
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

static void cb_cmd(const void *message)
{
  const kaiev26_msgs__msg__ActuatorCommand *cmd =
      (const kaiev26_msgs__msg__ActuatorCommand *)message;
  /* 명령 경계에서 물리적으로 가능한 전진 속도 범위로 한 번만 제한한다. */
  if (cmd->brake_engage || !isfinite(cmd->speed_target_mps)) {
    COMMAND_SPEED = 0.0f;
  } else {
    COMMAND_SPEED = fmaxf(0.0f,
        fminf(cmd->speed_target_mps, AUTO_SPEED_MAX_MPS));
  }
  s_command_received = 1u;
  s_last_command_tick = HAL_GetTick();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  (void)RxFifo0ITs;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    if (rx_header.Identifier == 0x1801D0EFUL) {
      /* 드라이버가 핸드셰이크 대기 중이면 8byte 전부 0x55로 온다. */
      uint8_t is_handshake_request = 1u;
      for (uint8_t i = 0u; i < 8u; ++i) {
        if (rx_data[i] != 0x55u) { is_handshake_request = 0u; break; }
      }
      if (is_handshake_request != 0u) {
        s_motor_handshake_done = 0u;
        if (send_motor_handshake_ack() == HAL_OK) {
          s_motor_handshake_done = 1u;
          s_motor_handshake_count++;
        }
        return;
      }

      uint16_t raw_rpm = (uint16_t)(rx_data[6] | ((uint16_t)rx_data[7] << 8));
      int32_t rpm = (int32_t)raw_rpm - 32000;
      float wheel_rpm = (float)rpm / GEAR_RATIO;
      s_motor_rpm = (float)rpm;
      CURRENT_SPEED = wheel_rpm * (3.14159f * TIRE_DIAM_M) / 60.0f;
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
