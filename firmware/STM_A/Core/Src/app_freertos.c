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
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int32.h>
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
#define TIRE_DIAM_M   0.47f
#define GEAR_RATIO    4.60f
#define RPM_FWD_MAX   4500
#define RPM_REV_MAX   1500
#define PHASE_LIMIT_A 300.0f
#define PEDAL_ADC_MIN 546
#define PEDAL_ADC_MAX 2730
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* BRx DMA 수신 버퍼 (4바이트 프레임) */
static uint8_t s_brx_buf[4];
static uint8_t s_life = 0;
static uint8_t s_safety_frame_received = 0;
/* TEMP DEBUG: 클램핑 전 raw ADC 값 저장 (StartInputs가 쓰고, StartuROS가 읽어서 발행) */
static volatile int32_t s_pedal_raw_dbg = 0;
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
static void send_motor_can(int16_t rpm, float curr_a, uint8_t run_command);
static void cb_cmd(const void *message);
static void cb_aeb(const void *message);
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
    if (DRIVE_STOP_REQUEST == 0u)
    {
      // 자율주행 : PC cmd, 수동주행 : 페달비율 (최대 3m/s)
      float spd = (MODE == 1) ? COMMAND_SPEED : ((float)PEDAL / 100.0f * 3.0f);
      // 속도 -> 모터 RPM 변환                          
      float wrpm   = spd * 60.0f / (3.14159f * TIRE_DIAM_M);
      int16_t mrpm = (int16_t)(wrpm * GEAR_RATIO);
      // 후진이면 음수 RPM
      if (GEAR == 1) mrpm = (int16_t)-abs(mrpm);
      // 모터 송신
      send_motor_can(mrpm, PHASE_LIMIT_A, 1u);
    }
    // ESTOP 활성시
    else
    {
      /* 정지 시 RPM/전류뿐 아니라 RUNNING bit도 내려 HALTED를 명령한다. */
      send_motor_can(0, 0.0f, 0u);
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
    MODE = (HAL_GPIO_ReadPin(SW_MODE_GPIO_Port, SW_MODE_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    GEAR = (HAL_GPIO_ReadPin(SW_GEAR_GPIO_Port, SW_GEAR_Pin) == GPIO_PIN_SET) ? 1u : 0u;

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
  /* 구독 메시지 저장 버퍼*/
  static std_msgs__msg__Float32 s_cmd_msg;
  static std_msgs__msg__UInt8   s_aeb_msg;

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
  rcl_subscription_t sub_cmd, sub_aeb;
  // Publisher 생성
  rcl_publisher_t    pub_spd, pub_estop, pub_gear, pub_mode;
  rcl_publisher_t    pub_dbg_pedal;       // 변환된 페달값
  rcl_publisher_t    pub_dbg_pedal_raw;   // 변환전 페달값
  
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

  // Subscriber 2개 초기화
  rclc_subscription_init_default(&sub_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/cmd/speed");
  rclc_subscription_init_default(&sub_aeb, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/aeb");

  // Publisher 초기화 (CURRENT_SPEED / ESTOP / GEAR / MODE)
  rclc_publisher_init_default(&pub_spd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current/speed");
  rclc_publisher_init_default(&pub_estop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/estop/status");
  rclc_publisher_init_default(&pub_gear, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/gear");
  rclc_publisher_init_default(&pub_mode, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/mode");
  // TEMP DEBUG: pedal debug
  rclc_publisher_init_default(&pub_dbg_pedal, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/debug/pedal");
  rclc_publisher_init_default(&pub_dbg_pedal_raw, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/debug/pedal_raw");

  // Subscriber 콜백 관리
  rclc_executor_t exec;
  rclc_executor_init(&exec, &sup.context, 2, &alloc);
  rclc_executor_add_subscription(&exec, &sub_cmd,
      &s_cmd_msg, &cb_cmd, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec, &sub_aeb,
      &s_aeb_msg, &cb_aeb, ON_NEW_DATA);

  for (;;)
  {
    // Subscribe
    rclc_executor_spin_some(&exec, RCL_MS_TO_NS(10));

    // 현재 속도 Publish
    std_msgs__msg__Float32 f = { .data = CURRENT_SPEED };
    rcl_publish(&pub_spd, &f, NULL);

    // 현재 상태 Publish(DRIVE_STOP_REQUEST, GEAR, MODE)
    std_msgs__msg__UInt8 u;
    u.data = DRIVE_STOP_REQUEST; rcl_publish(&pub_estop, &u, NULL);
    u.data = GEAR;         rcl_publish(&pub_gear,  &u, NULL);
    u.data = MODE;         rcl_publish(&pub_mode,  &u, NULL);
    u.data = PEDAL;        rcl_publish(&pub_dbg_pedal, &u, NULL);  // TEMP DEBUG: 삭제

    std_msgs__msg__Int32 ri = { .data = s_pedal_raw_dbg };
    rcl_publish(&pub_dbg_pedal_raw, &ri, NULL);

    osDelay(10);
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
  // 보드간 UART
  extern UART_HandleTypeDef huart1;
  // Circular DMA 수신 시작
  HAL_UART_Receive_DMA(&huart1, s_brx_buf, 4);

  for (;;)
  {
    // Safety frame: [0xAA][0x02][bitfield][XOR]
    if (s_brx_buf[0] == 0xAAu && s_brx_buf[1] == 0x02u) {
      // Checksum 연산
      uint8_t chk = s_brx_buf[0] ^ s_brx_buf[1] ^ s_brx_buf[2];
      // Checksum이 일치할 때만 반영
      if (chk == s_brx_buf[3]) {
        uint8_t safety_bits = s_brx_buf[2];
        ESTOP_DASHBOARD_FROM_B = (safety_bits >> 0) & 0x01u;
        ESTOP_SIDE_FROM_B = (safety_bits >> 1) & 0x01u;
        ESTOP_RF_FROM_B = (safety_bits >> 2) & 0x01u;
        AEB_FROM_B = (safety_bits >> 3) & 0x01u;
        BRAKE_REQUEST_FROM_B = (safety_bits >> 4) & 0x01u;
        BRAKE_COMMAND_APPLIED_FROM_B = (safety_bits >> 5) & 0x01u;
        s_safety_frame_received = 1u;
      }
    }

    /* 첫 정상 Safety frame 전에는 구동을 허용하지 않는다. */
    DRIVE_STOP_REQUEST = ((s_safety_frame_received == 0u) ||
                          (AEB != 0u) ||
                          (AEB_FROM_B != 0u) ||
                          (ESTOP_DASHBOARD_FROM_B != 0u) ||
                          (ESTOP_SIDE_FROM_B != 0u) ||
                          (ESTOP_RF_FROM_B != 0u) ||
                          (BRAKE_REQUEST_FROM_B != 0u)) ? 1u : 0u;
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

/* ================================================================ */ 
/* Task: BTx — 향후 Vehicle Status용으로 태스크 슬롯만 유지 */
/* ================================================================ */
/* USER CODE END Header_StartBTx */
void StartBTx(void *argument)
{
  /* USER CODE BEGIN StartBTx */
  /* 물리 E-stop의 STM_A -> STM_B 송신은 제거되었다. */
  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartBTx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void send_motor_can(int16_t rpm, float curr_a, uint8_t run_command)
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
  /* bit0: RUNNING, bit1: speed control. HALTED=0x02, RUNNING=0x03. */
  d[4] = (uint8_t)(0x02u | ((run_command != 0u) ? 0x01u : 0x00u));
  d[5] = 0x00u;
  d[6] = 0x00u;
  d[7] = s_life++;

  (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, d);
}

static void cb_cmd(const void *message)
{
  COMMAND_SPEED = ((const std_msgs__msg__Float32 *)message)->data;
}

static void cb_aeb(const void *message)
{
  AEB = ((const std_msgs__msg__UInt8 *)message)->data;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  (void)RxFifo0ITs;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
    if (rx_header.Identifier == 0x1801D0EFUL) {
      uint16_t raw_rpm = (uint16_t)(rx_data[6] | ((uint16_t)rx_data[7] << 8));
      int32_t rpm = (int32_t)raw_rpm - 32000;
      float wheel_rpm = (float)rpm / GEAR_RATIO;
      CURRENT_SPEED = wheel_rpm * (3.14159f * TIRE_DIAM_M) / 60.0f;
    }
  }
}
/* USER CODE END Application */

