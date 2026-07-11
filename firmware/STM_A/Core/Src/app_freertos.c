/*
---- CODE DESCRIPTION ----
STM A의 6개 Task에 대한 로직.
- Control (Realtime)   : 목표속도→RPM 변환 후 모터 CAN 송신. ESTOP시 정지
- Inputs  (High1)      : 페달 ADC 평균, 버튼·스위치 읽기, ESTOP 통합
- MotorRx (High2)      : 모터 CAN 수신은 콜백에서 처리, 태스크는 대기
- uROS    (Normal)     : /cmd/speed·/aeb 구독, 상태 4종 발행
- BRx     (AboveNormal): STM_B의 RF ESTOP를 UART 수신
- BTx     (BelowNormal): 물리버튼 ESTOP를 STM_B로 UART 송신
*/
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* BRx DMA 수신 버퍼 (4바이트 프레임) */
static uint8_t s_brx_buf[4];
/* USER CODE END Variables */

/* ── 태스크 속성 ── */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name       = "Control",
  .priority   = (osPriority_t)osPriorityRealtime,   /* ★ Realtime */
  .stack_size = 512 * 4
};
osThreadId_t InputsHandle;
const osThreadAttr_t Inputs_attributes = {
  .name       = "Inputs",
  .priority   = (osPriority_t)osPriorityHigh1,
  .stack_size = 256 * 4
};
osThreadId_t MotorRxHandle;
const osThreadAttr_t MotorRx_attributes = {
  .name       = "MotorRx",
  .priority   = (osPriority_t)osPriorityHigh2,
  .stack_size = 256 * 4
};
osThreadId_t uROSHandle;
const osThreadAttr_t uROS_attributes = {
  .name       = "uROS",
  .priority   = (osPriority_t)osPriorityNormal,
  .stack_size = 1536 * 4
};
osThreadId_t BRxHandle;
const osThreadAttr_t BRx_attributes = {
  .name       = "BRx",
  .priority   = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 256 * 4
};
osThreadId_t BTxHandle;
const osThreadAttr_t BTx_attributes = {
  .name       = "BTx",
  .priority   = (osPriority_t)osPriorityBelowNormal, /* ★ BelowNormal */
  .stack_size = 256 * 4
};

/* 태스트 함수 선언*/
void StartControl (void *argument);
void StartInputs  (void *argument);
void StartMotorRx (void *argument);
void StartuROS    (void *argument);
void StartBRx     (void *argument);
void StartBTx     (void *argument);

/* ======================================================= */
/* FDCAN CAN Tx (EZkontrol J1939)
 *   CAN ID : 0x0C01EFD0 (Extended)
 *   data[0-1]: 전류 리밋  uint16, offset -3200A, 0.1A/bit
 *   data[2-3]: 목표 RPM   uint16, offset -32000,  1rpm/bit
 *   data[4]  : 0x03 (CTRL_RUN|CTRL_SPEED)
 *   data[7]  : life counter (프레임마다 증가) */
 /* ======================================================= */

/* 
FreeRTOS 초기화
main.c에서 스케줄러 시작 전 호출
*/

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 모터 상수
#define TIRE_DIAM_M    0.47f    // 타이어 지름
#define GEAR_RATIO     4.60f    // 감속비 (모터 회전 : 바퀴 회전)
#define RPM_FWD_MAX    4500     // 전진 최대 RPM
#define RPM_REV_MAX    1500     // 후진 최대 RPM
#define PHASE_LIMIT_A  300.0f   // 전류 Limit = 300A

static uint8_t s_life = 0;  // life counter (0~255)

// 모터 송신 함수
static void send_motor_can(int16_t rpm, float curr_a)
{
  FDCAN_TxHeaderTypeDef hdr;    // CAN 프레임 헤더
  uint8_t d[8];                 // 8바이트 데이터
  // uint32_t mb;               // 송신 메일박스(이게 뭐지???)

  // 전류 인코딩
  // 실제 전류 -> 정수값
  double   rc  = (curr_a - (-3200.0)) / 0.1;
  // Clamping (0~ 65535) (왜???)
  uint16_t cr  = (uint16_t)fmax(0.0, fmin(65535.0, rc));

  // RPM 클램핑
  // 전진 & 후진 최대치 제한
  int32_t  rc2 = rpm >= 0
                   ? (int32_t)fmin((float)rpm, (float)RPM_FWD_MAX)
                   : (int32_t)fmax((float)rpm, (float)-RPM_REV_MAX);
  // offset 인코딩
  uint16_t sr  = (uint16_t)(rc2 + 32000);

  // CAN 헤더 설정
  hdr.Identifier          = 0x0C01EFD0UL;
  hdr.IdType              = FDCAN_EXTENDED_ID;
  hdr.TxFrameType         = FDCAN_DATA_FRAME;
  hdr.DataLength          = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch       = FDCAN_BRS_OFF;
  hdr.FDFormat            = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker       = 0;

  // 데이터 바이트 채움 : Little Endian 방식
  d[0] = (uint8_t)(cr & 0xFF);    
  d[1] = (uint8_t)(cr >> 8);
  d[2] = (uint8_t)(sr & 0xFF);
  d[3] = (uint8_t)(sr >> 8);
  d[4] = 0x03u;
  d[5] = 0x00u;
  d[6] = 0x00u;
  d[7] = s_life++;

  // CAN 송신
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, d);
}

/* 
 * FDCAN Rx 콜백 — ISR (MotorRx 로직)
 *   수신 ID : 0x1801D0EF
 *   data[6-7]: 실제 RPM uint16, offset -32000
*/

// HAL이 CAN프레임 도착 시 자동 호출
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rxh;
  uint8_t rxd[8];
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK)
  {
    if (rxh.Identifier == 0x1801D0EFUL) // 모터 피드백 프레임 확인
    {
      // RPM 디코딩
      // 바이트 결합
      uint16_t ru   = (uint16_t)(rxd[6] | ((uint16_t)rxd[7] << 8));
      // 오프셋 제거
      int32_t  rpm  = (int32_t)ru - 32000;
      // RPM -> Wheel RPM -> 속도 변환
      float    wrpm = (float)rpm / GEAR_RATIO;
      // CURREN_SPEED 전역변수 write
      CURRENT_SPEED = wrpm * (3.14159f * TIRE_DIAM_M) / 60.0f;
    }
  }
}

/* ====================================================== */
/* Task: Inputs — 페달 + 버튼 + ESTOP 통합 (osPriorityHigh1) */
/* ====================================================== */
#define PEDAL_ADC_MIN 200   // 페달 최소 ADC
#define PEDAL_ADC_MAX 3900  // 페달 최대 ADC

/* uROS 구독 콜백 (StartuROS 태스크에서 사용, 파일 스코프로 이동) */
// /cmd/speed -> COMMAND_SPEED
static void cb_cmd(const void *m)
{ COMMAND_SPEED = ((const std_msgs__msg__Float32 *)m)->data; }

// /aeb -> AEB
static void cb_aeb(const void *m)
{ AEB = ((const std_msgs__msg__UInt8 *)m)->data; }

/* 
FreeRTOS 초기화
main.c에서 스케줄러 시작 전 호출
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
  // 6개 Task 스레드 생성
  ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);
  InputsHandle  = osThreadNew(StartInputs,  NULL, &Inputs_attributes);
  MotorRxHandle = osThreadNew(StartMotorRx, NULL, &MotorRx_attributes);
  uROSHandle    = osThreadNew(StartuROS,    NULL, &uROS_attributes);
  BRxHandle     = osThreadNew(StartBRx,     NULL, &BRx_attributes);
  BTxHandle     = osThreadNew(StartBTx,     NULL, &BTx_attributes);

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
    // ESTOP 비활성시에만 주행
    if (ESTOP_STATUS == 0)
    {
      // 자율주행 : PC cmd, 수동주행 : 페달비율 (최대 3m/s)
      float spd = (MODE == 1) ? COMMAND_SPEED : ((float)PEDAL / 100.0f * 3.0f);
      // 속도 -> 모터 RPM 변환                          
      float wrpm   = spd * 60.0f / (3.14159f * TIRE_DIAM_M);
      int16_t mrpm = (int16_t)(wrpm * GEAR_RATIO);
      // 후진이면 음수 RPM
      if (GEAR == 1) mrpm = (int16_t)-abs(mrpm);
      // 모터 송신
      send_motor_can(mrpm, PHASE_LIMIT_A);
    }
    // ESTOP 활성시
    else
    {
      // RPM = 0로 명령
      send_motor_can(0, 0.0f);
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
    // 데드밴드 클램핑 (무반응 구간, 유격감안)
    if (raw < PEDAL_ADC_MIN) raw = PEDAL_ADC_MIN;
    if (raw > PEDAL_ADC_MAX) raw = PEDAL_ADC_MAX;
    // 0~100% 선형 매핑
    int p = (raw - PEDAL_ADC_MIN) * 100 / (PEDAL_ADC_MAX - PEDAL_ADC_MIN);
    // 떨림 방지를 위한 미세값 보정(0으로 대체)(???)
    if (p < 2) p = 0;
    PEDAL = (uint8_t)p;

    // 물리버튼 (PULLUP / 눌림 : RESET = LOW)
    ESTOP_DASHBOARD = (HAL_GPIO_ReadPin(ESTOP_DASH_GPIO_Port,
                                        ESTOP_DASH_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
    ESTOP_SIDE      = (HAL_GPIO_ReadPin(ESTOP_SIDE_GPIO_Port,
                                        ESTOP_SIDE_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

    // 스위치 (눌림 : Set = High)
    MODE = (HAL_GPIO_ReadPin(SW_MODE_GPIO_Port, SW_MODE_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    GEAR = (HAL_GPIO_ReadPin(SW_GEAR_GPIO_Port, SW_GEAR_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    // ESTOP 통합
    // B로 보낼 값 (물리버튼 두 개만)
    ESTOP_TO_B   = ESTOP_DASHBOARD | ESTOP_SIDE;
    // ESTOP 전체 통합
    ESTOP_STATUS = AEB | ESTOP_DASHBOARD | ESTOP_SIDE | ESTOP_FROM_B;

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

    // 현재 상태 Publish(ESTOP_STATUS, GEAR, MODE)
    std_msgs__msg__UInt8 u;
    u.data = ESTOP_STATUS; rcl_publish(&pub_estop, &u, NULL);
    u.data = GEAR;         rcl_publish(&pub_gear,  &u, NULL);
    u.data = MODE;         rcl_publish(&pub_mode,  &u, NULL);

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
/* Task: BRx — STM_B -> A 수신 (RF 상태)  (osPriorityAboveNormal) */
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
    // 프레임 확인 : [0xAA][0x01][data][XOR]
    if (s_brx_buf[0] == 0xAAu && s_brx_buf[1] == 0x01u) {
      // Checksum 연산
      uint8_t chk = s_brx_buf[0] ^ s_brx_buf[1] ^ s_brx_buf[2];
      // Checksum이 일치할 때만 반영
      if (chk == s_brx_buf[3])
        ESTOP_FROM_B = s_brx_buf[2];
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

/* ================================================================ */ 
/* Task: BTx — A -> STM_B 송신 (물리버튼 2개)  (osPriorityBelowNormal) */
/* ================================================================ */
/* USER CODE END Header_StartBTx */
void StartBTx(void *argument)
{
  /* USER CODE BEGIN StartBTx */
  extern UART_HandleTypeDef huart1;

  for (;;)
  {
    // 프레임 조립 : [0xAA][0x01][ESTOP_TO_B][XOR]
    uint8_t pkt[4];
    pkt[0] = 0xAAu;
    pkt[1] = 0x01u;
    pkt[2] = ESTOP_TO_B;
    pkt[3] = pkt[0] ^ pkt[1] ^ pkt[2];
    // 프레임 송신
    HAL_UART_Transmit(&huart1, pkt, 4, 10);
    osDelay(20);
  }
  /* USER CODE END StartBTx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

