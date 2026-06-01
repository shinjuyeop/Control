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

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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

/* USER CODE BEGIN Variables */
/* BRx DMA 수신 버퍼 */
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

void StartControl (void *argument);
void StartInputs  (void *argument);
void StartMotorRx (void *argument);
void StartuROS    (void *argument);
void StartBRx     (void *argument);
void StartBTx     (void *argument);

void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);
  InputsHandle  = osThreadNew(StartInputs,  NULL, &Inputs_attributes);
  MotorRxHandle = osThreadNew(StartMotorRx, NULL, &MotorRx_attributes);
  uROSHandle    = osThreadNew(StartuROS,    NULL, &uROS_attributes);
  BRxHandle     = osThreadNew(StartBRx,     NULL, &BRx_attributes);
  BTxHandle     = osThreadNew(StartBTx,     NULL, &BTx_attributes);
}

/* =========================================================================
 * 모터 상수
 * ========================================================================= */
#define TIRE_DIAM_M    0.47f
#define GEAR_RATIO     4.60f
#define RPM_FWD_MAX    4500
#define RPM_REV_MAX    1500
#define PHASE_LIMIT_A  300.0f

/* =========================================================================
 * FDCAN CAN Tx (EZkontrol J1939)
 *   CAN ID : 0x0C01EFD0 (Extended)
 *   data[0-1]: 전류 리밋  uint16, offset -3200A, 0.1A/bit
 *   data[2-3]: 목표 RPM   uint16, offset -32000,  1rpm/bit
 *   data[4]  : 0x03 (CTRL_RUN|CTRL_SPEED)
 *   data[7]  : life counter
 * ========================================================================= */
static uint8_t s_life = 0;

static void send_motor_can(int16_t rpm, float curr_a)
{
  FDCAN_TxHeaderTypeDef hdr;
  uint8_t d[8];
  // uint32_t mb;

  double   rc  = (curr_a - (-3200.0)) / 0.1;
  uint16_t cr  = (uint16_t)fmax(0.0, fmin(65535.0, rc));

  int32_t  rc2 = rpm >= 0
                   ? (int32_t)fmin((float)rpm, (float)RPM_FWD_MAX)
                   : (int32_t)fmax((float)rpm, (float)-RPM_REV_MAX);
  uint16_t sr  = (uint16_t)(rc2 + 32000);

  hdr.Identifier          = 0x0C01EFD0UL;
  hdr.IdType              = FDCAN_EXTENDED_ID;
  hdr.TxFrameType         = FDCAN_DATA_FRAME;
  hdr.DataLength          = FDCAN_DLC_BYTES_8;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch       = FDCAN_BRS_OFF;
  hdr.FDFormat            = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker       = 0;

  d[0] = (uint8_t)(cr & 0xFF);
  d[1] = (uint8_t)(cr >> 8);
  d[2] = (uint8_t)(sr & 0xFF);
  d[3] = (uint8_t)(sr >> 8);
  d[4] = 0x03u;
  d[5] = 0x00u;
  d[6] = 0x00u;
  d[7] = s_life++;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, d);
}

/* =========================================================================
 * FDCAN Rx 콜백 — ISR (MotorRx 로직)
 *   수신 ID : 0x1801D0EF
 *   data[6-7]: 실제 RPM uint16, offset -32000
 * ========================================================================= */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rxh;
  uint8_t rxd[8];
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, rxd) == HAL_OK)
  {
    if (rxh.Identifier == 0x1801D0EFUL)
    {
      uint16_t ru   = (uint16_t)(rxd[6] | ((uint16_t)rxd[7] << 8));
      int32_t  rpm  = (int32_t)ru - 32000;
      float    wrpm = (float)rpm / GEAR_RATIO;
      CURRENT_SPEED = wrpm * (3.14159f * TIRE_DIAM_M) / 60.0f;
    }
  }
}

/* =========================================================================
 * Task: Control — 모터 제어 (osPriorityRealtime)
 * ========================================================================= */
void StartControl(void *argument)
{
  /* USER CODE BEGIN StartControl */
  /* FDCAN 필터 설정 */
  FDCAN_FilterTypeDef f;
  f.IdType       = FDCAN_EXTENDED_ID;
  f.FilterIndex  = 0;
  f.FilterType   = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = 0x1801D0EFUL;
  f.FilterID2    = 0x1FFFFFFFUL;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1,
                                  FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  for (;;)
  {
    if (ESTOP_STATUS == 0)
    {
      float spd = (MODE == 1) ? COMMAND_SPEED
                              : ((float)PEDAL / 100.0f * 3.0f);
      float wrpm   = spd * 60.0f / (3.14159f * TIRE_DIAM_M);
      int16_t mrpm = (int16_t)(wrpm * GEAR_RATIO);
      if (GEAR == 1) mrpm = (int16_t)-abs(mrpm);
      send_motor_can(mrpm, PHASE_LIMIT_A);
    }
    else
    {
      send_motor_can(0, 0.0f);
    }
    osDelay(10);
  }
  /* USER CODE END StartControl */
}

/* =========================================================================
 * Task: Inputs — 페달 + 버튼 + ESTOP 통합 (osPriorityHigh1)
 * ========================================================================= */
#define PEDAL_ADC_MIN 200
#define PEDAL_ADC_MAX 3900

void StartInputs(void *argument)
{
  /* USER CODE BEGIN StartInputs */
  for (;;)
  {
    /* 페달 ADC 8회 평균 */
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 10);
      sum += HAL_ADC_GetValue(&hadc1);
    }
    int raw = (int)(sum / 8u);
    if (raw < PEDAL_ADC_MIN) raw = PEDAL_ADC_MIN;
    if (raw > PEDAL_ADC_MAX) raw = PEDAL_ADC_MAX;
    int p = (raw - PEDAL_ADC_MIN) * 100 / (PEDAL_ADC_MAX - PEDAL_ADC_MIN);
    if (p < 2) p = 0;
    PEDAL = (uint8_t)p;

    /* 물리버튼 (PULLUP → RESET = 눌림) */
    ESTOP_DASHBOARD = (HAL_GPIO_ReadPin(ESTOP_DASH_GPIO_Port,
                                        ESTOP_DASH_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
    ESTOP_SIDE      = (HAL_GPIO_ReadPin(ESTOP_SIDE_GPIO_Port,
                                        ESTOP_SIDE_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

    /* 스위치 */
    MODE = (HAL_GPIO_ReadPin(SW_MODE_GPIO_Port, SW_MODE_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    GEAR = (HAL_GPIO_ReadPin(SW_GEAR_GPIO_Port, SW_GEAR_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    /* ESTOP 통합 */
    ESTOP_TO_B   = ESTOP_DASHBOARD | ESTOP_SIDE;
    ESTOP_STATUS = AEB | ESTOP_DASHBOARD | ESTOP_SIDE | ESTOP_FROM_B;

    osDelay(10);
  }
  /* USER CODE END StartInputs */
}

/* =========================================================================
 * Task: MotorRx — Rx는 콜백 처리, 태스크 최소화 (osPriorityHigh2)
 * ========================================================================= */
void StartMotorRx(void *argument)
{
  /* USER CODE BEGIN StartMotorRx */
  for (;;) { osDelay(100); }
  /* USER CODE END StartMotorRx */
}

/* =========================================================================
 * Task: uROS — PC 양방향 통신 (osPriorityNormal)
 * ========================================================================= */
static std_msgs__msg__Float32 s_cmd_msg;
static std_msgs__msg__UInt8   s_aeb_msg;

static void cb_cmd(const void *m)
{ COMMAND_SPEED = ((const std_msgs__msg__Float32 *)m)->data; }

static void cb_aeb(const void *m)
{ AEB = ((const std_msgs__msg__UInt8 *)m)->data; }

void StartuROS(void *argument)
{
  /* USER CODE BEGIN StartuROS */
  extern UART_HandleTypeDef hlpuart1;

  rmw_uros_set_custom_transport(
      true, (void *)&hlpuart1,
      cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read);

  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t  sup;
  rcl_node_t      node;

  rcl_subscription_t sub_cmd, sub_aeb;
  rcl_publisher_t    pub_spd, pub_estop, pub_gear, pub_mode;

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
    osDelay(500);

  rclc_support_init(&sup, 0, NULL, &alloc);
  rclc_node_init_default(&node, "stm_a_node", "", &sup);

  rclc_subscription_init_default(&sub_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/cmd/speed");
  rclc_subscription_init_default(&sub_aeb, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/aeb");

  rclc_publisher_init_default(&pub_spd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current/speed");
  rclc_publisher_init_default(&pub_estop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/estop/status");
  rclc_publisher_init_default(&pub_gear, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/gear");
  rclc_publisher_init_default(&pub_mode, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/mode");

  rclc_executor_t exec;
  rclc_executor_init(&exec, &sup.context, 2, &alloc);
  rclc_executor_add_subscription(&exec, &sub_cmd,
      &s_cmd_msg, &cb_cmd, ON_NEW_DATA);
  rclc_executor_add_subscription(&exec, &sub_aeb,
      &s_aeb_msg, &cb_aeb, ON_NEW_DATA);

  for (;;)
  {
    rclc_executor_spin_some(&exec, RCL_MS_TO_NS(10));

    std_msgs__msg__Float32 f = { .data = CURRENT_SPEED };
    rcl_publish(&pub_spd, &f, NULL);

    std_msgs__msg__UInt8 u;
    u.data = ESTOP_STATUS; rcl_publish(&pub_estop, &u, NULL);
    u.data = GEAR;         rcl_publish(&pub_gear,  &u, NULL);
    u.data = MODE;         rcl_publish(&pub_mode,  &u, NULL);

    osDelay(10);
  }
  /* USER CODE END StartuROS */
}

/* =========================================================================
 * Task: BRx — STM_B→A 수신 (RF 상태)  (osPriorityAboveNormal)
 * ========================================================================= */
void StartBRx(void *argument)
{
  /* USER CODE BEGIN StartBRx */
  extern UART_HandleTypeDef huart1;
  HAL_UART_Receive_DMA(&huart1, s_brx_buf, 4);

  for (;;)
  {
    if (s_brx_buf[0] == 0xAAu && s_brx_buf[1] == 0x01u) {
      uint8_t chk = s_brx_buf[0] ^ s_brx_buf[1] ^ s_brx_buf[2];
      if (chk == s_brx_buf[3])
        ESTOP_FROM_B = s_brx_buf[2];
    }
    osDelay(10);
  }
  /* USER CODE END StartBRx */
}

/* =========================================================================
 * Task: BTx — A→STM_B 송신 (물리버튼 2개)  (osPriorityBelowNormal)
 * ========================================================================= */
void StartBTx(void *argument)
{
  /* USER CODE BEGIN StartBTx */
  extern UART_HandleTypeDef huart1;

  for (;;)
  {
    uint8_t pkt[4];
    pkt[0] = 0xAAu;
    pkt[1] = 0x01u;
    pkt[2] = ESTOP_TO_B;
    pkt[3] = pkt[0] ^ pkt[1] ^ pkt[2];
    HAL_UART_Transmit(&huart1, pkt, 4, 10);
    osDelay(20);
  }
  /* USER CODE END StartBTx */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */