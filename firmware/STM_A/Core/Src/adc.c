/*
---CODE DESCRIPTION---
본 스크립트는 아날로그 전압 0V ~ 3.3V를 12비트 디지털 값 (0~4095 = 2^12)으로 변환합니다.
(이때 12비트는 해상도)

PIN : PA0
*/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * 변경: SamplingTime 2CYCLES_5 → 47CYCLES_5 (페달 신호 안정화)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "adc.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;      // ADC1 Handle(Global)
DMA_HandleTypeDef hdma_adc1;  // DMA Handle for ADC

void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      // ADC 클럭 = PCLK / 4
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;            // 12 비트 (0~4095) -> 해상도 
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           // 우측 정렬
  hadc1.Init.GainCompensation      = 0;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;              // 채널 1개만 사용
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           
  hadc1.Init.LowPowerAutoWait      = DISABLE;                       
  hadc1.Init.ContinuousConvMode    = ENABLE;                        // 연속 변환 모드 설정
  hadc1.Init.NbrOfConversion       = 1;                             // 변환 채널 수 1개  
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            // SW 트리거
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  // ADC1 단독 사용 
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_1;               // IN1 = PA0
  sConfig.Rank         = ADC_REGULAR_RANK_1;          // 변환 순서 1번
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;   // ADC가 입력 전압을 샘플링 하는 시간 -> 47.5 cycle (시간이 짦으면 커패시터 충전이 덜 되기 때문에 값이 튄다)
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            // 단극 입력
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (adcHandle->Instance == ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */
  /* USER CODE END ADC1_MspInit 0 */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK; //ADC 클럭원
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
    
    // ADC 클럭원 활성화
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = GPIO_PIN_0;          // PA0
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;    // 아날로그 모드 설정
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // 아날로그 모드 -> 풀업, 풀다운 없음
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc1.Instance                 = DMA1_Channel5;             // DMA1 채널5
    hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;          
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;      // ADC→메모리
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;           // 메모리 주소 증가
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   // 16비트
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;              // 순환 모드
    hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);                // ADC <-> DMA 연결
  /* USER CODE BEGIN ADC1_MspInit 1 */
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if (adcHandle->Instance == ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */
  /* USER CODE END ADC1_MspDeInit 0 */
    __HAL_RCC_ADC12_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */
  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */