#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

typedef struct
{
  uint32_t unused;
} SPI_HandleTypeDef;

typedef enum
{
  HAL_OK = 0U,
  HAL_ERROR = 1U
} HAL_StatusTypeDef;

typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET = 1U
} GPIO_PinState;

#define GPIOB ((void *)0x40020400U)
#define GPIO_PIN_0  (1U << 0)
#define GPIO_PIN_1  (1U << 1)
#define GPIO_PIN_2  (1U << 2)
#define GPIO_PIN_10 (1U << 10)

#define LCD_CS_Pin GPIO_PIN_0
#define LCD_CS_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_10
#define LCD_BL_GPIO_Port GPIOB

void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms);

#endif /* TEST_MAIN_H */
