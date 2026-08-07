#include "hil_command.h"

#include "hil_logic.h"
#include "hil_state.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <string.h>

#define HIL_UART_DMA_BUFFER_SIZE 128u
#define HIL_UART_RING_SIZE       512u
#define HIL_UART_LINE_SIZE       160u
#define HIL_UART_REPLY_SIZE      768u
#define HIL_UART_TX_TIMEOUT_MS   100u

extern UART_HandleTypeDef hlpuart1;

static uint8_t s_dma_buffer[HIL_UART_DMA_BUFFER_SIZE];
static uint8_t s_rx_ring[HIL_UART_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_rx_overflow;

static void start_rx_dma(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, s_dma_buffer,
                                  sizeof(s_dma_buffer)) == HAL_OK) {
    if (hlpuart1.hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
    }
  }
}

static bool pop_byte(uint8_t *value)
{
  const uint16_t tail = s_rx_tail;
  if ((value == NULL) || (tail == s_rx_head)) return false;
  *value = s_rx_ring[tail];
  s_rx_tail = (uint16_t)((tail + 1u) % HIL_UART_RING_SIZE);
  return true;
}

static void uart_write_line(const char *text)
{
  if (text == NULL) return;
  (void)HAL_UART_Transmit(&hlpuart1, (const uint8_t *)text,
                         (uint16_t)strlen(text), HIL_UART_TX_TIMEOUT_MS);
  static const uint8_t newline[] = "\r\n";
  (void)HAL_UART_Transmit(&hlpuart1, newline, 2u, HIL_UART_TX_TIMEOUT_MS);
}

void HilCommand_Init(void)
{
  s_rx_head = 0u;
  s_rx_tail = 0u;
  s_rx_overflow = false;
  start_rx_dma();
}

void HilCommand_RxEventFromISR(uint16_t size)
{
  if (size > HIL_UART_DMA_BUFFER_SIZE) size = HIL_UART_DMA_BUFFER_SIZE;
  for (uint16_t i = 0u; i < size; ++i) {
    const uint16_t next = (uint16_t)((s_rx_head + 1u) % HIL_UART_RING_SIZE);
    if (next == s_rx_tail) {
      s_rx_overflow = true;
      break;
    }
    s_rx_ring[s_rx_head] = s_dma_buffer[i];
    s_rx_head = next;
  }
  start_rx_dma();
}

void HilCommand_UartErrorFromISR(void)
{
  start_rx_dma();
}

void HilCommand_Task(void)
{
  /* Static, not automatic: the HostCommand thread has a 2 KB stack and the
   * STATUS snprintf already spends a few hundred bytes of it on newlib float
   * formatting. Keeping 928 bytes of buffers off the stack leaves that margin
   * intact. Only this one thread calls into here, so there is no sharing. */
  static char line[HIL_UART_LINE_SIZE];
  static char reply[HIL_UART_REPLY_SIZE];
  size_t length = 0u;
  bool discard_line = false;

  for (;;) {
    if (s_rx_overflow) {
      s_rx_overflow = false;
      discard_line = true;
      uart_write_line("ERR rx_overflow");
    }

    uint8_t byte = 0u;
    while (pop_byte(&byte)) {
      if (byte == '\r') continue;
      if (byte == '\n') {
        if (discard_line) {
          discard_line = false;
          length = 0u;
          continue;
        }
        if (length == 0u) continue;
        line[length] = '\0';
        HilState_t *state = HilState_Lock(20u);
        if (state == NULL) {
          uart_write_line("ERR state_busy");
        } else {
          (void)HilLogic_ProcessCommand(state, line, reply, sizeof(reply));
          HilState_Unlock();
          uart_write_line(reply);
        }
        length = 0u;
      } else if (!discard_line) {
        if (length < sizeof(line) - 1u) {
          line[length++] = (char)byte;
        } else {
          discard_line = true;
          length = 0u;
          uart_write_line("ERR line_too_long");
        }
      }
    }
    osDelay(2u);
  }
}
