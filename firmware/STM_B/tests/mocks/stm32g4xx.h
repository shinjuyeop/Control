#ifndef TEST_STM32G4XX_H
#define TEST_STM32G4XX_H
#include <stdint.h>
typedef struct { volatile uint32_t CTRL, CYCCNT; } TestDwt;
typedef struct { volatile uint32_t DEMCR; } TestCoreDebug;
extern TestDwt test_dwt;
extern TestCoreDebug test_core_debug;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_core_debug)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define __NOP() (++test_dwt.CYCCNT)
void test_barrier(void);
#define __DMB() test_barrier()
#endif
