#ifndef __BSP_SYSTIC_H__
#define __BSP_SYSTIC_H__

#include "stm32h7xx_hal.h"

#define TICPS 100000

void SystemClock_Config(void);

#endif  // __BSP_SYSTIC_H__