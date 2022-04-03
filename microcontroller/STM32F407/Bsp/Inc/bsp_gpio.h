#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "stm32f4xx_hal.h"

#define CHANNEL1_RCC_CLK_ENABLE          __HAL_RCC_GPIOA_CLK_ENABLE
#define CHANNEL1_GPIO_PIN                GPIO_PIN_4
#define CHANNEL1_GPIO                    GPIOA

#define CHANNEL2_RCC_CLK_ENABLE          __HAL_RCC_GPIOA_CLK_ENABLE
#define CHANNEL2_GPIO_PIN                GPIO_PIN_5
#define CHANNEL2_GPIO                    GPIOA

#define CHANNEL3_RCC_CLK_ENABLE          __HAL_RCC_GPIOA_CLK_ENABLE
#define CHANNEL3_GPIO_PIN                GPIO_PIN_6
#define CHANNEL3_GPIO                    GPIOA

#define CHANNEL4_RCC_CLK_ENABLE          __HAL_RCC_GPIOH_CLK_ENABLE
#define CHANNEL4_GPIO_PIN                GPIO_PIN_13
#define CHANNEL4_GPIO                    GPIOH

#define START_RCC_CLK_ENABLE             __HAL_RCC_GPIOH_CLK_ENABLE
#define START_GPIO_PIN                   GPIO_PIN_14
#define START_GPIO                       GPIOH

#define CLOCK_RCC_CLK_ENABLE             __HAL_RCC_GPIOH_CLK_ENABLE
#define CLOCK_GPIO_PIN                   GPIO_PIN_15
#define CLOCK_GPIO                       GPIOH

#define CHANNEL1_HIGH()                  HAL_GPIO_WritePin(CHANNEL1_GPIO,CHANNEL1_GPIO_PIN,GPIO_PIN_SET)
#define CHANNEL1_LOW()                   HAL_GPIO_WritePin(CHANNEL1_GPIO,CHANNEL1_GPIO_PIN,GPIO_PIN_RESET)
#define CHANNEL1_TOGGLE()                HAL_GPIO_TogglePin(CHANNEL1_GPIO, CHANNEL1_GPIO_PIN)

#define CHANNEL2_HIGH()                  HAL_GPIO_WritePin(CHANNEL2_GPIO,CHANNEL2_GPIO_PIN,GPIO_PIN_SET)
#define CHANNEL2_LOW()                   HAL_GPIO_WritePin(CHANNEL2_GPIO,CHANNEL2_GPIO_PIN,GPIO_PIN_RESET)
#define CHANNEL2_TOGGLE()                HAL_GPIO_TogglePin(CHANNEL2_GPIO, CHANNEL2_GPIO_PIN)

#define CHANNEL3_HIGH()                  HAL_GPIO_WritePin(CHANNEL3_GPIO,CHANNEL3_GPIO_PIN,GPIO_PIN_SET)
#define CHANNEL3_LOW()                   HAL_GPIO_WritePin(CHANNEL3_GPIO,CHANNEL3_GPIO_PIN,GPIO_PIN_RESET)
#define CHANNEL3_TOGGLE()                HAL_GPIO_TogglePin(CHANNEL3_GPIO, CHANNEL3_GPIO_PIN)

#define CHANNEL4_HIGH()                  HAL_GPIO_WritePin(CHANNEL4_GPIO,CHANNEL4_GPIO_PIN,GPIO_PIN_SET)
#define CHANNEL4_LOW()                   HAL_GPIO_WritePin(CHANNEL4_GPIO,CHANNEL4_GPIO_PIN,GPIO_PIN_RESET)
#define CHANNEL4_TOGGLE()                HAL_GPIO_TogglePin(CHANNEL4_GPIO, CHANNEL4_GPIO_PIN)

#define START_HIGH()                     HAL_GPIO_WritePin(START_GPIO,START_GPIO_PIN,GPIO_PIN_SET)
#define START_LOW()                      HAL_GPIO_WritePin(START_GPIO,START_GPIO_PIN,GPIO_PIN_RESET)
#define START_TOGGLE()                   HAL_GPIO_TogglePin(START_GPIO, START_GPIO_PIN)

#define CLOCK_HIGH()                     HAL_GPIO_WritePin(CLOCK_GPIO,CLOCK_GPIO_PIN,GPIO_PIN_SET)
#define CLOCK_LOW()                      HAL_GPIO_WritePin(CLOCK_GPIO,CLOCK_GPIO_PIN,GPIO_PIN_RESET)
#define CLOCK_TOGGLE()                   HAL_GPIO_TogglePin(CLOCK_GPIO, CLOCK_GPIO_PIN)


void OUTPUT_GPIO_Init(void);

#endif  // __BSP_GPIO_H__