#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32f4xx_hal.h"


#define RCC_PERIPHCLK_UARTx                    RCC_PERIPHCLK_USART1
#define MX_USARTx                                 USART1
#define MX_USARTx_BAUDRATE                        115200
#define MX_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART1_CLK_ENABLE()
#define MX_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART1_CLK_DISABLE()
#define MX_USARTx_GPIO_AFx                        GPIO_AF7_USART1

#define MX_USARTx_Tx_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define MX_USARTx_Tx_GPIO_PIN                     GPIO_PIN_6
#define MX_USARTx_Tx_GPIO                         GPIOB

#define MX_USARTx_Rx_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define MX_USARTx_Rx_GPIO_PIN                     GPIO_PIN_7   
#define MX_USARTx_Rx_GPIO                         GPIOB



#define MX_USARTx_IRQn                            USART1_IRQn
#define MX_USARTx_IRQHandler                      USART1_IRQHandler



#define MAX_COUNT           255


extern UART_HandleTypeDef husartx;
extern __IO uint8_t aRxBuffer[MAX_COUNT];
extern __IO uint16_t RxCount;
extern void RxCallback(uint16_t len);


void MX_USARTx_Init(void);

#endif // __BSP_USART_H__
