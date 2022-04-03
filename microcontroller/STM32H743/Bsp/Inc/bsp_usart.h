#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32h7xx_hal.h"


#define RCC_PERIPHCLK_UARTx                    RCC_PERIPHCLK_USART3
#define MX_USARTx                              USART3
#define MX_USARTx_BAUDRATE                     115200
#define MX_USART_RCC_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE()
#define MX_USARTx_GPIO_AFx                     GPIO_AF7_USART3

#define MX_USARTx_Tx_GPIO_ClK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define MX_USARTx_Tx_GPIO_PIN                  GPIO_PIN_10
#define MX_USARTx_Tx_GPIO                      GPIOB

#define MX_USARTx_Rx_GPIO_ClK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define MX_USARTx_Rx_GPIO_PIN                  GPIO_PIN_11
#define MX_USARTx_Rx_GPIO                      GPIOB

#define MX_USARTx_IRQn                         USART3_IRQn
#define MX_USARTx_IRQHandler                   USART3_IRQHandler

#define MAX_COUNT           255


extern UART_HandleTypeDef husartx;
extern __IO uint8_t aRxBuffer[MAX_COUNT];
extern __IO uint16_t RxCount;
extern void RxCallback(uint16_t len);


void MX_USARTx_Init(void);

#endif // __BSP_USART_H__
