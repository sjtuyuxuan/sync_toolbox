
#include "bsp_usart.h"

UART_HandleTypeDef husartx;



static void YS_UARTx_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct={0};
  MX_USART_RCC_CLK_ENABLE();

  MX_USARTx_Tx_GPIO_ClK_ENABLE();
	MX_USARTx_Rx_GPIO_ClK_ENABLE();
  

  GPIO_InitStruct.Pin = MX_USARTx_Tx_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = MX_USARTx_GPIO_AFx;
  HAL_GPIO_Init(MX_USARTx_Tx_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MX_USARTx_Rx_GPIO_PIN;
  HAL_GPIO_Init(MX_USARTx_Rx_GPIO, &GPIO_InitStruct);
    

  HAL_NVIC_SetPriority(MX_USARTx_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(MX_USARTx_IRQn);
}


void MX_USARTx_Init(void)
{
  YS_UARTx_MspInit();
  husartx.Instance = MX_USARTx;
  husartx.Init.BaudRate   = MX_USARTx_BAUDRATE;
  husartx.Init.WordLength = UART_WORDLENGTH_8B;
  husartx.Init.StopBits   = UART_STOPBITS_1;
  husartx.Init.Parity     = UART_PARITY_NONE;
  husartx.Init.Mode       = UART_MODE_TX_RX;
  husartx.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  husartx.Init.OverSampling    = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx);
}

void MX_USARTx_IRQHandler(void)
{
	if(__HAL_USART_GET_FLAG(&husartx,USART_FLAG_RXNE)!= RESET)
	{
		uint8_t data;
		data=READ_REG(husartx.Instance->DR);
		if(RxCount==0)
		{
			__HAL_UART_CLEAR_FLAG(&husartx,USART_FLAG_IDLE);
		  __HAL_UART_ENABLE_IT(&husartx,UART_IT_IDLE);  
		}
		if(RxCount<MAX_COUNT)
		{
			aRxBuffer[RxCount]=data;
			RxCount++;
		}
	}
	else	if(__HAL_USART_GET_FLAG(&husartx,USART_FLAG_IDLE)!= RESET)
	{
		__HAL_UART_CLEAR_FLAG(&husartx,USART_FLAG_IDLE);
		__HAL_UART_DISABLE_IT(&husartx,UART_IT_IDLE);
		RxCallback(RxCount);
		RxCount=0;
	}
}
