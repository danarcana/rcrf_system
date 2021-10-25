/*
 * usart_cb.c
 *
 *  Created on: Oct 1, 2021
 *      Author: MVD
 */

#include "stm32f1xx_ll_usart.h"

volatile uint8_t usart3_buffer[250] = {0};
volatile uint8_t usart3_dr = 0;
volatile uint8_t usart3_buff_idx = 0;

volatile uint8_t usart3_last_rcv_idx = 0;
/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of DR register */
	usart3_dr = LL_USART_ReceiveData8(USART3);
//	LL_USART_TransmitData8(USART2, usart3_dr);
	usart3_buffer[usart3_buff_idx] = usart3_dr;
	usart3_buff_idx++;
	if (usart3_buff_idx >=235 )
	{
		usart3_buff_idx = 0;
	}

//  if (usart3_buff_idx == ubUSART1NbDataToReceive)
//  {
//    /* Set USART1 End of Reception flag */
//    ubUSART1ReceptionComplete = 1;
//  }
}


/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART_TXEmpty_Callback(void)
{
	//Transmit USART3 buffer on USART1
	if (usart3_last_rcv_idx > 0)
	{
		LL_USART_TransmitData8(USART2, usart3_buffer[usart3_buff_idx]);
		usart3_buff_idx ++;
		if (usart3_buff_idx > usart3_last_rcv_idx)
		{
			usart3_buff_idx = 0;
			usart3_last_rcv_idx = 0;
		}
	}
	else
	{
//		usart3_buff_idx = 0;
//		usart3_last_rcv_idx = 0;
	}
//  if(ubSend == (ubUSART1NbDataToTransmit - 1))
//  {
//    /* Disable TXE interrupt */
//    LL_USART_DisableIT_TXE(USART1);
//
//    /* Enable TC interrupt */
//    LL_USART_EnableIT_TC(USART1);
//  }
//
//  /* Fill DR with a new char */
//  LL_USART_TransmitData8(USART1, aUSART1TxBuffer[ubSend++]);
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART_CharTransmitComplete_Callback(void)
{
//  if(ubSend == ubUSART1NbDataToTransmit)
//  {
//    /* Disable TC interrupt */
//    LL_USART_DisableIT_TC(USART1);
//
//    /* Set USART1 End of traubUSART1TransmissionCompletensmission flag */
//    ubUSART1TransmissionComplete = 1;
//  }
}


/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void USART_TransferError_Callback(void)
{
//  /* Disable USART1_IRQn */
//  NVIC_DisableIRQ(USART1_IRQn);
//
//  /* Set LED2 to Blinking mode to indicate error occurs */
//  LED_Blinking(LED_BLINK_ERROR);
}


void TIM_ExpireCallback(void)
{
	usart3_last_rcv_idx  = usart3_buff_idx;
	usart3_buff_idx = 0;
	LL_USART_TransmitData8(USART2, usart3_buffer[usart3_buff_idx]);
	usart3_buff_idx ++;
	TIM_ITConfig(TIM4, TIM_IT_Update , DISABLE);
}
