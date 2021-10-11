/*
 * usart_cb.c
 *
 *  Created on: Oct 1, 2021
 *      Author: MVD
 */


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
//  /* Read Received character. RXNE flag is cleared by reading of DR register */
//  aUSART1RxBuffer[ubReceive++] = LL_USART_ReceiveData8(USART1);
//
//  if (ubReceive == ubUSART1NbDataToReceive)
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
