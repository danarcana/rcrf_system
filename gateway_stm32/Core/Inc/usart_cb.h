/*
 * usart_cb.h
 *
 *  Created on: Oct 1, 2021
 *      Author: MVD
 */

#ifndef INC_USART_CB_H_
#define INC_USART_CB_H_


void USART_CharReception_Callback(void);

/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART_TXEmpty_Callback(void);

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART_CharTransmitComplete_Callback(void);

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void USART_TransferError_Callback(void);

#endif /* INC_USART_CB_H_ */
