/*
 * rc11xx.c
 *
 *  Created on: Oct 30, 2021
 *      Author: MVD
 */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_ll_gpio.h"
#include "rc11xx.h"

#define USART3_RX_BUFFER_SIZE      250

extern __IO uint8_t     usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
extern __IO uint8_t     usart3_is_msg;
extern __IO uint8_t		usart3_buff_idx;
rc11xx_config_mem_t * 	p_rc_config_mem;
volatile uint32_t RCTimeout = 250;

bool rc_enter_config(void)
{
	uint8_t ret_val = false;
//	uint8_t in_char=0;
//	uint8_t i=0;
//	rc_hw_reset();
	LL_GPIO_ResetOutputPin(RC11XX_CFG_PIN);
	usart3_is_msg= 0;
	usart3_buff_idx = 0;
	while(usart3_is_msg!=1){}

	while (usart3_rx_buffer[0]!='>'){};
	usart3_is_msg= 0;
	usart3_buff_idx = 0;
	LL_GPIO_SetOutputPin(RC11XX_CFG_PIN);
	return ret_val;
}


void rc_exit_config(void)
{
//	USART_TX(RC11XX_CMD_EXIT_CONFIG_MODE);
	LL_USART_TransmitData8(USART3, RC11XX_CMD_EXIT_CONFIG_MODE);
}


void rc_hw_reset(void)
{
	LL_GPIO_ResetOutputPin(RC11XX_RST_PIN);
	HAL_Delay(200);
	LL_GPIO_SetOutputPin(RC11XX_RST_PIN);
}

void USART3_TX(unsigned char val) {
	LL_USART_TransmitData8(USART3, val);
	while (!LL_USART_IsActiveFlag_TC(USART3)) {};
}

void rc_write_config_value(uint8_t config_address, uint8_t config_value)
{
	USART3_TX(RC11XX_CMD_WRITE_CONFIG_MEMORY);

	usart3_is_msg= 0;
	usart3_buff_idx = 0;
	while(usart3_is_msg!=1){}
	while (usart3_rx_buffer[0]!='>'){};
	usart3_is_msg= 0;
	usart3_buff_idx = 0;

	USART3_TX(config_address);
	USART3_TX(config_value);
	USART3_TX(RC11XX_CMD_TERMINATE_CONFIG_MEMORY);
	usart3_is_msg= 0;
	usart3_buff_idx = 0;
	while(usart3_is_msg!=1){}
	while (usart3_rx_buffer[0]!='>'){};
	usart3_is_msg= 0;
	usart3_buff_idx = 0;
}

void rc_read_config()
{
	LL_USART_TransmitData8(USART3, '0');
	while (!LL_USART_IsActiveFlag_TC(USART3)) {};
	while (!usart3_is_msg) {};
}

void rc_factory_reset()
{
	LL_USART_TransmitData8(USART3, '@');
	while (!LL_USART_IsActiveFlag_TC(USART3)) {};
	LL_USART_TransmitData8(USART3, 'T');
	while (!LL_USART_IsActiveFlag_TC(USART3)) {};
	LL_USART_TransmitData8(USART3, 'M');
	while (!LL_USART_IsActiveFlag_TC(USART3)) {};
}
void rc_configure()
{
	p_rc_config_mem = (rc11xx_config_mem_t *) &usart3_rx_buffer[0];
	if (p_rc_config_mem->rf_data_rate!=RC11XX_CFG_DATA_RATE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_DATA_RATE,RC11XX_CFG_DATA_RATE);
	}

	if (p_rc_config_mem->device_type!=RC11XX_CFG_DEVICE_TYPE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE,RC11XX_CFG_DEVICE_TYPE);
	}

	if (p_rc_config_mem->protocol_mode!=RC11XX_CFG_PROTOCOL_MODE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE,RC11XX_CFG_PROTOCOL_MODE);
	}

	if (p_rc_config_mem->indicators_on!=RC11XX_INDICATORS_ON)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON,RC11XX_INDICATORS_OFF);
	}

	if (p_rc_config_mem->ima_on_connect!=RC11XX_IMA_ON_CONNECT_OFF)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT,RC11XX_IMA_ON_CONNECT_OFF);
	}

	//10 x 0.1 sec
	if (p_rc_config_mem->end_device_wait_cmd!=10)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD,10);
	}

	if (p_rc_config_mem->uid[0]!=192)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_UID0,192);
	}

	if (p_rc_config_mem->uid[1]!=44)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_UID1,44);
	}

	if (p_rc_config_mem->uid[2]!=0)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_UID2,0);
	}

	if (p_rc_config_mem->uid[3]!=1)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_UID3,1);
	}


//	if (p_rc_config_mem->end_device_wakeup_enable!=RC11XX_CFG_WAKE_UP_SOURCE)
//	{
//		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE,RC11XX_CFG_WAKE_UP_SOURCE);
//	}

	HAL_Delay(170);
	rc_exit_config();
	HAL_Delay(170);
	  usart3_is_msg = 0;
	  usart3_buff_idx =0;
}

void RC_TimerHandler(void)
{
	if (RCTimeout) RCTimeout--;
}

extern __IO uint8_t   usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
extern __IO uint8_t   usart3_rx_len;
extern __IO uint8_t   usart3_is_msg;
extern __IO uint8_t   usart3_buff_idx;

void RC_Handler(void)
{
	if (RCTimeout) return;
//	  HAL_Delay(250);
	  //forward USART3 receive data on USART2
	  if (usart3_is_msg==1)
	  {
		  usart3_rx_len = usart3_buff_idx;
		  usart3_buff_idx = 0;
		  LL_USART_TransmitData8(USART2, usart3_rx_buffer[usart3_buff_idx]);
		  usart3_buff_idx ++;
		  usart3_is_msg = 0;
	  }

}
