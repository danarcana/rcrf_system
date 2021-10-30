/*
 * rc11xx.c
 *
 *  Created on: Oct 30, 2021
 *      Author: MVD
 */

bool rc_enter_config(void)
{
	uint8_t ret_val = FALSE;
	uint8_t in_char=0;
	uint8_t i=0;
	rc_hw_reset();
	GPIO_WriteLow(RC11XX_CFG_PIN);
	while (!(UART1_SR & (1 << 5)));


	for (i=0;i<3;i++)
	{
		in_char = USART_RX();
		if ('>'==in_char)
		{
			ret_val = TRUE;
			break;
		}
	}
	GPIO_WriteHigh(RC11XX_CFG_PIN);
	return ret_val;
}


void rc_exit_config(void)
{
	USART_TX(RC11XX_CMD_EXIT_CONFIG_MODE);
}


void rc_hw_reset(void)
{
	GPIO_WriteLow(RC11XX_RST_PIN);
	delay_ms(100);
	GPIO_WriteHigh(RC11XX_RST_PIN);
}

void rc_write_config_value(uint8_t config_address, uint8_t config_value)
{
	USART_TX(RC11XX_CMD_WRITE_CONFIG_MEMORY);
	//rc_wait_for_prompt();
	while (!(UART1_SR & (1 << 5)));
	USART_TX(config_address);
	USART_TX(config_value);
	USART_TX(RC11XX_CMD_TERMINATE_CONFIG_MEMORY);
	//rc_wait_for_prompt();
	while (!(UART1_SR & (1 << 5)));
}

uint8_t rc_read_config_address(uint8_t cfg_address)
{
	uint8_t in_char = 0;
	uint8_t ret_val = 0;
	USART_TX(RC11XX_CMD_READ_CONFIG_MEMORY);
	USART_TX(cfg_address);
	ret_val = USART_RX();
	in_char = USART_RX();		//prompt char
	return ret_val;
}

void rc_configure_ed()
{
	uint8_t config_value = 0;
	rc_enter_config();

	rc_config_mem.data_rate = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_DATA_RATE);
	rc_config_mem.device_type = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE);
	rc_config_mem.protocol_mode = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE);
	rc_config_mem.indicators_on = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON);
	rc_config_mem.ima_on_connect = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT);
	rc_config_mem.end_device_wait_cmd = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD);
	rc_config_mem.end_device_wakeup_enable = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE);

	if (rc_config_mem.data_rate!=RC11XX_CFG_DATA_RATE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_DATA_RATE,RC11XX_CFG_DATA_RATE);
	}

	if (rc_config_mem.device_type!=RC11XX_CFG_DEVICE_TYPE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE,RC11XX_CFG_DEVICE_TYPE);
	}

	if (rc_config_mem.protocol_mode!=RC11XX_CFG_PROTOCOL_MODE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE,RC11XX_CFG_PROTOCOL_MODE);
	}

	if (rc_config_mem.indicators_on!=RC11XX_INDICATORS_OFF)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON,RC11XX_INDICATORS_OFF);
	}

	if (rc_config_mem.ima_on_connect!=RC11XX_IMA_ON_CONNECT_OFF)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT,RC11XX_IMA_ON_CONNECT_OFF);
	}

	//10 x 0.1 sec
	if (rc_config_mem.end_device_wait_cmd!=10)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD,10);
	}

	if (rc_config_mem.end_device_wakeup_enable!=RC11XX_CFG_WAKE_UP_SOURCE)
	{
		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE,RC11XX_CFG_WAKE_UP_SOURCE);
	}
	delay_ms(70);
	rc_exit_config();
	//rc_hw_reset();
}
