/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */

//#include "stm8s_spi.h"
//#include "stm8s_clk.h"
#include "iostm8s003.h"

#include "stm8s.h"
#include "stm8s_gpio.h"


typedef enum
{
  RC11XX_PACKET_MODE    = 0,
  RC11XX_TRANSPARENT_MODE
} rc11xx_protocol_mode_t;

typedef enum
{
  RC11XX_GATEWAY = 1,
  RC11XX_ROUTER,
  RC11XX_END_DEVICE
} rc11xx_device_type_t;

typedef enum
{
  RC11XX_INDICATORS_OFF = 0,
  RC11XX_INDICATORS_ON  = 255
} rc11xx_indicators_t;

typedef enum
{
  RC11XX_IMA_ON_CONNECT_OFF = 0,
  RC11XX_IMA_ON_CONNECT_ON  = 1
} rc11xx_ima_on_connect_t;


#define RC11XX_CMD_SET_SLEEP_MODE					'Z'
#define RC11XX_CMD_ALTERNATE_SLEEP_MODE		'z'
#define RC11XX_CMD_GET_VOLTAGE						'V'
#define RC11XX_CMD_GET_RSSI								'S'
#define RC11XX_CMD_SET_END_DEVICE_MODE		'N'
#define RC11XX_CMD_SET_ROUTER_MODE				'R'
#define RC11XX_CMD_GATEWAY_MODE						'G'
#define RC11XX_CMD_READ_CONFIG_MEMORY			'Y'
#define RC11XX_CMD_WRITE_CONFIG_MEMORY		'M'
#define RC11XX_CMD_TERMINATE_CONFIG_MEMORY		255
#define RC11XX_CMD_WRITE_CALIB_MEMORY		  "HW"
#define RC11XX_CMD_EXIT_CONFIG_MODE				'X'
#define RC11XX_CMD_LIST_CALIB_MEMORY			'r'
#define RC11XX_CMD_LIST_CONFIG_MEMORY			'0'
#define RC11XX_PROMPT											'>'

#define RC11XX_ADDRESS_CFG_MEM_RF_POWER			1
#define RC11XX_ADDRESS_CFG_MEM_DATA_RATE		2
#define RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE 	3
#define RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE	14

#define RC11XX_ADDRESS_CFG_MEM_UID0				45
#define RC11XX_ADDRESS_CFG_MEM_UID1				46
#define RC11XX_ADDRESS_CFG_MEM_UID2				47
#define RC11XX_ADDRESS_CFG_MEM_UID3				48

#define RC11XX_ADDRESS_CFG_MEM_SID0				49
#define RC11XX_ADDRESS_CFG_MEM_SID1				50
#define RC11XX_ADDRESS_CFG_MEM_SID2				51
#define RC11XX_ADDRESS_CFG_MEM_SID3				52

#define RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD		86
#define RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE		87
#define RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON	89
#define RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT	94

#define RC11XX_ADDRESS_CALIB_MEMORY_NID0	23
#define RC11XX_ADDRESS_CALIB_MEMORY_NID1	24
#define RC11XX_ADDRESS_CALIB_MEMORY_NID2	25
#define RC11XX_ADDRESS_CALIB_MEMORY_NID3	26

#define RC11XX_ADDRESS_CALIB_MEMORY_FDID0	27
#define RC11XX_ADDRESS_CALIB_MEMORY_FDID1	28
#define RC11XX_ADDRESS_CALIB_MEMORY_FDID2	29
#define RC11XX_ADDRESS_CALIB_MEMORY_FDID3	30

//#define RC11XX_CFG_DATA_RATE							2
#define RC11XX_CFG_DATA_RATE							3
#define RC11XX_CFG_DEVICE_TYPE						RC11XX_END_DEVICE
#define RC11XX_CFG_PROTOCOL_MODE					RC11XX_PACKET_MODE
//Wake up source on usart
#define RC11XX_CFG_WAKE_UP_SOURCE					4

#pragma pack(1)
typedef struct
{
	uint8_t   rf_channel;
  uint8_t   rf_power;
  uint8_t   data_rate;
  uint8_t 	protocol_mode;
  uint8_t   rssi_acceptance_lvl;
  uint8_t   hiam_time;
  uint8_t   ima_time;
  uint8_t   device_type;
  uint32_t 	uid;
  uint32_t  sid;
  uint8_t   ima_basetime;
  uint8_t   end_device_wait_cmd;
  uint8_t   end_device_wakeup_enable;
  uint8_t   indicators_on;
  uint8_t   receive_neighbour_msg;
  uint8_t 	ima_on_connect;
} rc11xx_config_mem_t;

rc11xx_config_mem_t  rc_config_mem = {0};

bool rc_enter_config(void);
void rc_exit_config(void);
void rc_configure_ed(void);
void rc_hw_reset(void);

void USART_TX(unsigned char val) {
  UART1_DR = val;
  while (!(UART1_SR & (1 << 6)));
}

unsigned char USART_RX(void) {
  while (!(UART1_SR & (1 << 5)));
  return UART1_DR;
}


#define RC11XX_CFG_PIN GPIOD, GPIO_PIN_4
#define RC11XX_RST_PIN GPIOD, GPIO_PIN_3

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


#define F_CPU 16000000UL
#ifndef F_CPU
#warning "F_CPU not defined, using 2MHz by default"
#define F_CPU 2000000UL
#endif

//#include <stdint.h>
void delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
        _asm("nop");
    }
}


//void delay(int ms) //Function Definition 
//{
//	int i =0 ;
//	int j=0;
//	for (i=0; i<=ms; i++)
//	{
//		for (j=0; j<120; j++) // Nop = Fosc/4
//			_asm("nop"); //Perform no operation //assembly code 
//	}
//}

void rc_wait_for_prompt(void)
{
	uint8_t inChar = 0;
	uint8_t i =0;
	//wait for prompt - 10 retries
	for (i=0;i<10;i++)
	{
		while ((UART1_SR & (1 << 5))==0){};
		inChar = USART_RX();
		if(inChar == '>')
		{
			break;
		}
	}
}

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
uint8_t counter_tx=0;
void main(void)
{
	GPIO_DeInit(GPIOD); // prepare Port D for working 
	CLK_CKDIVR = 0x00;  //set clock 16MHZ
  CLK_PCKENR1 = 0xFF;  // Enable peripherals
  UART1_BRR2 = 0x01;
	UART1_BRR1 = 0x34;  // 19200 baud
  UART1_CR3 &= ~((1 << 4) | (1 << 5)); //1 STOP BIT
  UART1_CR2 |= (1 << 2) | (1 << 3); //ENABLE TX AND RX
	GPIO_Init (RC11XX_RST_PIN, GPIO_MODE_OUT_PP_HIGH_SLOW);
	GPIO_Init (RC11XX_CFG_PIN, GPIO_MODE_OUT_PP_HIGH_SLOW);
	
	rc_configure_ed();
	
	while (1) {
		delay_ms(450);
		USART_TX(counter_tx);
		USART_TX(counter_tx);
		USART_TX(counter_tx);
		counter_tx++;
  }
}