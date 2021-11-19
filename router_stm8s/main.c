/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */

//#include "stm8s_spi.h"
//#include "stm8s_clk.h"
#include "iostm8s003.h"

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_clk.h"

typedef enum
{
  RC11XX_PACKET_MODE    = 0,
  RC11XX_TRANSPARENT_MODE
} rc11xx_protocol_mode_t;

typedef enum
{
  I2C_OK    = 0,
	I2C_RESET_TRIED,
  I2C_INIT_FAILED,
	I2C_SLAVE_NOT_FOUND,
	I2C_STS_PRESENT,
	I2C_SHT_PRESENT
} i2c_state_t;

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


typedef union {
		uint16_t u16; // element specifier for accessing whole u16
		int16_t i16; // element specifier for accessing whole i16
		struct 
		{
#ifdef LITTLE_ENDIAN // Byte-order is little endian
		uint8_t u8L; // element specifier for accessing low u8
		uint8_t u8H; // element specifier for accessing high u8
#else // Byte-order is big endian
		uint8_t u8H; // element specifier for accessing low u8
		uint8_t u8L; // element specifier for accessing high u8
#endif
		} s16; // element spec. for acc. struct with low or high u8
}int16Union;

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

#define RC11XX_CFG_UID0										192
#define RC11XX_CFG_UID1										44
#define RC11XX_CFG_UID2										0
#define RC11XX_CFG_UID3										77

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
  uint8_t 	uid[4];
  uint8_t   sid[4];
  uint8_t   ima_basetime;
  uint8_t   end_device_wait_cmd;
  uint8_t   end_device_wakeup_enable;
  uint8_t   indicators_on;
  uint8_t   receive_neighbour_msg;
  uint8_t 	ima_on_connect;
} rc11xx_config_mem_t;

rc11xx_config_mem_t  rc_config_mem = {0};
uint8_t SerialNumber_SHT2x[8];  //64bit serial number
int16Union sts21_temperature = {0};
float measured_temperature = 0.0f;
static uint8_t i2c_state = 0;
static uint8_t i2c_slave_address = 0;

bool rc_enter_config(void);
void rc_exit_config(void);
void rc_configure_ed(void);
void rc_hw_reset(void);
void rc_factory_reset(void);

uint8_t I2C_ClearBusyFlagErrata_2_14_7(void);
uint8_t SHT2x_SoftReset(void);


uint8_t I2C_CheckBusy(void);
uint8_t I2C_StartCondition(void);
uint8_t I2C_SendAddress(uint8_t address);
uint8_t I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(void);
float SHT2x_CalcTemperatureC(uint16_t u16sT);
uint16_t SHT2x_MeasureTempHoldMaster(void);

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


#if defined(__CSMC__)
static @inline void _delay_cycl( unsigned short __ticks )
{
/* COSMIC */  
	#define T_COUNT(x) (( x * (FCLK / 1000000UL) )-3)/3)
	// ldw X, __ticks ; insert automaticaly
	_asm("nop\n $N:\n decw X\n jrne $L\n nop\n ", __ticks);
}
#endif


void delay_ms(uint32_t ms) 
{
    uint32_t i;
//   for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
//        _asm("nop");
//    }
	for (i = 0; i < ms; i++) 
	{
			_delay_cycl((unsigned short) (T_COUNT(1000));
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
	uint8_t inChar = 0;
	
	USART_TX(RC11XX_CMD_WRITE_CONFIG_MEMORY);
	//rc_wait_for_prompt();
	while (!(UART1_SR & (1 << 5)));
	USART_TX(config_address);
	USART_TX(config_value);
	USART_TX(RC11XX_CMD_TERMINATE_CONFIG_MEMORY);
	//rc_wait_for_prompt();
	while (inChar!='>')
	{
			while (!(UART1_SR & (1 << 5))){};
			inChar = USART_RX();
	}
	delay_ms(20);
}

void rc_factory_reset(void)
{
	USART_TX('@');
	USART_TX('T');
	USART_TX('M');
}

void rc_write_config_array(uint8_t *config_array, uint8_t array_len)
{
	uint8_t inChar = 0;
	uint8_t i = 0;
	USART_TX(RC11XX_CMD_WRITE_CONFIG_MEMORY);
	//rc_wait_for_prompt();
	//while (!(UART1_SR & (1 << 5)));
	while (inChar!='>')
	{
			while (!(UART1_SR & (1 << 5))){};
			inChar = USART_RX();
	}
	for (i=0;i<array_len;i++)
	{
		USART_TX(*config_array);
		config_array++;
	}
	USART_TX(RC11XX_CMD_TERMINATE_CONFIG_MEMORY);
	while (inChar!='>')
	{
			while (!(UART1_SR & (1 << 5))){};
			inChar = USART_RX();
	}
	delay_ms(20);
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
	uint8_t config_array[30] = {0};
	uint8_t config_array_idx = 0;
	uint8_t rc_uid[4] = {0};

	rc_enter_config();
	
	rc_config_mem.data_rate = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_DATA_RATE);
	rc_config_mem.device_type = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE);
	rc_config_mem.protocol_mode = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE);
	rc_config_mem.indicators_on = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON);
	rc_config_mem.ima_on_connect = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT);
	rc_config_mem.end_device_wait_cmd = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD);
	rc_config_mem.end_device_wakeup_enable = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE);

	rc_uid[0] = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_UID0);
	rc_uid[1] = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_UID1);
	rc_uid[2] = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_UID2);
	rc_uid[3] = rc_read_config_address(RC11XX_ADDRESS_CFG_MEM_UID3);


	if (rc_config_mem.data_rate!=RC11XX_CFG_DATA_RATE)
	{
		//rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_DATA_RATE,RC11XX_CFG_DATA_RATE);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_DATA_RATE; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_DATA_RATE; config_array_idx++;
	}

	if (rc_config_mem.device_type!=RC11XX_CFG_DEVICE_TYPE)
	{
		//rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE,RC11XX_CFG_DEVICE_TYPE);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_DEVICE_TYPE; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_DEVICE_TYPE; config_array_idx++;
	}
	
	if (rc_config_mem.protocol_mode!=RC11XX_CFG_PROTOCOL_MODE)
	{
//		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE,RC11XX_CFG_PROTOCOL_MODE);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_PROTOCOL_MODE; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_PROTOCOL_MODE; config_array_idx++;
	}

	if (rc_config_mem.indicators_on!=RC11XX_INDICATORS_ON)
	{
//		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON,RC11XX_INDICATORS_OFF);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_INDICATORS_ON; config_array_idx++;
		config_array[config_array_idx] = RC11XX_INDICATORS_ON; config_array_idx++;
	}
	
	if (rc_config_mem.ima_on_connect!=RC11XX_IMA_ON_CONNECT_OFF)
	{
//		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT,RC11XX_IMA_ON_CONNECT_OFF);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_IMA_ON_CONNECT; config_array_idx++;
		config_array[config_array_idx] = RC11XX_IMA_ON_CONNECT_OFF; config_array_idx++;
	}

	//10 x 0.1 sec
	if (rc_config_mem.end_device_wait_cmd!=10)
	{
//		rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD,10);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_ED_WAIT_CMD; config_array_idx++;
		config_array[config_array_idx] = 10; config_array_idx++;
	}
	
	if (rc_config_mem.end_device_wakeup_enable!=RC11XX_CFG_WAKE_UP_SOURCE)
	{
		//rc_write_config_value(RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE,RC11XX_CFG_WAKE_UP_SOURCE);
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_WAKE_SOURCE; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_WAKE_UP_SOURCE; config_array_idx++;
	}
	if (rc_uid[0]!= RC11XX_CFG_UID0)
	{
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_UID0; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_UID0; config_array_idx++;
	}

	if (rc_uid[1]!= RC11XX_CFG_UID1)
	{
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_UID1; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_UID1; config_array_idx++;
	}
	
	if (rc_uid[2]!= RC11XX_CFG_UID2)
	{
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_UID2; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_UID2; config_array_idx++;
	}
	
	if (rc_uid[3]!= RC11XX_CFG_UID3)
	{
		config_array[config_array_idx] = RC11XX_ADDRESS_CFG_MEM_UID3; config_array_idx++;
		config_array[config_array_idx] = RC11XX_CFG_UID3; config_array_idx++;
	}
	rc_write_config_array(config_array, config_array_idx);
	delay_ms(70);
	rc_exit_config();
	//rc_hw_reset();
}
uint8_t counter_tx=0;

typedef enum{
  I2C_ADR_W   = 0x94,   // sensor I2C address + write bit
  I2C_ADR_R		= 0x95,   // sensor I2C address + read bit
	I2C_STS21_ADDR = 0x94,
	I2C_SHT21_ADDR = 0x80
}etSTS21Address;

typedef enum{
TRIG_T_MEASUREMENT_HM = 0xE3, // command trig. temp meas. hold master
TRIG_T_MEASUREMENT_POLL = 0xF3, // command trig. temp meas. no hold master
USER_REG_W = 0xE6, // command writing user register
USER_REG_R = 0xE7, // command reading user register
SOFT_RESET 	= 0xFE // command soft reset
}etSHT2xCommand;


/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define sEE_FLAG_TIMEOUT         ((uint32_t)0x5000)
#define sEE_LONG_TIMEOUT         ((uint32_t)(10 * sEE_FLAG_TIMEOUT))
__IO uint32_t  sEETimeout = sEE_LONG_TIMEOUT;


/**
  * @brief  Example of timeout situation management.
  * @param  None.
  * @retval None.
  */
uint8_t sEE_TIMEOUT_UserCallback(void)
{
  /* User application may try to recover the communication by resetting I2C
    peripheral (calling the function I2C_SoftwareResetCmd()) then re-start
    the transmission/reception from a previously stored recover point.
    For simplicity reasons, this example only shows a basic way for errors 
    managements which consists of stopping all the process and requiring system
    reset. */
  
  
  /* Block communication and all processes */
  //while (1)
  //{   
  //}  
	i2c_state = I2C_SLAVE_NOT_FOUND;
}

/** 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */
typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
}GPIO_PinState;



/* USER CODE BEGIN 1 */
/**
1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register.
2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level
(Write 1 to GPIOx_ODR).
3. Check SCL and SDA High level in GPIOx_IDR.
4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to
GPIOx_ODR).
5. Check SDA Low level in GPIOx_IDR.
6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to
GPIOx_ODR).
7. Check SCL Low level in GPIOx_IDR.
8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to
GPIOx_ODR).
9. Check SCL High level in GPIOx_IDR.
10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to
GPIOx_ODR).
11. Check SDA High level in GPIOx_IDR.
12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
13. Set SWRST bit in I2Cx_CR1 register.
14. Clear SWRST bit in I2Cx_CR1 register.
15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register.
**/
uint8_t I2C_ClearBusyFlagErrata_2_14_7(void) {

		uint8_t SDA_PIN = GPIO_PIN_4;
    uint8_t SCL_PIN = GPIO_PIN_5;
		static uint8_t resetTried = 0;
    uint8_t ret_val = I2C_OK;
		do
		{
			
		if (resetTried == 1) {
			ret_val = I2C_RESET_TRIED;
			break;
    }
// 1
		I2C_Cmd( DISABLE);

// 2
		GPIO_Init(GPIOB, SDA_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
		GPIO_Init(GPIOB, SCL_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
		GPIO_Write(GPIOB, SDA_PIN|SCL_PIN);

// 3
 //   GPIO_PinState pinState;
		if (GPIO_ReadInputPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET) {
        //for(;;){}
				ret_val = I2C_INIT_FAILED; 
				break;
    }
    if (GPIO_ReadInputPin(GPIOB, SCL_PIN) == GPIO_PIN_RESET) {
        //for(;;){}
				ret_val = I2C_INIT_FAILED;
				break;
    }
		
    // 4
		GPIO_Init(GPIOB, SDA_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);

    //HAL_GPIO_TogglePin(GPIOB, SDA_PIN);
		GPIO_WriteLow(GPIOB, SDA_PIN);

    // 5
    if (GPIO_ReadInputPin(GPIOB, SDA_PIN) == GPIO_PIN_SET) {
        //for(;;){}
				ret_val = I2C_INIT_FAILED;
				break;
    }

    // 6
    //HAL_GPIO_TogglePin(GPIOB, SCL_PIN);
		GPIO_Init(GPIOB, SCL_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
		
		GPIO_WriteLow(GPIOB, SCL_PIN);

    // 7
    if (GPIO_ReadInputPin(GPIOB, SCL_PIN) == GPIO_PIN_SET) {
        //for(;;){}
				ret_val = I2C_INIT_FAILED;
				break;
    }

    // 8
    //HAL_GPIO_WRITE_ODR(GPIOB, SDA_PIN);
		GPIO_Init(GPIOB, SDA_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
		GPIO_Write(GPIOB, SDA_PIN);

    // 9
    if (GPIO_ReadInputPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET) {
        //for(;;){}
				ret_val = I2C_INIT_FAILED;
				break;
    }

    // 10
    //GPIO_InitStruct.Pin = SCL_PIN;
    //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //HAL_GPIO_WRITE_ODR(GPIOB, SCL_PIN);
		
		GPIO_Init(GPIOB, SCL_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
		GPIO_Write(GPIOB, SDA_PIN|SCL_PIN);

    // 11
    if (GPIO_ReadInputPin(GPIOB, SCL_PIN) == GPIO_PIN_RESET) {
        //for(;;){}
				ret_val = I2C_INIT_FAILED;
				break;
    }

   // 13
   //hi2c->Instance->CR1 |= I2C_CR1_SWRST;
	 I2C->CR2 |= I2C_CR2_SWRST;

   // 14
   //hi2c->Instance->CR1 ^= I2C_CR1_SWRST;
	 I2C->CR2 ^= I2C_CR2_SWRST;

   // 15
	I2C_Cmd(ENABLE);
	resetTried = 1;
}
	while (0);
	return ret_val;
}

//void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//{
  /* Check the parameters */
  //assert_param(IS_GPIO_PIN(GPIO_Pin));

  //GPIOx->ODR |= GPIO_Pin;
//}

uint8_t I2C_CheckBusy(void)
{
	/* While the bus is busy */
  sEETimeout = sEE_LONG_TIMEOUT;
  while(I2C_GetFlagStatus( I2C_FLAG_BUSBUSY))
  {
    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
  }
}

uint8_t I2C_StartCondition(void)
{
	 /* Send START condition */
  I2C_GenerateSTART(ENABLE);
	
  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  sEETimeout = sEE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent( I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
  }
}

uint8_t I2C_SendAddress(uint8_t address)
{
  I2C_SendData(address);    //I2C address

  /* Test on EV6 and clear it */
  sEETimeout = sEE_FLAG_TIMEOUT;
	if (address&0x01)
	{
		while(!I2C_CheckEvent( I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
		}
	}
	else
	{
		while(!I2C_CheckEvent( I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
		}
	}
}

uint8_t I2C_WriteByte(uint8_t data)
{
	I2C_SendData(data);         //Command for readout on-chip memory
	
	/* Test on EV8 and clear it */
  sEETimeout = sEE_FLAG_TIMEOUT;
	//Event = I2C_GetLastEvent();
  while(!I2C_CheckEvent( I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
		//Event = I2C_GetLastEvent();
			if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
  }
}

uint8_t I2C_ReadByte(void)
{
//	while ((I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)); /* Poll on RxNE */

	sEETimeout = sEE_LONG_TIMEOUT;
  while ((I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)) /* Poll on RxNE */
  {
    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
	}
		
	return I2C_ReceiveData();
}

uint8_t I2C_StopCondition(void)
{
	I2C_GenerateSTOP(ENABLE);
}
//==============================================================================
uint8_t SHT2x_GetSerialNumber(uint8_t u8SerialNumber[])
//==============================================================================
{
  uint8_t  error=0;                          //error variable
	uint16_t Event = 0;
	
	I2C_CheckBusy();
	I2C_StartCondition();
	I2C_SendAddress(I2C_ADR_W);
	I2C_WriteByte(0xFA);
	I2C_WriteByte(0x0F);

	I2C_StartCondition();
	I2C_SendAddress(I2C_ADR_R);
	
	u8SerialNumber[5] = I2C_ReadByte(); //Read SNB_3
  I2C_ReadByte();                     //Read CRC SNB_3 (CRC is not analyzed)
  u8SerialNumber[4] = I2C_ReadByte(); //Read SNB_2
  I2C_ReadByte();                     //Read CRC SNB_2 (CRC is not analyzed)
  u8SerialNumber[3] = I2C_ReadByte(); //Read SNB_1
  I2C_ReadByte();                     //Read CRC SNB_1 (CRC is not analyzed)
  u8SerialNumber[2] = I2C_ReadByte(); //Read SNB_0
  I2C_AcknowledgeConfig(I2C_ACK_NONE);
	I2C_ReadByte();                  //Read CRC SNB_0 (CRC is not analyzed)
	
	I2C_GenerateSTOP(ENABLE);

  //Read from memory location 2
  I2C_StartCondition();
	I2C_SendAddress(I2C_ADR_W);
	I2C_WriteByte(0xFC);
	I2C_WriteByte(0xC9);
	I2C_StartCondition();
	I2C_SendAddress(I2C_ADR_R);
	I2C_AcknowledgeConfig(I2C_ACK_CURR);
  u8SerialNumber[1] = I2C_ReadByte(); //Read SNC_1
  u8SerialNumber[0] = I2C_ReadByte(); //Read SNC_0
  I2C_ReadByte();                     //Read CRC SNC0/1 (CRC is not analyzed)
  u8SerialNumber[7] = I2C_ReadByte(); //Read SNA_1
  u8SerialNumber[6] = I2C_ReadByte(); //Read SNA_0
	I2C_AcknowledgeConfig(I2C_ACK_NONE);
  I2C_ReadByte();                  //Read CRC SNA0/1 (CRC is not analyzed)
  I2C_GenerateSTOP(ENABLE);
	
  return error;
}

float SHT2x_CalcTemperatureC(uint16_t u16sT)
{
	float temperatureC; // variable for result
	u16sT &= ~0x0003; // clear bits [1..0] (status bits)
	//-- calculate temperature [°C] --
	temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
	return temperatureC;
}



uint16_t SHT2x_MeasureTempHoldMaster(void)
{
		uint8_t checksum; //checksum
		uint8_t data[2]; //data array for checksum verification
		uint8_t error=0; //error variable
		uint16_t i; //counting variable
		//-- write I2C sensor address and command --
		I2C_StartCondition();
		//error |= I2C_WriteByte(I2C_ADR_W); // I2C Adr
		error |= I2C_SendAddress(I2C_ADR_W); // I2C Adr
		error |= I2C_WriteByte(TRIG_T_MEASUREMENT_HM);
		//-- wait until hold master is released --
		I2C_StartCondition();
		error |= I2C_SendAddress(I2C_ADR_R);
//		SCL=HIGH; // set SCL I/O port as input
		for(i=0; i<100; i++) // wait until master hold is released or
		{
		  delay_ms(10);
//			DelayMicroSeconds(1000); // a timeout (~1s) is reached
//			if (SCL_CONF==1) break;
		}
		//-- check for timeout --
//		if(SCL_CONF==0) error |= TIME_OUT_ERROR;
		//-- read two data bytes and one checksum byte --
		//sts21_temperature.s16.u8H = data[0] = I2C_ReadByte(ACK);
		//sts21_temperature.s16.u8L = data[1] = I2C_ReadByte(ACK);
		
		sts21_temperature.s16.u8H = data[0] = I2C_ReadByte();
		sts21_temperature.s16.u8L = data[1] = I2C_ReadByte();
		
		//checksum=I2C_ReadByte(NO_ACK);
		checksum=I2C_ReadByte();
		//-- verify checksum --
//		error |= SHT2x_CheckCrc(data,2,checksum);
		I2C_StopCondition();
		return error;
}

uint8_t SHT2x_SoftReset(void)
{
	uint8_t error=0; //error variable
	//I2c_StartCondition();
	I2C_GenerateSTART(ENABLE);
	
	  /* Test on EV5 and clear it */
	sEETimeout = sEE_FLAG_TIMEOUT;
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
  }
	
	
	//error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	//error |= I2c_WriteByte (SOFT_RESET); // Command
	I2C_SendData (I2C_ADR_W);    //I2C address
	
  /* Send EEPROM address for write */
  //I2C_Send7bitAddress( (uint8_t)0x80, I2C_DIRECTION_TX);

  /* Test on EV6 and clear it */
  sEETimeout = sEE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent( I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
  }
	
	I2C_SendData (SOFT_RESET);    
	
	/* Test on EV8 and clear it */
  sEETimeout = sEE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent( I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
  }
	
	//I2c_StopCondition();
	I2C_GenerateSTOP(ENABLE);
	delay_ms(130); // wait till sensor has restarted
	return error;
}
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
	
	
	GPIO_DeInit(GPIOB);

	GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_HIZ_FAST);

	GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);
	
	GPIO_DeInit(GPIOC);

	GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
	
  
	I2C_DeInit();
	i2c_state = I2C_ClearBusyFlagErrata_2_14_7();
	if (i2c_state == I2C_OK)
	{
	I2C_Cmd(ENABLE);
	
	//I2C_DeInit();
		
	//I2C_Cmd( DISABLE);
	//I2C Init 
 	I2C_Init(88000, 0x200, I2C_DUTYCYCLE_16_9, I2C_ACK_CURR, I2C_ADDMODE_7BIT,(CLK_GetClockFreq() / 1000000));
	//I2C_Init(88000, 128, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT,16);
	//I2C_Init_Test();
	//I2C->CR2 |= I2C_CR2_SWRST;
	//CLK->PCKENR1 |= CLK_PCKENR1_I2C1;
	
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
	
	
	I2C_Cmd(ENABLE);
	  /* Enable Buffer and Event Interrupt*/
	I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR), ENABLE);
	//I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE);
	//delay_ms(500);
	enableInterrupts();
	
	SHT2x_SoftReset();
	SHT2x_GetSerialNumber(SerialNumber_SHT2x);
	}
//	rc_enter_config();
	
//	rc_factory_reset();
	
	rc_configure_ed();
	
	while (1) {
		delay_ms(7500);
//		_delay_cycl((unsigned short) (T_COUNT(10000));
		if (i2c_state == I2C_OK)
		{
			SHT2x_MeasureTempHoldMaster();
			measured_temperature = SHT2x_CalcTemperatureC(sts21_temperature.u16);
		}
		
		USART_TX(counter_tx);
		USART_TX(sts21_temperature.s16.u8H);
		USART_TX(sts21_temperature.s16.u8L);
		USART_TX(counter_tx);
		USART_TX(counter_tx);
		counter_tx++;
  }
}