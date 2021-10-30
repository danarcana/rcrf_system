/*
 * rc11xx.h
 *
 *  Created on: Oct 30, 2021
 *      Author: MVD
 */

#ifndef INC_RC11XX_H_
#define INC_RC11XX_H_


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
#define RC11XX_CFG_DATA_RATE							5
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


#endif /* INC_RC11XX_H_ */
