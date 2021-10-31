/*
 * rc11xx.h
 *
 *  Created on: Oct 30, 2021
 *      Author: MVD
 */

#ifndef INC_RC11XX_H_
#define INC_RC11XX_H_
#include "stdbool.h"
#include "main.h"


#define RC11XX_CFG_PIN RC11XX_CFG_PIN_GPIO_Port, RC11XX_CFG_PIN_Pin
#define RC11XX_RST_PIN RC11XX_RESET_PIN_GPIO_Port, RC11XX_RESET_PIN_Pin

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

#define RC11XX_CFG_DATA_RATE				3
#define RC11XX_CFG_DEVICE_TYPE				RC11XX_GATEWAY
#define RC11XX_CFG_PROTOCOL_MODE			RC11XX_PACKET_MODE
//Wake up source on usart
#define RC11XX_CFG_WAKE_UP_SOURCE			4

#pragma pack(1)
typedef struct
{
  uint8_t   rf_channel;
  uint8_t   rf_power;
  uint8_t   rf_data_rate;
  uint8_t 	protocol_mode;
  uint8_t   rssi_acceptance_lvl;
  const uint8_t rssi_clear_chn;
  uint8_t   hiam_time;
  uint8_t   ima_time;
  uint8_t   connect_check_time;
  uint8_t   max_jump_lvl;
  uint8_t   max_jump_cnt;
  uint8_t   max_packet_latency;
  uint8_t   rf_tx_retry_limit;
  uint8_t   serial_port_timeout;
  uint8_t   device_type;
  const uint8_t   excellent_rssi_lvl;
  uint8_t         gpio_0_config;
  uint8_t         gpio_1_config;
  uint8_t         gpio_2_config;
  uint8_t         gpio_3_config;
  uint8_t         gpio_4_config;
  uint8_t         gpio_5_config;
  uint8_t         gpio_6_config;
  uint8_t         gpio_7_config;
  uint8_t         gpio_0_trig;
  uint8_t         gpio_1_trig;
  uint8_t         gpio_2_trig;
  uint8_t         gpio_3_trig;
  uint8_t         gpio_4_trig;
  uint8_t         gpio_5_trig;
  uint8_t         gpio_6_trig;
  uint8_t         gpio_7_trig;
  uint8_t         input_debounce;
  uint8_t         adc_0_hi_trig_hi;
  uint8_t         adc_0_hi_trig_lo;
  uint8_t         adc_0_lo_trig_hi;
  uint8_t         adc_0_lo_trig_lo;
  uint8_t         adc_0_sampling_interval;
  uint8_t         adc_1_hi_trig_hi;
  uint8_t         adc_1_hi_trig_lo;
  uint8_t         adc_1_lo_trig_hi;
  uint8_t         adc_1_lo_trig_lo;
  uint8_t         adc_1_sampling_interval;
  uint8_t         cts_hold_time;
  uint8_t         locator_enable;
  uint8_t        uid[4];
  uint8_t        sid[4];
  uint8_t         uart_baud_rate;
  uint8_t         uart_bits;
  uint8_t         uart_parity;
  uint8_t         uart_stop_bits;
  const uint8_t         reserved1;
  uint8_t         uart_flow_control;
  uint8_t         uart_buffer_full;
  char            part_number[11];
  uint8_t         fill_byte1;
  char            hw_revision[4];
  uint8_t         fill_byte2;
  char            fw_revision[4];
  uint8_t         security_level;
  uint8_t         reserved2[2];
  uint8_t         max_packet_latecy_basetime;
  uint8_t         ima_basetime;
  uint8_t         end_device_wait_cmd;
  uint8_t         end_device_wakeup_enable;
  uint8_t         config_mode_entry_ctrl;
  uint8_t			indicators_on;
  uint8_t         receive_neighbour_msg;
  uint8_t         cmd_ack;
  uint8_t         reserved3;
  uint8_t         sleep_or_rts;
  uint8_t 			ima_on_connect;
  uint8_t               pwm_default;
  uint8_t               pulse_counter_mode;
  uint8_t               pulse_counter_debounce;
  const uint8_t         connection_change_margin;
  uint8_t               clustered_mode_device_limit;
  uint8_t               clustered_node_rssi;
  uint8_t               detect_network_busy;
  uint8_t               rf_jamming_detect;
  uint8_t               rf_jamming_alarm_port;
  uint8_t               feedback_port;
  uint8_t               feedback_enable;
  uint8_t               ima_msg_data;
  uint8_t               ima_msg_address;
  uint8_t               trig_hold;
  uint8_t               end_device_awake_port;
  uint8_t               config_lock_override;
  uint8_t               reserved4[2];
  uint8_t               group_table[8];
  const uint8_t         accept_new_cmd_timeout;
  const uint8_t         command_retry;
  const uint8_t         mac_rndtime2;
  const uint8_t         mac_rndtime1;
  uint8_t               reserved5[3];
} rc11xx_config_mem_t;

void rc_hw_reset(void);

bool rc_enter_config(void);
void rc_exit_config(void);
void rc_read_config(void);
void rc_configure(void);

#endif /* INC_RC11XX_H_ */
