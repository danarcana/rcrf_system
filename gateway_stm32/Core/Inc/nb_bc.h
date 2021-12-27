/*
 * nb_bc66.h
 *
 *  Created on: Nov 22, 2021
 *      Author: MVD
 */

#ifndef INC_NB_BC_H_
#define INC_NB_BC_H_

#include "ring_buffer.h"

#define NB_BC_UART		USART1

#define NB_BC_UART_RX_MEM_SIZE   256
#define NB_BC_UART_RX_BASE_ADDR  (__IO uint8_t*) nb_bc_recv_buf
#define NB_BC_UART_RX_MAX_ADDR   (__IO uint8_t*) (nb_bc_recv_buf + NB_BC_UART_RX_MEM_SIZE - 1)
#define NB_BC_UART_TX_MEM_SIZE   128


typedef enum
{
  NB_OFF = 0,
  NB_START = 1,
  NB_ENABLE_POWER = 2,
  NB_PWRKEY_OFF_PULSE = 3,
  NB_PWRKEY_ON_PULSE = 4,
  NB_INFO = 5,
  NB_ON = 6,
  NB_WAITING_TO_CONNECT = 7,
  NB_CONNECTED = 8,
  NB_STOP = 9,
  NB_DISABLE_POWER = 10,
  NB_UNRESET = 11,
  NB_WAITING_TO_IP = 12,
  NB_DISCONNECTING = 13,
  NB_GETID = 14,
} NB_StateType;

typedef enum
{
  NB_AT = 0,
  NB_AT_CCID = 1,
  NB_AT_CGSN = 2,
  NB_AT_CESQ = 3,
  NB_AT_CIMI = 4,
  NB_AT_CREGW = 5,
  NB_AT_CREG = 6,
  NB_AT_CGATT = 7,
  NB_AT_CIPMODE = 8,
  NB_AT_CSTT = 9,
  NB_AT_CIICR = 10,
  NB_AT_CIFSR = 11,
  NB_AT_CIPSTART = 12,
  NB_AT_CIPCLOSE = 13,
  NB_AT_CIPSEND = 14,
  NB_ATE0 = 15,
  NB_AT_CFUN1 = 16,
  NB_AT_QBAND = 17,
  NB_AT_COPS = 18,
  NB_AT_NSOCR = 19,
  NB_AT_NSOCL = 20,
  NB_AT_NSOST = 21,
  NB_WAIT_TO_NEUL = 22,
  NB_AT_CGPADDR = 23,
  NB_AT_QCGDEFCONT = 24,
  NB_AT_CFUN0 = 25,
  NB_ATQIOPEN = 26,
  NB_AT_CSCON = 27,
  NB_ATQICLOSE = 28,
  NB_ATQISENDEX = 29,
  NB_AT_QPOWD0 = 30,
  NB_AT_CPSMS1 = 31,
  NB_AT_QSCLK2 = 32,
  NB_AT_CCLK = 33,
  NB_AT_ENDFLAG = 255,
} NB_ATCommandType;

typedef enum
{
  AT_COMPLETE_NONE = 0,
  AT_COMPLETE_QISENDEX = 1,
  AT_COMPLETE_QIOPEN = 2,
} NB_ATCommandCompleteType;

typedef enum
{
  NB_CMD_NONE = 0,
  NB_CMD_TX = 1,
  NB_CMD_RXWAIT = 2,
  NB_CMD_TXREPEAT = 3,
  NB_CMD_RXTIMEOUT = 4,
  NB_CMD_DONE_WITHOUT_OK = 5,
  NB_CMD_DONE_WITH_OK = 6,
  NB_CMD_RXCHECKERROR = 7,
  NB_CMD_DONE = 8,
  NB_CMD_RXWAIT_TO_ANSWER = 9,
  NB_CMD_WAIT_TO_COMPLETE_COMMAND = 10,
} NB_ATCommandState;

typedef struct
{
	NB_ATCommandType Index;
	char Command[64];	/* max command size is 64 */
	char Parse[32];	/* max parse size is 32 */
	NB_ATCommandState State;	/* current state of command */
	uint8_t Retries;
	uint16_t Timeout;
	void *Value;	/* Pointer to Parsed Value */
	char Check[32];	/* max parse size is 32 */
	NB_ATCommandCompleteType CommandComplete;
}ATCommand_TypeDef;



typedef struct
{
	NB_StateType Mode;
	NB_StateType PrevMode;	/* Previous Mode of NB */
	ATCommand_TypeDef Command;
	char ICCID[28];
	char IMEI[16];
	char IMSI[28];
	char IP[28];
	char NetTime[28];	/* Network Time */
	NB_ATCommandType *Sequence;
	uint8_t SequenceIndex;
	uint8_t RxLevel;
	uint8_t CheckError;	/* 1=unexcepted value on AT command 0=OK */
	uint8_t ConnectID;
	FlagStatus SequenceDone;
}NBInfo_TypeDef;

typedef enum
{
  NB_OK = 0,
  NB_ERROR = 255,
} NB_ErrorType;


extern __IO uint8_t       nb_bc_recv_buf[NB_BC_UART_RX_MEM_SIZE];
extern __IO uint8_t       * nb_bc_recv_buf_p;
extern __IO uint32_t      nb_bc_recv_len;
extern __IO uint8_t       nb_bc_send_buf[NB_BC_UART_TX_MEM_SIZE];
extern __IO uint8_t       * nb_bc_send_buf_p;

extern __IO ring_buffer_t nb_bc_rx_ring_buffer;
extern __IO uint32_t      nb_bc_start_of_msg;
extern __IO uint32_t      nb_bc_reply_timeout;



void nb_bc_reboot(void);
void NB_Handler(void);
void NB_TimerHandler(void);
void NB_Init(void);
void NB_SaveByte(uint8_t ucByte);


#endif /* INC_NB_BC_H_ */
