#include "stdint.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_dma.h"
#include "stdbool.h"
#include "nb_bc.h"
#include "ring_buffer.h"
#include "main.h"
#include "buffer.h"

#if defined ( __GNUC__ )
#define __root
#endif

/** @brief known message texts */
__root const char __AT[]            = "AT\r\n";
__root const char __E0[]            = "ATE0\r\n";
__root const char __ATCPINQ[]       = "AT+CPIN?\r\n";
__root const char __ATCREGQ[]       = "AT+CREG?\r\n";
__root const char __ATCGREGQ[]      = "AT+CGREG?\r\n";
__root const char __ATQIFGCNT[]     = "AT+QIFGCNT=0\r\n";
__root const char __ATQICSGP[]      = "AT+QICSGP=1,\"ipfix.vodafone.ro\",\"ipfix.vodafone.ro\",\"vodafone\"\r\n";
__root const char __ATQISTATE[]     = "AT+QISTATE\r\n";
__root const char __ATQISDE[]       = "AT+QISDE=0\r\n";
__root const char __ATQILPORT[]     = "AT+QILPORT=\"TCP\",48123\r\n";
__root const char __ATQIPROMPT[]    = "AT+QIPROMPT=1\r\n";
__root const char __ATQIMUX[]       = "AT+QIMUX=0\r\n";
__root const char __ATQIMODE[]      = "AT+QIMODE=0\r\n";
__root const char __ATQIREGAPP[]    = "AT+QIREGAPP\r\n";
__root const char __ATQIACT[]       = "AT+QIACT\r\n";
__root const char __ATQILOCIP[]     = "AT+QILOCIP\r\n";
__root const char __ATQISERVER[]    = "AT+QISERVER\r\n";
__root const char __ATQISERVERQ[]   = "AT+QISERVER?\r\n";
__root const char __ATQISRVC[]      = "AT+QISRVC=2\r\n";
__root const char __ATQICLOSE[]     = "AT+QICLOSE\r\n";
__root const char __ATCSQ[]         = "AT+CSQ\r\n";


/** @brief indexes in sz_m95_replies */
#define AT_IDX_OK        (0)
#define AT_TXT_OK        "OK\0"
#define AT_IDX_READY     (1)
#define AT_TXT_READY     "READY\0"
#define AT_IDX_0_1       (2)
#define AT_TXT_0_1       ": 0,1\0"
#define AT_IDX_1__0      (3)
#define AT_TXT_1__0      ": 1, 0\0"
#define AT_IDX_1__1      (4)
#define AT_TXT_1__1      ": 1, 1\0"
#define AT_IDX_0__C      (5)
#define AT_TXT_0__C      ": 0, \0"
#define AT_IDX_REMOTEIP  (6)
#define AT_TXT_REMOTEIP  "REMOTE IP\0"
#define AT_IDX_CLOSED    (7)
#define AT_TXT_CLOSED    "CLOSED\0"
#define AT_IDX_QISERVER  (8)
#define AT_TXT_QISERVER  "QISERVER\0"
#define AT_IDX_GPRSACT   (9)
#define AT_TXT_GPRSACT   "GPRSACT\0"
#define AT_IDX_PROMPT    (10)
#define AT_TXT_PROMPT    "> \0"
#define AT_IDX_ERROR     (11)
#define AT_TXT_ERROR     "ERROR\0"
#define AT_IDX_SENDOK    (12)
#define AT_TXT_SENDOK    "SEND OK\0"
#define AT_IDX_QISACK    (13)
#define AT_TXT_QISACK    "QISACK\0"
#define AT_IDX_CSQ       (14)
#define AT_TXT_CSQ       "CSQ\0"
#define AT_IDX_IPCLOSE   (15)
#define AT_TXT_IPCLOSE   "IP CLOSE\0"
#define AT_IDX_IPIND     (16)
#define AT_TXT_IPIND     "IP IND\0"
#define AT_IDX_CGREG     (17)
#define AT_TXT_CGREG     "+CGREG\0"
#define TMSHTEMP_IDX      (18)
#define TMSHTEMP_TXT      "TMSHTEMP\0"
#define TMSHON_IDX        (19)
#define TMSHON_TXT        "TMSHON\0"
#define TMSHOFF_IDX       (20)
#define TMSHOFF_TXT       "TMSHOFF\0"

#define NB_BC_UART_RX_MEM_SIZE   256
#define NB_BC_UART_RX_BASE_ADDR  (__IO uint8_t*) nb_bc_recv_buf
#define NB_BC_UART_RX_MAX_ADDR   (__IO uint8_t*) (nb_bc_recv_buf + NB_BC_UART_RX_MEM_SIZE - 1)
#define NB_BC_UART_TX_MEM_SIZE   128


__root const char *sz_nb_bc_replies[] =
{
  AT_TXT_OK, AT_TXT_READY, AT_TXT_0_1, AT_TXT_1__0, AT_TXT_1__1, AT_TXT_0__C,
  AT_TXT_REMOTEIP, AT_TXT_CLOSED, AT_TXT_QISERVER,AT_TXT_GPRSACT, AT_TXT_PROMPT,
  AT_TXT_ERROR, AT_TXT_SENDOK, AT_TXT_QISACK, AT_TXT_CSQ, AT_TXT_IPCLOSE,
  AT_TXT_IPIND, AT_TXT_CGREG,
  TMSHTEMP_TXT, TMSHON_TXT, TMSHOFF_TXT
};

#define __ATLEN           sizeof(__AT)
//extern __root const char  __E0[];
#define __E0LEN           sizeof(__E0)
//extern __root const char  __ATCPINQ[];
#define __CPINQLEN        sizeof(__ATCPINQ)
//extern __root const char  __ATCREGQ[];
#define __CREGLEN         sizeof(__ATCREGQ)
//extern __root const char  __ATCGREGQ[];
#define __CGREQLEN        sizeof(__ATCGREGQ)
//extern __root const char  __ATQIFGCNT[];
#define __QIFGCNTLEN      sizeof(__ATQIFGCNT)
//extern __root const char  __ATQICSGP[];
#define __QICSGPLEN       sizeof(__ATQICSGP)
//extern __root const char  __ATQISTATE[];
#define __QISTATELEN      sizeof(__ATQISTATE)
//extern __root const char  __ATQISDE[];
#define __QISDELEN        sizeof(__ATQISDE)
//extern __root const char  __ATQILPORT[];
#define __QILPORTLEN      sizeof(__ATQILPORT)
//extern __root const char  __ATQIPROMPT[];
#define __QIPROMPTLEN     sizeof(__ATQIPROMPT)
//extern __root const char  __ATQIMUX[];
#define __QIMUXLEN        sizeof(__ATQIMUX)
//extern __root const char  __ATQIMODE[];
#define __QIMODELEN       sizeof(__ATQIMODE)
//extern __root const char  __ATQIREGAPP[];
#define __QIREGAPPLEN     sizeof(__ATQIREGAPP)
//extern __root const char  __ATQIACT[];
#define __QIACTLEN        sizeof(__ATQIACT)
//extern __root const char  __ATQILOCIP[];
#define __QILOCIPLEN      sizeof(__ATQILOCIP)
//extern __root const char  __ATQISERVER[];
#define __QISERVERLEN     sizeof(__ATQISERVER)
//extern __root const char  __ATQISERVERQ[];
#define __QISERVERQLEN    sizeof(__ATQISERVERQ)
//extern __root const char  __ATQISRVC[];
#define __QISRVCLEN       sizeof(__ATQISRVC)
//extern __root const char  __ATQICLOSE[];
#define __QICLOSELEN       sizeof(__ATQICLOSE)
//extern __root const char  __ATCSQ[];
#define __CSQLEN          sizeof(__ATCSQ)



__IO uint32_t  		systick_counter;

__IO ring_buffer_t nb_bc_rx_ring_buffer  = {0};
__IO uint32_t      nb_bc_start_of_msg   = 0;
__IO uint32_t      nb_bc_reply_timeout  = 0;

__IO uint8_t  nb_bc_recv_buf[NB_BC_UART_RX_MEM_SIZE] = {0};


__IO uint8_t  *nb_bc_recv_buf_p = nb_bc_recv_buf;


__IO uint32_t nb_bc_recv_len = 0;


__IO uint8_t  nb_bc_send_buf[NB_BC_UART_TX_MEM_SIZE] = {0};


__IO uint8_t  *nb_bc_send_buf_p = nb_bc_send_buf;


__IO uint16_t nb_bc_usart_send_len = 0;


static uint32_t nb_reset_to  = 0;

__IO uint8_t usart1_idle_flag = 0;

void NB_SendData(uint8_t *data, uint16_t len, uint8_t enter);

//void nb_bc_send_cmd(const char * c, uint32_t l, uint32_t to)
//{
//  to += systick_counter;
//  nb_bc_reply_timeout = to;
//
//  /* as per multibuffer operations */
//  LL_USART_ClearFlag_TC(NB_BC_UART);
//
//  /* update # of bytes to transfer */
//  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
//  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t) (c));
//  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, l);
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
//}

void USART_TX(unsigned char val) {

	  LL_USART_TransmitData8(USART1, val);
	  while (!LL_USART_IsActiveFlag_TC(USART1)) {}
//UART1_DR = val;
//  while (!(UART1_SR & (1 << 6)));
}

unsigned char USART_RX(void) {
	while (!LL_USART_IsActiveFlag_RXNE(USART1)){}
	return LL_USART_ReceiveData8(USART1);
//  while (!(UART1_SR & (1 << 5)));
//  return UART1_DR;
}


void nb_bc_send_cmd(const char * c, uint32_t l, uint32_t to)
{
  uint32_t i =0;
    to += systick_counter;
    nb_bc_reply_timeout = to;
  /* as per multibuffer operations */
  LL_USART_ClearFlag_TC(NB_BC_UART);

  for (i=1;i<l;i++)
  {
	  USART_TX(*c);
	  LL_USART_ClearFlag_TC(NB_BC_UART);
	  c++;
  }

}

ring_buffer_status_t nb_bc_wait_replay(uint32_t *p, uint32_t *l)
{
  ring_buffer_status_t ret = RING_BUFFER_EMPTY;
  uint32_t to = nb_bc_reply_timeout;

  *p = *l = 0;

  while(systick_counter < to)
  {
    ret = ringbuffer_Read(&nb_bc_rx_ring_buffer, p, l);
    if( ret == RING_BUFFER_OK)
    {
      break;
    }
  }

  return ret;
}

uint32_t nb_bc_find_text( uint32_t idx, __IO uint8_t *p_haystack, uint32_t len )
{
  uint8_t * p_find  = (uint8_t*)sz_nb_bc_replies[idx];
  uint32_t loopctrl = true;

  if( 0 == len) return false;

  while (loopctrl)
  {
    if(p_haystack > NB_BC_UART_RX_MAX_ADDR) p_haystack = NB_BC_UART_RX_BASE_ADDR;

    /* find first char match */
    while(*p_find != *p_haystack++)
    {
      if(p_haystack > NB_BC_UART_RX_MAX_ADDR) p_haystack = NB_BC_UART_RX_BASE_ADDR;
      len--;
      if(0 == len) return false;
    }

    p_find++;
    if(p_haystack > NB_BC_UART_RX_MAX_ADDR) p_haystack = NB_BC_UART_RX_BASE_ADDR;

    while(*p_find++ == *p_haystack++)
    {
      if(*p_find == 0) return true;
      if(p_haystack > NB_BC_UART_RX_MAX_ADDR) p_haystack = NB_BC_UART_RX_BASE_ADDR;

      len--;
      if(0 == len) return false;
    }

    p_find  = (uint8_t*)sz_nb_bc_replies[idx];
  }

  return false;
}


uint8_t nb_bc_start_TCP_server( void )
{
  uint32_t  c, ptr, len;
  uint32_t  gprsact_status = ERROR;

  /* select a foreground context */
  nb_bc_send_cmd(__ATQIFGCNT, __QIFGCNTLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* set bearer and APN */
  nb_bc_send_cmd(__ATQICSGP, __QICSGPLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  nb_bc_send_cmd(__ATQISTATE, __QISTATELEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* No echo for QISEND */
  nb_bc_send_cmd(__ATQISDE, __QISDELEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* set PORT */
  nb_bc_send_cmd(__ATQILPORT, __QILPORTLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* set > prompt */
  nb_bc_send_cmd(__ATQIPROMPT, __QIPROMPTLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* select single  TCP connection */
  nb_bc_send_cmd(__ATQIMUX, __QIMUXLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* set non-transparent mode */
  nb_bc_send_cmd(__ATQIMODE, __QIMODELEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  /* try to activate PDP context */
  nb_bc_send_cmd(__ATQIREGAPP, __QIREGAPPLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  nb_bc_send_cmd(__ATQIACT, __QIACTLEN, 150000);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  c = 15;
  while (--c)
  {
	nb_bc_send_cmd(__ATQISTATE, __QISTATELEN, 1000);
    if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

    if(nb_bc_find_text(AT_IDX_GPRSACT, (uint8_t *)ptr, len))
    {
      gprsact_status = SUCCESS;
      break;
    }
  }

  nb_bc_send_cmd(__ATQILOCIP, __QILOCIPLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  nb_bc_send_cmd(__ATQISERVER, __QISERVERLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  nb_bc_send_cmd(__ATQISRVC, __QISRVCLEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();

  return gprsact_status;
}

void nb_bc_power_off(void)
{
  /* status PIN inverted, that is, 1 means module is OFF */
//  if( !LL_GPIO_IsInputPinSet(M95_STATUS_GPIO_Port,M95_STATUS_Pin) )
  {
    LL_GPIO_SetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin);
    LL_mDelay(850);
    LL_GPIO_ResetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin);

    /* wait 12 seconds for logout net, see Turn-off timing (UM, p29) */
//    LL_mDelay(6000);
//    LL_mDelay(6000);

    /* wait for status OFF */
    //while (!LL_GPIO_IsInputPinSet(M95_STATUS_GPIO_Port,M95_STATUS_Pin));
  }
}


void nb_bc_power_on(void)
{
  /* status PIN inverted, that is, 0 means module is ON */
//  if( LL_GPIO_IsInputPinSet(M95_STATUS_GPIO_Port,M95_STATUS_Pin) )
  {
    LL_GPIO_SetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin);
    LL_mDelay(850);

    /* wait for status ON */
   // while (LL_GPIO_IsInputPinSet(M95_STATUS_GPIO_Port,M95_STATUS_Pin));
    LL_GPIO_ResetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin);
  }
}

void nb_bc_reboot(void)
{
  uint32_t i;
  uint32_t ptr, len;

//  LL_USART_Disable(NB_BC_UART);

//  LL_USART_Enable(NB_BC_UART);

  LL_GPIO_SetOutputPin(OUT_NB_STAT_GPIO_Port, OUT_NB_STAT_Pin);
  LL_mDelay(850);
  LL_GPIO_ResetOutputPin(OUT_NB_STAT_GPIO_Port, OUT_NB_STAT_Pin);


//  LL_GPIO_SetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);
//  LL_mDelay(150);
//  LL_GPIO_ResetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);

//  LL_mDelay(100);
  nb_bc_power_off();
//  nb_bc_power_on();
return;

  /* delay at least 5 secs for M95 to accept AT commands */
  usart1_idle_flag = 0;
//  LL_mDelay(6000);
//  LL_mDelay(7000);

//    LL_GPIO_ResetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);
//    LL_mDelay(150);
//    LL_GPIO_SetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);



//
//  LL_GPIO_SetOutputPin(OUT_NB_RST_GPIO_Port, OUT_NB_RST_Pin);
//  LL_mDelay(100);
//  LL_GPIO_ResetOutputPin(OUT_NB_RST_GPIO_Port, OUT_NB_RST_Pin);
//  LL_mDelay(2000);


//  usart1_idle_flag = 0;	//drop bootup messages

//  LL_USART_ReceiveData8(NB_BC_UART);

//  m95_recv_buf_p = m95_recv_buf;
//  m95rx_ring_buffer.newest_index = m95rx_ring_buffer.oldest_index = 0;

  /* autobaud, try 15 times  */
  i = 15;
  usart1_idle_flag = 0;
  while(--i)
  {
	nb_bc_send_cmd(__AT, __ATLEN, 300);
	LL_mDelay(20);
	if (usart1_idle_flag)
	{
//		break;
		__NOP();
		usart1_idle_flag = 0;
	}
	if( nb_bc_wait_replay(&ptr, &len) == RING_BUFFER_OK )
//	if(ringbuffer_Read(&nb_bc_rx_ring_buffer, &ptr, &len)==RING_BUFFER_OK)
	{
	  if( nb_bc_find_text(AT_IDX_OK, (uint8_t *)ptr, len) )
	  {
		break;
	  }
	}
  }

  /* autobaud failed, reboot */
  if(0 == i)
  {
	LL_mDelay(1000);
	NVIC_SystemReset();
  }

  /* no echo */
  nb_bc_send_cmd(__E0, __E0LEN, 300);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();
  if( !nb_bc_find_text(AT_IDX_OK, (uint8_t *)ptr, len) ) NVIC_SystemReset();

  /* CPIN should pe ready */
  nb_bc_send_cmd(__ATCPINQ, __CPINQLEN, 3000);
  if( nb_bc_wait_replay(&ptr, &len) != RING_BUFFER_OK ) NVIC_SystemReset();
  if( !nb_bc_find_text(AT_IDX_READY, (uint8_t *)ptr, len) ) NVIC_SystemReset();

  /* test GSM registration */
  i = 30;
  while(--i)
  {
	  nb_bc_send_cmd(__ATCREGQ, __CREGLEN, 300);
	if( nb_bc_wait_replay(&ptr, &len) == RING_BUFFER_OK )
	{
	  if( nb_bc_find_text(AT_IDX_0_1, (uint8_t *)ptr, len) )
	  {
		break;
	  }
	}
	LL_mDelay(1000);
  }

  if(0 == i)
  {
	LL_mDelay(1000);
	NVIC_SystemReset();
  }

  /* test GSM registration */
  i = 30;
  while(--i)
  {
	nb_bc_send_cmd(__ATCGREGQ, __CGREQLEN, 300);
	if( nb_bc_wait_replay(&ptr, &len) == RING_BUFFER_OK )
	{
	  if( nb_bc_find_text(AT_IDX_0_1, (uint8_t *)ptr, len) )
	  {
		break;
	  }
	}
	LL_mDelay(1000);
  }

  if(0 == i)
  {
	LL_mDelay(1000);
	NVIC_SystemReset();
  }


  nb_bc_start_TCP_server();

  nb_reset_to = 0;


}

NBInfo_TypeDef NBInfo;

#define NB_RX_TIMEOUT_IN_MS 500
volatile uint32_t NBTimeout = 0, NB10sTimer = 10000, NBReceiveIndex=0, NBRxTimeout = NB_RX_TIMEOUT_IN_MS;

ATCommand_TypeDef ATCommands[] = {
		{NB_AT, {"at"}, {""}, NB_CMD_NONE, 3, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CCID, {"at+qccid"}, {"+QCCID: %s"}, NB_CMD_NONE, 2, 3000, NBInfo.ICCID, {""}, },
		{NB_AT_CGSN, {"at+cgsn=1"}, {"+CGSN: %s"}, NB_CMD_NONE, 2, 2000, NBInfo.IMEI, {""}, AT_COMPLETE_NONE},
		{NB_AT_CFUN0, {"at+cfun=0"}, {""}, NB_CMD_NONE, 1, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CFUN1, {"at+cfun=1"}, {""}, NB_CMD_NONE, 1, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_QBAND, {"at+qband=1,20"}, {""}, NB_CMD_NONE, 1, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_COPS, {"at+cops=1,2,\"23003\""}, {""}, NB_CMD_NONE, 1, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CESQ, {"at+cesq"}, {"+CESQ: %d"}, NB_CMD_NONE, 2, 2000, &NBInfo.RxLevel, {""}, AT_COMPLETE_NONE},
		{NB_AT_CIMI, {"at+cimi"}, {"%s"}, NB_CMD_NONE, 2, 20000, NBInfo.IMSI, {""}, AT_COMPLETE_NONE},
		{NB_AT_CREGW, {"at+cereg=5"}, {""}, NB_CMD_NONE, 2, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CREG, {"at+cereg?"}, {"+CEREG: 5,5,%s"}, NB_CMD_NONE, 10, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CGATT, {"at+cgatt?"}, {"+CGATT: %s"}, NB_CMD_NONE, 100, 2000, NULL, {"1"}, AT_COMPLETE_NONE},
		{NB_AT_CSCON, {"at+cscon=0"}, {""}, NB_CMD_NONE, 100, 2000, NULL, {""}, AT_COMPLETE_NONE},
		//{NB_ATQIOPEN, {"at+qiopen=1,0,\"UDP\",192.168.0.20,4242,0,0,0"}, {"+QIOPEN: %d"}, NB_CMD_NONE, 10, 2000, &NBInfo.ConnectID, {""}, AT_COMPLETE_NONE},
		{NB_ATQIOPEN, {"at+qiopen=1,0,\"UDP\","}, {"+QIOPEN: %d"}, NB_CMD_NONE, 10, 2000, &NBInfo.ConnectID, {""}, AT_COMPLETE_QIOPEN},
		{NB_ATQISENDEX, {"at+qisendex="}, {""}, NB_CMD_NONE, 1, 2000, NULL, {""}, AT_COMPLETE_QISENDEX},
		{NB_ATQICLOSE, {"at+qiclose=0"}, {""}, NB_CMD_NONE, 10, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_ATE0, {"ATE0"}, {""}, NB_CMD_NONE, 3, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CGPADDR, {"at+cgpaddr=1"}, {"+CGPADDR: 1,%s"}, NB_CMD_NONE, 10, 2000, NBInfo.IP, {""}, AT_COMPLETE_NONE},
		{NB_AT_QCGDEFCONT, {"at+qcgdefcont=\"IP\",\"nb.m2mc\""}, {""}, NB_CMD_NONE, 1, 2000, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_QPOWD0, {"at+qpowd=0"}, {""}, NB_CMD_NONE, 3, 400, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CPSMS1, {"at+cpsms=1,,,\"00000001\",\"00000011\""}, {""}, NB_CMD_NONE, 3, 400, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_QSCLK2, {"AT+QSCLK=2"}, {""}, NB_CMD_NONE, 3, 800, NULL, {""}, AT_COMPLETE_NONE},
		{NB_AT_CCLK, {"AT+CCLK?"}, {"+CCLK: %s"}, NB_CMD_NONE, 3, 800, &NBInfo.NetTime, {""}, AT_COMPLETE_NONE},

};

NB_ATCommandType NB_Sequence_Info[] = {NB_AT, NB_ATE0, NB_AT_QSCLK2, NB_AT_CGSN, NB_AT_CCID, NB_AT_CREGW, NB_AT_CSCON, NB_AT_QBAND, NB_AT_CFUN0, NB_AT_QCGDEFCONT, NB_AT_CFUN1, NB_AT_CREG, NB_AT_ENDFLAG};
NB_ATCommandType NB_Sequence_ID[] = {NB_AT, NB_ATE0, NB_AT_CGSN, NB_AT_ENDFLAG};
NB_ATCommandType NB_Sequence_Send[] = {NB_AT_CESQ, NB_ATQISENDEX, /*NB_ATQICLOSE, */NB_AT_ENDFLAG};
NB_ATCommandType NB_Sequence_Connect[] = {NB_AT_CGATT, NB_AT_CCLK, NB_AT_CESQ, NB_ATQIOPEN, NB_AT_CPSMS1, NB_AT_ENDFLAG};
NB_ATCommandType NB_Sequence_Disconnect[] = {NB_ATQICLOSE, NB_AT_ENDFLAG};
NB_ATCommandType NB_Sequence_Wait2IP[] = {NB_AT_CGATT, NB_AT_CGPADDR, NB_AT_CESQ, NB_AT_ENDFLAG};
NB_ATCommandType NB_Sequence_PowerOFF[] = {NB_AT_QPOWD0, NB_AT_ENDFLAG};

Buffer_TypeDef NB_RXMSG;
__IO BufferItem_TypeDef NB_Msg;

Buffer_TypeDef PACKET_RXMSG, PACKET_TXMSG;
uint8_t NB_Flag_GetID = 0;
#define BSP_NB_DEST_IP_MEMADDR 0x0000	/* IP Address of Server in memory */

uint32_t NB_GetDestIP(void)
{
	uint32_t tmp = 0;
	uint8_t i, byte = 0;
	for (i=0; i< 4; i++)
	{
//		MRAM_Read(BSP_NB_DEST_IP_MEMADDR+i,  (uint8_t *)&byte, 1);
		tmp = (tmp << 8) | byte;
	}
	return tmp;
}

uint16_t NB_GetDestPort(void)
{
	uint16_t tmp = 0;
	uint8_t i, byte = 0;
	for (i=0; i< 2; i++)
	{
//		MRAM_Read(BSP_NB_DEST_PORT_MEMADDR+i,  (uint8_t *)&byte, 1);
		tmp = (tmp << 8) | byte;
	}
	return tmp;
}

NB_ErrorType NB_DoAtComplete(NB_ATCommandCompleteType complete)
{
	BufferItem_TypeDef msg;
	char txt[256]={0};
	uint8_t txtlen;
	uint32_t i, ip;
	switch (complete)
	{
	case AT_COMPLETE_QISENDEX:
		if (BUFFER_Get(&PACKET_TXMSG, &msg) != 0xFF)
		{
//			PACKET_UpdateElement(&msg);
			//Add current ConnectID
			txtlen = sprintf(txt, "%d,", NBInfo.ConnectID);
			//Add length of message
			txtlen += sprintf(&txt[txtlen], "%d,", (int)msg.len);
			for (i=0; i< msg.len; i++) txtlen += sprintf(&txt[txtlen], "%02x", msg.data[i]);
			printf("%s--->", txt);
			NB_SendData(txt, txtlen, 1); //nothing to send enter
		}
		break;
	case AT_COMPLETE_QIOPEN:
		//at+qiopen=1,0,\"UDP\",192.168.0.20,4242,0,0,0
		ip = NB_GetDestIP();
		txtlen = sprintf(txt, "%d.%d.%d.%d,%d,0,0,0", (int)((ip >> 24)&0xff), (int)((ip >> 16)&0xff), (int)((ip >> 8)&0xff), (int)(ip&0xff), NB_GetDestPort());
//		printf("%s--->", txt);
		NB_SendData(txt, txtlen, 1); //nothing to send enter
		break;
	case AT_COMPLETE_NONE:
		break;
	}
	return NB_OK;
}

/*
 * This routine decrement timout timer for NB purposes
 */
void NB_TimerHandler(void)
{
	if (NBTimeout) NBTimeout--;
	if (NBInfo.Command.Timeout) NBInfo.Command.Timeout--;
	if (NB10sTimer) NB10sTimer--;
	if (NBRxTimeout) NBRxTimeout--;
}
uint8_t NB_SeqID2Index(NB_ATCommandType id)
{
	uint8_t i=0;
	while (ATCommands[i].Index != NB_AT_ENDFLAG)
	{
		if (ATCommands[i].Index == id) return i;
		i++;
	}
	return 0;
}

void NB_SendData(uint8_t *data, uint16_t len, uint8_t enter)
{
	uint8_t delimiter[2]={'\r', '\n'};
	uint8_t i =0;


	  /* as per multibuffer operations */
	  LL_USART_ClearFlag_TC(NB_BC_UART);

	  for (i=1;i<len;i++)
	  {
		  USART_TX(*data);
		  LL_USART_ClearFlag_TC(NB_BC_UART);
		  data++;
	  }
	  USART_TX(delimiter[0]);
	  USART_TX(delimiter[1]);

//	uint8_t delimiter[2]={'\r', '\n'};
//	while(NB_UART_BUS_HANDLER.gState != HAL_UART_STATE_READY);
//	while(HAL_UART_Transmit_DMA(&NB_UART_BUS_HANDLER, data, len) == HAL_BUSY);
//	while(NB_UART_BUS_HANDLER.gState != HAL_UART_STATE_READY);
//	if (enter)
//	{
//		while(HAL_UART_Transmit_DMA(&NB_UART_BUS_HANDLER, delimiter, 1) == HAL_BUSY);
//		while(NB_UART_BUS_HANDLER.gState != HAL_UART_STATE_READY);
//	}
}

void NB_ATCommandSend(NB_ATCommandType index)
{
	uint8_t cmdindex = NB_SeqID2Index(index);
	uint8_t enter=1;

	NBInfo.Command.Index = index;
	if (NBInfo.Command.State == NB_CMD_TXREPEAT) NBInfo.Command.Retries--;
	else NBInfo.Command.Retries = ATCommands[cmdindex].Retries;
	NBInfo.Command.Timeout = ATCommands[cmdindex].Timeout;
	NBInfo.Command.Value = ATCommands[cmdindex].Value;
	NBInfo.Command.CommandComplete = ATCommands[cmdindex].CommandComplete;
	strcpy(&NBInfo.Command.Command[0], &ATCommands[cmdindex].Command[0]);
	strcpy(&NBInfo.Command.Parse[0], &ATCommands[cmdindex].Parse[0]);
	strcpy(&NBInfo.Command.Check[0], &ATCommands[cmdindex].Check[0]);
	NBInfo.CheckError = 0;
	if (NBInfo.Command.CommandComplete == AT_COMPLETE_NONE)
	{
		printf("%s--->", &NBInfo.Command.Command[0]);
		NBInfo.Command.State = NB_CMD_RXWAIT;
	}
	else
	{
		printf("%s", &NBInfo.Command.Command[0]);
		NBInfo.Command.State = NB_CMD_WAIT_TO_COMPLETE_COMMAND;
		enter = 0;
	}
	NB_SendData((uint8_t*)&NBInfo.Command.Command[0], strlen(NBInfo.Command.Command), enter);
}


NB_ErrorType NB_ATCommand_Handler(void)
{
	BufferItem_TypeDef msg;
	char txt[128];
	char *pBuff;
	uint8_t txtlen;
	uint32_t i;
	//if (NBInfo.Mode == NB_OFF) NBInfo.Mode = NB_START;
	if ((NBRxTimeout == 0) && (NBReceiveIndex))
	{
//		printf("#%d#\r\n", NBReceiveIndex);
		NB_Msg.data[NBReceiveIndex] = '\0';
		NB_Msg.len = NBReceiveIndex;
		BUFFER_Add(&NB_RXMSG, &NB_Msg);
		NBReceiveIndex = 0;
		//NBRxTimeout = NB_RX_TIMEOUT_IN_MS;
		return NB_OK;
	}
	switch(NBInfo.Command.State)
	{
	case NB_CMD_NONE:
		if(BUFFER_Get(&NB_RXMSG, &msg) != 0xFF)
		{
//			printf("%s\r\n", msg.data);
		}
		break;
	case NB_CMD_TXREPEAT:
		NB_ATCommandSend(NBInfo.Command.Index);
		break;
	case NB_CMD_RXCHECKERROR:
		if (NBInfo.Command.Timeout == 0) NBInfo.Command.State = NB_CMD_RXTIMEOUT;
		break;
	case NB_CMD_RXTIMEOUT:
		if (NBInfo.Command.Index == NB_AT_CIFSR)	//CIFSR not returning OK
		{
			NBInfo.Command.State = NB_CMD_DONE;
			break;
		}
		if (NBInfo.Command.Index == NB_AT_CIPSEND)	//NB_AT_CIPSEND not returning OK
		{
			NBInfo.Command.State = NB_CMD_DONE;
			break;
		}
		if (NBInfo.Command.Retries)
		{
			NBInfo.Command.State = NB_CMD_TXREPEAT;
		}
		else
		{
//			printf("TIMEOUT\r\n");
			NBInfo.Command.State = NB_CMD_DONE;
			return NB_ERROR;
		}

		break;
	case NB_CMD_DONE_WITH_OK:
	case NB_CMD_DONE_WITHOUT_OK:
		NBInfo.Command.State = NB_CMD_DONE;
		break;
	case NB_CMD_RXWAIT_TO_ANSWER:
		if (NBInfo.Command.Timeout == 0) NBInfo.Command.State = NB_CMD_RXTIMEOUT;
		break;
	case NB_CMD_WAIT_TO_COMPLETE_COMMAND:
		NB_DoAtComplete(NBInfo.Command.CommandComplete);
		NBInfo.Command.State = NB_CMD_RXWAIT;
		break;
	case NB_CMD_RXWAIT:
		if (NBInfo.Command.Timeout == 0) NBInfo.Command.State = NB_CMD_RXTIMEOUT;
		if(BUFFER_Get(&NB_RXMSG, &msg) != 0xFF)
		{
#if defined (NB_DEBUG)
//			printf("<%s>-%d:", msg.data, msg.len);
			for (i=0; i< msg.len; i++) printf(" %02x", msg.data[i]);
			printf("\r\n");
#endif


			printf("%s\r\n", msg.data);
			if (strcmp((char*)&msg.data[0], "OK") == 0)
			{
				if (NBInfo.Command.Index == NB_ATQIOPEN)	return NB_OK;//AT+QIOPEN returning OK and result after that
				if (NBInfo.CheckError)
				{
					NBInfo.Command.State = NB_CMD_RXCHECKERROR;
					return NB_OK;
				}
				NBInfo.Command.State = NB_CMD_DONE;
			}
			else if (strcmp((char*)&msg.data[0], "ERROR") == 0)
			{
				if (NBInfo.Command.Retries == 0) return NB_ERROR;
			}
			else if (strcmp((char*)&msg.data[0], (char*)&NBInfo.Command.Command[0]) == 0)
			{
				//Ignore Echo
			}
			else
			{
				memset(txt, 0, sizeof(txt));
				if (sscanf((char *)&msg.data[0], (char*)&NBInfo.Command.Parse[0], txt) == 1)
				{
					if (NBInfo.Command.Value != NULL)	strcpy(NBInfo.Command.Value, txt);
					if (strcmp(NBInfo.Command.Check, "") && strcmp(txt, NBInfo.Command.Check)) NBInfo.CheckError = 1;
					if (NBInfo.Command.Index == NB_ATQIOPEN)	NBInfo.Command.State = NB_CMD_DONE;
					if (NBInfo.Command.Index == NB_AT_CCLK)	//parse time from Network
					{
//						if (RTC_isValid == 0)
						{
							pBuff = (char *)NBInfo.Command.Value;
//							RTC_DateTypeDef sDate;
//							sDate.Year   = (pBuff[2] - '0')*10 + (pBuff[3] - '0');
//							pBuff = strstr(pBuff, "/") + 1;
//							sDate.Month   = (pBuff[0] - '0') * 10 + (pBuff[1] - '0');
//							pBuff = strstr(pBuff, "/") + 1;
//							sDate.Date = (pBuff[0] - '0') * 10 + (pBuff[1] - '0');
//							sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//
//							RTC_TimeTypeDef sTime;
//							pBuff = strstr(pBuff, ",") + 1;
//							sTime.Hours   = (pBuff[0] - '0') * 10 + (pBuff[1] - '0');
//							pBuff = strstr(pBuff, ":") + 1;
//							sTime.Minutes = (pBuff[0] - '0') * 10 + (pBuff[1] - '0');
//							pBuff = strstr(pBuff, ":") + 1;
//							sTime.Seconds = (pBuff[0] - '0') * 10 + (pBuff[1] - '0');
//							sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//							sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//							RTC_SetTime(sTime);
//							RTC_SetDate(sDate);
//							printf("NB> RTC Date and Time was updated...\r\n");
						}
					}
				}
			}
		}
		break;
	case NB_CMD_DONE:
		if (NBInfo.Sequence != NULL)
		{
			if (NBInfo.Sequence[NBInfo.SequenceIndex+1] == NB_AT_ENDFLAG)
			{
				NBInfo.Command.State = NB_CMD_NONE;
				NBInfo.SequenceDone = SET;
				NBInfo.Sequence = NULL;
			}
			else
			{
				NBInfo.SequenceIndex++;
				NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
			}
		}
		break;
	default:
		break;
	}

return NB_OK;
}

void NB_Init(void)
{
	BUFFER_Create(&NB_RXMSG, 32);	//gsm message buffer
//	CmdLine_RegisterCmd("nb", "s", NB_DBG, 0);// register debug command
//	CmdLine_RegisterCmd("nbi", "", NB_DBG, 1);// register debug command
//	CmdLine_RegisterCmd("set", "ss", NB_DBG, 2);// register debug command
	/* Initialize pins to default configuration */
	//BSP_NB_EN_LOW();
	//BSP_NB_RST_LOW();
	NBInfo.Mode = NB_OFF;
	NBInfo.PrevMode = NB_STOP;
	NBInfo.Sequence = NULL;
	NBInfo.Command.State = NB_CMD_NONE;
	NBInfo.IMEI[0] = 0;
	NBInfo.ConnectID = 0xff;
//	sprintf(NBInfo.IMEI, "%s", "867997030054596");
	NB_Flag_GetID = 1;
//	HAL_UART_Receive_IT(&NB_UART_BUS_HANDLER, &rxNB, 1);
	nb_bc_power_off();
}

void NB_SaveByte(uint8_t ucByte)
{
	if ((ucByte == '\r') || (NB_Msg.data[NBReceiveIndex-1] == '\r'))	//<CR><LF> detected
	//if (*ucByte == '\r')
	{
		if (NBReceiveIndex)
		{
			NB_Msg.data[NBReceiveIndex] = '\0';
			NB_Msg.len = NBReceiveIndex;
			BUFFER_Add(&NB_RXMSG, &NB_Msg);
			NBReceiveIndex = 0;
		}
		//NBReceiveIndex = 0;
	}
	else if (ucByte == '\n')
	{
		//ignore LF
	}
	else
	{
		NB_Msg.data[NBReceiveIndex++] = ucByte;

	}
	NBRxTimeout = NB_RX_TIMEOUT_IN_MS;
//	HAL_UART_Receive_IT(&NB_UART_BUS_HANDLER, &rxNB, 1);
}

void NB_Handler(void)
{

	NB_StateType mode=NBInfo.Mode;
	if (NBTimeout) return;
//	if (NBInfo.PrevMode != NBInfo.Mode) printf("NB MODE=%02d\r\n", NBInfo.Mode);
	switch(NBInfo.Mode)
	{
	case NB_OFF:
		NB_ATCommand_Handler();
		NBTimeout = 0;
		if (BUFFER_IsEmpty(&PACKET_TXMSG)) NBInfo.Mode = NB_OFF;
		else NBInfo.Mode = NB_START;
		if (NB_Flag_GetID == 1) NBInfo.Mode = NB_START;
		break;
	case NB_START:
		NBTimeout = 0;	//nothing to wait
		NBInfo.Mode = NB_ENABLE_POWER;
		break;
	case NB_ENABLE_POWER:
		NBTimeout = 100;	//wait to enable power
		NBInfo.Mode = NB_PWRKEY_ON_PULSE;
	    LL_GPIO_SetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin); //set PWRKEY to default state
		break;
	case NB_PWRKEY_ON_PULSE:
		NBTimeout = 600;	//POWERKEY is required to be low >500ms
		LL_GPIO_ResetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin); //set PWRKEY to default state
		NBInfo.Mode = NB_PWRKEY_OFF_PULSE;
		break;
	case NB_PWRKEY_OFF_PULSE:
	    LL_GPIO_SetOutputPin(OUT_NB_PWRKEY_GPIO_Port, OUT_NB_PWRKEY_Pin); //set PWRKEY to default state
//		printf("NB> device is running.\r\n");
		if (NB_Flag_GetID == 1) NBInfo.Mode = NB_GETID;
		else NBInfo.Mode = NB_INFO;
		break;
	case NB_GETID:
//		printf("GetID Sequence Started\r\n");
		NBInfo.Mode = NB_ON;
		NBInfo.Sequence = NB_Sequence_ID;
		NBInfo.SequenceIndex = 0;
		NBInfo.SequenceDone = RESET;
		NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
		break;
	case NB_INFO:
		printf("Info Sequence Started\r\n");
		NBInfo.Mode = NB_ON;
		NBInfo.Sequence = NB_Sequence_Info;
		NBInfo.SequenceIndex = 0;
		NBInfo.SequenceDone = RESET;
		NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
		break;
	case NB_ON:
		if (NB_ATCommand_Handler() == NB_ERROR)
		{
			NBInfo.Mode = NB_STOP;
			break;
		}
		if (NBInfo.SequenceDone == SET)
		{
			if (NB_Flag_GetID == 1)
			{
				NB_Flag_GetID = 0;
				NBInfo.Mode = NB_STOP;
				break;
			}
			NBInfo.Sequence = NB_Sequence_Connect;
			NBInfo.SequenceIndex = 0;
			NBInfo.SequenceDone = RESET;
			NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
			NBInfo.Mode = NB_WAITING_TO_CONNECT;
		}
		break;
	case NB_WAITING_TO_CONNECT:
		if (NB_ATCommand_Handler() == NB_ERROR)
		{
			NBInfo.Mode = NB_STOP;
			break;
		}
		if (NBInfo.SequenceDone == SET)
		{
			NBInfo.Sequence = NB_Sequence_Wait2IP;
			NBInfo.SequenceIndex = 0;
			NBInfo.SequenceDone = RESET;
			NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
			NBInfo.Mode = NB_WAITING_TO_IP;
		}
		break;
	case NB_WAITING_TO_IP:
			if (NB_ATCommand_Handler() == NB_ERROR)
			{
				NBInfo.Mode = NB_STOP;
				break;
			}
			if (NBInfo.SequenceDone == SET)
			{
				if (strcmp((char*)&NBInfo.IP, "0") == 0)
				{
					NBInfo.Sequence = NB_Sequence_Wait2IP;
					NBInfo.SequenceIndex = 0;
					NBInfo.SequenceDone = RESET;
					NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
					NBInfo.Mode = NB_WAITING_TO_IP;
					NBTimeout = 1000;	//wait to next step
				}
				else NBInfo.Mode = NB_CONNECTED;
			}
			break;
	case NB_CONNECTED:
		if (NB_ATCommand_Handler() == NB_ERROR)
		{
			NBInfo.Mode = NB_DISCONNECTING;
			NBInfo.Sequence = NB_Sequence_Disconnect;
			NBInfo.SequenceIndex = 0;
			NBInfo.SequenceDone = RESET;
			NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
			NBTimeout = 1000;	//wait to transmit
			break;
		}
		if (NBInfo.SequenceDone == SET)
		{
			/*if (BUFFER_IsEmpty(&PACKET_TXMSG))
			{
				NBInfo.Mode = NB_STOP;
				NBTimeout = 1000;	//wait to transmit
			}*/
			//if (NB_10sExpired() == SET)
			//else
			if (!BUFFER_IsEmpty(&PACKET_TXMSG))
			{
				NBInfo.Sequence = NB_Sequence_Send;
				NBInfo.SequenceIndex = 0;
				NBInfo.SequenceDone = RESET;
				NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
				NBTimeout = 1000;	//wait to transmit
			}
		}
		break;
	case NB_DISCONNECTING:
		if (NB_ATCommand_Handler() == NB_ERROR)
		{
			NBInfo.Mode = NB_STOP;
			break;
		}
		if (NBInfo.SequenceDone == SET)
		{
			NBInfo.Mode = NB_STOP;
		}
		break;
	case NB_STOP:
		NBInfo.Sequence = NB_Sequence_PowerOFF;
		NBInfo.SequenceIndex = 0;
		NBInfo.SequenceDone = RESET;
		NB_ATCommandSend(NBInfo.Sequence[NBInfo.SequenceIndex]);
		NBTimeout = 1100;	//wait to transmit
		NBInfo.Mode = NB_DISABLE_POWER;
		break;
	case NB_DISABLE_POWER:
		if (NB_ATCommand_Handler() == NB_ERROR)
		{
			NBInfo.Mode = NB_OFF;
			break;
		}
		NBTimeout = 100;
		if (NBInfo.SequenceDone == SET) NBInfo.Mode = NB_OFF;
		break;
	default:
		break;
	}
	NBInfo.PrevMode = mode;

}
