#include "stdint.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_dma.h"
#include "stdbool.h"
#include "nb_bc.h"
#include "ring_buffer.h"
#include "main.h"


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
    LL_mDelay(1100);

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


  LL_GPIO_SetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);
  LL_mDelay(850);
  LL_GPIO_ResetOutputPin(OUT_NB_PSM_EINT_GPIO_Port, OUT_NB_PSM_EINT_Pin);

  LL_mDelay(100);
  nb_bc_power_off();
  nb_bc_power_on();


  /* delay at least 5 secs for M95 to accept AT commands */
  usart1_idle_flag = 0;
  LL_mDelay(6000);
  LL_mDelay(2000);

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
