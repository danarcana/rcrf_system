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


extern __IO uint8_t       nb_bc_recv_buf[NB_BC_UART_RX_MEM_SIZE];
extern __IO uint8_t       * nb_bc_recv_buf_p;
extern __IO uint32_t      nb_bc_recv_len;
extern __IO uint8_t       nb_bc_send_buf[NB_BC_UART_TX_MEM_SIZE];
extern __IO uint8_t       * nb_bc_send_buf_p;

extern __IO ring_buffer_t nb_bc_rx_ring_buffer;
extern __IO uint32_t      nb_bc_start_of_msg;
extern __IO uint32_t      nb_bc_reply_timeout;



void nb_bc_reboot(void);


#endif /* INC_NB_BC_H_ */
