#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "stm32f1xx.h"

#define MAX_POINTERS  16

typedef enum {RING_BUFFER_OK, RING_BUFFER_EMPTY, RING_BUFFER_FULL} ring_buffer_status_t;

typedef struct
{
  uint32_t  msg_ptr;  /* a pointer to start of message(s) */
  uint32_t  msg_len;
} tm_raw_msg_t;

typedef struct
{
  tm_raw_msg_t  data[MAX_POINTERS];
  uint32_t      newest_index;
  uint32_t      oldest_index;
} ring_buffer_t;

ring_buffer_status_t ringbuffer_Write(volatile ring_buffer_t *buffer, uint32_t pvalue, uint32_t lvalue);
ring_buffer_status_t ringbuffer_Read(volatile ring_buffer_t *buffer, uint32_t *pvalue, uint32_t *lvalue);

#endif /*__RING_BUFFER_H__*/
