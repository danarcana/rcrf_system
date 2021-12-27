/*
 * buffer.h
 *
 *  Created on: Dec 9, 2021
 *      Author: MVD
 */

#ifndef INC_BUFFER_H_
#define INC_BUFFER_H_

#ifndef __BUFFERS_H__
#define __BUFFERS_H__
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


/* Memory allocation function */
#ifndef LIB_ALLOC_FUNC
#define LIB_ALLOC_FUNC    malloc
#endif

/* Memory free function */
#ifndef LIB_FREE_FUNC
#define LIB_FREE_FUNC     free
#endif

struct BufferItem
{
	uint32_t len;     /* number of bytes */
	uint8_t data[128];			/* parameter of messages */
} __attribute__((packed));
typedef struct BufferItem BufferItem_TypeDef;

struct Buffer
{
	BufferItem_TypeDef buff[32];    /*!< Pointer to pointers to strings */
	uint32_t rdi;    /* read index  */
	uint32_t wri;    /* write index  */
	uint32_t Size;     /*!< Number of all allocated pointers for strings */
}__attribute__((packed));
typedef struct Buffer Buffer_TypeDef;

//extern Buffer_TypeDef RXMSG;
//extern Buffer_TypeDef TXMSG __attribute__ ((aligned (4)));

void BUFFER_Create(Buffer_TypeDef* buffer, uint16_t size);
uint16_t BUFFER_Add(Buffer_TypeDef* buffer, BufferItem_TypeDef* data);
void BUFFER_Handler(void);
uint16_t BUFFER_Get(Buffer_TypeDef* buffer,  BufferItem_TypeDef* data);
uint16_t BUFFER_IsEmpty(Buffer_TypeDef* buffer);

#endif /* __BUFFERS_H__ */


#endif /* INC_BUFFER_H_ */
