/*
 * buffer.c
 *
 *  Created on: Dec 9, 2021
 *      Author: MVD
 */


#include "buffer.h"

//Buffer_TypeDef RXMSG;

void BUFFER_Create(Buffer_TypeDef* buffer, uint16_t size) {

	/* Set default settings */
	buffer->Size = size;
	buffer->rdi = 0;
	buffer->wri = 0;
}

uint16_t BUFFER_Add(Buffer_TypeDef* buffer, BufferItem_TypeDef* data) {
	/* Check if slot is available */
	if ((buffer->wri + 1) % buffer->Size == buffer->rdi) return 0xFF;
	/* Copy content */
	buffer->buff[buffer->wri] = *data;
	/* Increase count */
	buffer->wri = (buffer->wri + 1) % buffer->Size;
	/* Return pointer */
	return (buffer->wri);
}


uint16_t BUFFER_Get(Buffer_TypeDef* buffer,  BufferItem_TypeDef* data)
{
	uint32_t index = buffer->rdi;
	/* Check input pointer */
	if (buffer == NULL) return 0xFF;
	/* Check if memory available */
	if (buffer->rdi != buffer->wri)
	{
		buffer->rdi = (buffer->rdi + 1) % buffer->Size;
		/* Copy content */
		*data = buffer->buff[index];
		//buffer->rdi = (buffer->rdi + 1) % buffer->Size;
		return index;
	}
	return 0xFF;
}

uint16_t BUFFER_IsEmpty(Buffer_TypeDef* buffer)
{
	/* Check input pointer */
	if (buffer == NULL) return 1;
	/* Check if memory available */
	if (buffer->rdi != buffer->wri) return 0;
	return 1;
}



