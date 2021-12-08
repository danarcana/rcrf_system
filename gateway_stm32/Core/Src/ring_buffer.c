#include "ring_buffer.h"

ring_buffer_status_t ringbuffer_Write(volatile ring_buffer_t *buffer, uint32_t pvalue, uint32_t lvalue)
{
  uint32_t next_index = ((buffer->newest_index)+1) % MAX_POINTERS;

  if (next_index == buffer->oldest_index)
  {
    return RING_BUFFER_FULL;
  }
  buffer->data[buffer->newest_index].msg_ptr = pvalue;
  buffer->data[buffer->newest_index].msg_len = lvalue;
  buffer->newest_index = next_index;

  return RING_BUFFER_OK;
}

ring_buffer_status_t ringbuffer_Read(volatile ring_buffer_t *buffer, uint32_t *pvalue, uint32_t *lvalue)
{
  uint32_t l_oldest_index = buffer->oldest_index;

  if (buffer->newest_index == l_oldest_index)
  {
      return RING_BUFFER_EMPTY;
  }

  *pvalue = buffer->data[l_oldest_index].msg_ptr;
  *lvalue = buffer->data[l_oldest_index].msg_len;
  buffer->oldest_index = (((buffer->oldest_index)+1) % MAX_POINTERS);

  return RING_BUFFER_OK;
}
