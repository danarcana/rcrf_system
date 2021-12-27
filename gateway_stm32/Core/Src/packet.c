///*
// * packet.c
// *
// *  Created on: Dec 9, 2021
// *      Author: MVD
// */
//
//
//#include <string.h>
//#include "board.h"
//#include "configuration.h"
//#include "cmdline.h"
//#include "packet.h"
//#include "buffer.h"
//#include "nb.h"
//
//
//#define PACKET_INFO_VERSION	1	/* Version of packet structure */
//#define PACKET_NUM_OF_ELEMENTS_INDEX	22	/* Index of number of elements */
//
//
//Buffer_TypeDef PACKET_RXMSG, PACKET_TXMSG;
//
//void PACKET_Init(void)
//{
//	BUFFER_Create(&PACKET_TXMSG, 16);	//gsm message buffer
//}
//
//uint16_t PACKET_getCrc16(uint8_t *buffer, uint32_t offset, uint32_t bufLen, uint16_t polynom, uint16_t preset)
//{
//	preset &= 0xFFFF;
//	polynom &= 0xFFFF;
//	uint16_t crc = preset;
//	uint32_t i, j;
//	uint16_t data;
//	for (i = 0; i < bufLen; i++)
//	{
//		data = buffer[i + offset] & 0xFF;
//		crc ^= data;
//		for (j = 0; j < 8; j++)
//		{
//			if ((crc & 0x0001) != 0) crc = (crc >> 1) ^ polynom;
//			else crc = crc >> 1;
//		}
//	}
//	return crc & 0xFFFF;
//}
//
///*
// *
// * in: uint32_t mask
// * mask bit
// * 0 - restart flag
// */
//BufferItem_TypeDef data;
//uint16_t elements_pos, elements;
//
//void PACKET_Start(void)
//{
//	uint32_t timestamp = (uint32_t)RTC_GetTimestamp();
//
//	data.len = 0;
//	data.data[data.len++] = PACKET_INFO_VERSION;	//0
//	data.data[data.len++] = 0x00;					//1 - length
//	data.data[data.len++] = 0x00;					//2 - length
//	data.data[data.len++] = (timestamp >> 24)&0xFF;
//	data.data[data.len++] = (timestamp >> 16)&0xFF;
//	data.data[data.len++] = (timestamp >> 8)&0xFF;
//	data.data[data.len++] = (timestamp)&0xFF;
//	data.len += sprintf(&data.data[data.len],"%s", NBInfo.IMEI);
//	elements_pos = data.len;	//save position of elements count
//	data.data[data.len++] = 0;	//Quantity of elements = 0
//	elements = 0;
//}
//
//void PACKET_AddElement(	ELEMENT_ID_TypeDef elid,
//						ELEMENT_Type_TypeDef eltype,
//						uint8_t *value,
//						uint8_t len)
//{
//	data.data[data.len++] = eltype;					//8 - Element Type
//	data.data[data.len++] = len + 1;				//9 - Element Length
//	data.data[data.len++] = elid;					//10 - Element ID
//	for (int i=0; i<len; i++) data.data[data.len++] = *(value+i);
//	elements++;
//}
//
//void PACKET_UpdateElement(BufferItem_TypeDef *value)
//{
//	uint8_t i, index, elmlen;
//	uint16_t crc;
//
//	index = PACKET_NUM_OF_ELEMENTS_INDEX + 1;	//set index to first element
//	for (i=0; i<value->data[PACKET_NUM_OF_ELEMENTS_INDEX]; i++)
//	{
//		elmlen = value->data[index+1];
//		switch(value->data[index+2])
//		{
//			case ELEMENT_ID_SIGNAL: /* Signal Strength element */
//				value->data[index+3] = NBInfo.RxLevel;
//				break;
//		}
//		index = elmlen + 2;
//	}
//
//	/* Refresh signal strength */
//	crc = PACKET_getCrc16(value->data, 3, value->len-5, 0xA001, 0);
//	value->data[value->len -2] = (crc >> 8) & 0xFF;	//CRC
//	value->data[value->len -1] = (crc) & 0xFF;		//CRC
//}
//
//void PACKET_Finish(void)
//{
//	uint16_t crc, i;
//
//	data.data[elements_pos] = elements;		//7 - Quantity of elements
//	/* Update length */
//	data.data[1] = ((data.len-3) >> 8) & 0xff;		//1 - length
//	data.data[2] = ((data.len-3)) & 0xff;			//2 - length
//	crc = PACKET_getCrc16(data.data, 3, data.len-3, 0xA001, 0);
//	data.data[data.len++] = (crc >> 8) & 0xFF;	//CRC
//	data.data[data.len++] = (crc) & 0xFF;		//CRC
//	printf("TX->Buffer: ");
//	for (i=0; i<data.len; i++) printf("%02x", data.data[i]);
//	printf("\r\n");
//	BUFFER_Add(&PACKET_TXMSG, &data);
//}
