/*
 * packet.h
 *
 *  Created on: Dec 9, 2021
 *      Author: MVD
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#ifndef _DRV_PACKET_H_
#define _DRV_PACKET_H_
#include "buffer.h"

#define PACKET_MASK_RESTART 		0x0001
#define PACKET_MASK_FORWARD_FLOW 	0x0002
#define PACKET_MASK_DCLICK	 		0x0004
#define PACKET_MASK_CLICK	 		0x0008

// Element Types
typedef enum{
 ELEMENT_TYPE_1BYTE = 0x1, /* One BYTE Element */
 ELEMENT_TYPE_2BYTE = 0x2, /* Two BYTE Element */
 ELEMENT_TYPE_4BYTE = 0x3, /* Four BYTE Element */
 ELEMENT_TYPE_STRING = 0xFF, /* STRING Element */
}ELEMENT_Type_TypeDef;

// Element IDs
typedef enum{
 ELEMENT_ID_DIAGNOSTIC = 0x0, /* Diagnostic Data Element */
 ELEMENT_ID_FORWARDFLOWCOUNTER = 0x1,
 ELEMENT_ID_SIGNAL = 0x2,
 ELEMENT_ID_HUMIDITY = 0x3,
 ELEMENT_ID_TEMPERATURE = 0x4,
 ELEMENT_ID_ACCX = 0x5,
 ELEMENT_ID_ACCY = 0x6,
 ELEMENT_ID_ACCZ = 0x7,
 ELEMENT_ID_LIGHT12 = 0x8,
 ELEMENT_ID_LIGHT1 = 0x9,
 ELEMENT_ID_LATITUDE = 0xA,
 ELEMENT_ID_LONGITUDE = 0xB,
 }ELEMENT_ID_TypeDef;

extern Buffer_TypeDef PACKET_RXMSG;
extern Buffer_TypeDef PACKET_TXMSG;

void PACKET_Init(void);
void PACKET_Create(uint32_t mask);
void PACKET_imei2buff(char *imei);
uint16_t PACKET_getCrc16(uint8_t *buffer, uint32_t offset, uint32_t bufLen, uint16_t polynom, uint16_t preset);

void PACKET_Start(void);
void PACKET_AddElement(	ELEMENT_ID_TypeDef elid,
						ELEMENT_Type_TypeDef eltype,
						uint8_t *value,
						uint8_t len);
void PACKET_Finish(void);

#endif



#endif /* INC_PACKET_H_ */
