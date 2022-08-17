#ifndef _PROTOCOL_FRAMEHANDLER_H_
#define _PROTOCOL_FRAMEHANDLER_H_
#include "UART.h"

#define DEVICE_ID 0

#define FRAME_HEADER_SIZE 2
#define FRAME_TRAILER_SIZE 2
#define FRAME_MIN_LEGAL_SIZE (FRAME_HEADER_SIZE + FRAME_TRAILER_SIZE)
#define FRAME_ID_INDEX 0
#define FRAME_CMD_INDEX 1
#define FRAME_DATA_INDEX 2

/*
 *	Frame Handler Status
 */
typedef enum
{
  FRAME_OK      			= 0x00U,
  FRAME_ERROR_ILLEGAL = 0x01U,
  FRAME_ERROR_ID 			= 0x02U,
  FRAME_ERROR_CRC			= 0x03U,
	FRAME_WAITTING 			= 0x04U
} FRAME_StatusTypeDef;


typedef struct
{
	uint8_t cmd;
	uint8_t* data;
	uint8_t data_size;
	uint8_t rx_flag;
}Protocol_Frame_TypeDef;

extern Protocol_Frame_TypeDef Protocol_Frame_Box;

FRAME_StatusTypeDef Frame_Handler(UART_485_Frame_TypeDef* _frame);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);
void Frame_ReceiveAnotherOne(void);
void Frame_Send(uint8_t id, uint8_t cmd, uint8_t* pdata,uint16_t size);

#endif
