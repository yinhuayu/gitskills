#ifndef __UART_H
#define __UART_H

#include "gd32f30x.h"

#define RX_BUFFER_MAX_LENGTH (1024*2)

typedef struct
{
	volatile uint8_t rx_buffer[RX_BUFFER_MAX_LENGTH];
	volatile uint16_t rx_size;
	volatile uint8_t rx_flag;
}UART_485_Frame_TypeDef;

/* global */
extern UART_485_Frame_TypeDef UART1_Frame_Box;


void UART1_Init(uint32_t baud);
void UART1_Transmit_DMA(uint8_t* pdata,uint16_t size);
void UART1_Receive_Frame(void);
void uart_send_state();

#endif
