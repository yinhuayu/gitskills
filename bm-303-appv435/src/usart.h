#pragma once

#include "gd32f30x.h"

#include "foc.h"

#define RX_BUFFER_MAX_LENGTH (1024*2)



void uart1_init();

void uart_write(uint32_t uartx, uint8_t *data, uint16_t size);


void uart_send_state();


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


//foc_var_int_t fv_M1;
//foc_var_int_t fv_M2;
