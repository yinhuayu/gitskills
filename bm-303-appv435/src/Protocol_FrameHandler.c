#include "Protocol_FrameHandler.h"
#include "string.h"

/* Application Frame Structure(global) */
Protocol_Frame_TypeDef Protocol_Frame_Box;


/**
  * @brief  
  * @param  
  * @param  
  * @param  
  * @retval 
  */
FRAME_StatusTypeDef Frame_Handler(UART_485_Frame_TypeDef* _frame)
{
	/* return if not a new frame */
	if (_frame->rx_flag != 1) return FRAME_WAITTING;
	_frame->rx_flag = 0;
	
	/* return if not send to this device */
	if (_frame->rx_buffer[FRAME_ID_INDEX] != DEVICE_ID) 
	{
		Frame_ReceiveAnotherOne();
		return FRAME_ERROR_ID;
	}
	
	/* frame size check */
	if( _frame->rx_size < FRAME_MIN_LEGAL_SIZE) 
	{
		Frame_ReceiveAnotherOne();
		return FRAME_ERROR_ILLEGAL;
	}
	
	/* crc check */
	uint16_t crc;
	crc = _frame->rx_buffer[ _frame->rx_size - 2 ] << 8;
	crc += _frame->rx_buffer[ _frame->rx_size - 1 ];
	if (Cal_CRC16((uint8_t*)_frame->rx_buffer, (_frame->rx_size - FRAME_TRAILER_SIZE)) != crc ) 
	{
		Frame_ReceiveAnotherOne();
		return FRAME_ERROR_CRC;
	}
	
	/* legal frame*/
	Protocol_Frame_Box.cmd = _frame->rx_buffer[FRAME_CMD_INDEX];
	Protocol_Frame_Box.data_size = _frame->rx_size - FRAME_HEADER_SIZE - FRAME_TRAILER_SIZE;
	if(Protocol_Frame_Box.data_size == 0) /* no data, a only CMD frame*/
	{
		Protocol_Frame_Box.data = (void *)0;
	}
	else
	{
		Protocol_Frame_Box.data = (uint8_t *)&_frame->rx_buffer[FRAME_DATA_INDEX];
	}
	Protocol_Frame_Box.rx_flag = 1;
	
	return FRAME_OK;
}


/**
  * @brief  
  * @param  
  * @param  
  * @param  
  * @retval 
  */
void Frame_ReceiveAnotherOne(void)
{
	UART1_Receive_Frame();
}

/**
  * @brief  
  * @param  
  * @param  
  * @param  
  * @retval 
  */
void Frame_Send(uint8_t id, uint8_t cmd, uint8_t* pdata,uint16_t size)
{
	static uint8_t _frame[2000];
	
	/* build frame */
	_frame[FRAME_ID_INDEX] = id;
	_frame[FRAME_CMD_INDEX] = cmd;
	if(size>0)memcpy(&_frame[FRAME_DATA_INDEX],pdata,size);
	
	/* add CRC */
	uint16_t _crc16 = Cal_CRC16(_frame,FRAME_HEADER_SIZE+size);
	_frame[FRAME_DATA_INDEX+size] 	= _crc16>>8;
	_frame[FRAME_DATA_INDEX+size+1] = _crc16 & 0xff;
	
	/* send frame */
	UART1_Transmit_DMA(_frame,FRAME_MIN_LEGAL_SIZE+size);
}


/**
  * @brief  Update CRC16 for input byte
  * @param  crc_in input value 
  * @param  input byte
  * @retval None
  */
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
  uint32_t crc = crc_in;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;
    if(in & 0x100)
      ++crc;
    if(crc & 0x10000)
      crc ^= 0x1021;
  }
  
  while(!(in & 0x10000));

  return crc & 0xffffu;
}

/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = p_data+size;

  while(p_data < dataEnd)
    crc = UpdateCRC16(crc, *p_data++);
 
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);

  return crc&0xffffu;
}


