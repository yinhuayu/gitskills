#ifndef __MC_CAN_RX_H
#define __MC_CAN_RX_H

#include "gd32f30x.h"

#define CAN_REC_LEN  			       2*1024 	//定义最大接收字节数  


#define CAN_ID_Drive1	 	         0x32
#define CAN_ID_Drive2	           0x33

#define CAN_ID_Updating	         0x100
#define CAN_ID_Updatefinish	     0x101
#define CAN_ID_Jumpapp	         0x102
#define CAN_ID_UpdateAPP	       0x103
#define CAN_ID_Angleoffset	     0x104
#define CAN_ID_MC_Mode_Set	     0x105
#define CAN_ID_Feback_Mode_Set	 0x106
#define CAN_ID_Look_Mode_Set	   0x107
#define CAN_ID_Set_State	       0x108
#define CAN_ID_Resistor_ONOFF	   0x109

#define CAN_ID_Drive_FB	 	       0x96
#define CAN_ID_Mileage_FB	 	     0xFB
#define CAN_ID_Feback_Mode_FB	   0x264
#define CAN_ID_MC_Mode_Set_FB	   0x200

typedef struct 
{
    uint8_t can_receive_flag;
	 
} CAN_RX_Date_t;

extern CAN_RX_Date_t  CAN_RX_Date;
extern uint16_t CAN_RX_CNT;
extern uint8_t CAN_RX_BUF[CAN_REC_LEN];
extern uint8_t Onebuff_flat;

CAN_RX_Date_t *Get_CAN_RX_Date_Addr(void);

uint32_t StdId_cb(void);
uint8_t crc_check(uint16_t size);
void CAN_RX_Deal(void);

#endif 
