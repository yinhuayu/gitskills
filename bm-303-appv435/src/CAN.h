#ifndef __CAN_H
#define __CAN_H

#include "gd32f30x.h"
#include "FMC.h"

#define CAN_ID_UpdateAPP	       0x103

void CAN_Config(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp);
uint8_t Can_Send_Msg(uint32_t can_periph , uint32_t StdId, char* msg, uint8_t len);

#endif
