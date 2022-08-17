#ifndef __Hardware_Init_H
#define __Hardware_Init_H





#include "gd32f30x.h"
//#include "gd32f407r_start.h"
#include "systick.h"
#include <stdio.h>


#include "Interrupt.h"

#include "CAN.h"

#include "FMC.h"

#include "IAP.h"

#include "MC_CAN_RX.h"

#include "UART.h"


typedef enum 
{
  FLASHIF_OK = 0,
  FLASHIF_ERASEKO,
	FLASHIF_ADDR_ERROR,
	FLASHIF_ERASE_ERROR,
  FLASHIF_WRITINGCTRL_ERROR,
  FLASHIF_WRITING_ERROR,
  FLASHIF_PROTECTION_ERRROR
} FLASH_IF_Status_TypeDef;

/**
	* FLASH Defination
	*/
#define FLASH_DATA_BASE_ADDRESS (0x800D800)

#define AES_KEY_FLASH_ADDRESS (FLASH_DATA_BASE_ADDRESS + 0x00)

#define DEVICE_HARDWARE_VERSION (*(__IO uint16_t*)(FLASH_DATA_BASE_ADDRESS + 0x10))

#define DEVICE_BOOTLOADER_VERSION (*(__IO uint16_t*)(FLASH_DATA_BASE_ADDRESS + 0x14))

#define DEVICE_APP_VERSION (*(__IO uint16_t*)(FLASH_DATA_BASE_ADDRESS + 0x18))

#define DEVICE_ID_ADDR ((FLASH_DATA_BASE_ADDRESS + 0x1C))

#define DEVICE_MODEL_ADDR (FLASH_DATA_BASE_ADDRESS + 0x20)

#define APP_READY (FLASH_DATA_BASE_ADDRESS + 0x30)

#define APP_A_BASE_ADDRESS (0x800E000)

#define APP_B_BASE_ADDRESS (0x8027000)

#define APP_MAX_SIZE (0x19000)












void Hardware_Init(void);
FLASH_IF_Status_TypeDef FLASH_IF_Write_Word(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);

#endif

