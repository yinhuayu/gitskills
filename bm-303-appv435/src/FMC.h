#ifndef __FMC_H
#define __FMC_H

#include "gd32f30x.h"
//#include "MCtype.h"
#include "string.h"
#include "systick.h"

#define   GD32F30X_PAGE_SIZE      2048 //�ֽ�
#define   GD32_FLASH_BASE         0x08000000 	//GD32 FLASH����ʼ��ַ
#define   PARAMS_START_ADDR       0x08030000  //����������ʼ��ַ �����С���ܳ����˵�ַ


#define   Vision                  Real_time_get()



/*
//FLASH ��������ʼ��ַ  GD32F407RE
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  
*/

//FLASH ��������ʼ��ַ  GD32F407RE
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  

typedef struct
{
	uint16_t MC_APP_Mark;        //APP������ڱ�־ 
	int32_t MC_Code_Vision; 	   //�汾��������
	int16_t MC_EleAngle_Offset;  //�����ƫ��Ƕ�
  uint8_t MC_CAN_ID;  
} MC_Params_t;



typedef union
{
  MC_Params_t  MC_Params;
//  int32_t Data[2];
 		 
} MC_Params_union;

uint32_t FLASH_GetFlashPage(uint32_t addr);
uint32_t FLASH_ReadWord(uint32_t faddr);
uint16_t FLASH_ReadHalfWord(uint32_t faddr);
void FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint32_t NumToWrite);

uint8_t MC_Params_Load(uint32_t addr);
uint8_t MC_Params_Save(MC_Params_t *params);
MC_Params_t *Get_Params(void);
uint32_t Real_time_get(void);
uint32_t Message_ReadWrite(void);
uint8_t MC_Save_Angle_offset(int16_t angle);
int16_t MC_Load_Angle_offset(void);
uint8_t MC_Load_CAN_ID(void);
uint8_t MC_Save_CAN_ID(uint8_t id);
void iap_Func(void);
#endif
