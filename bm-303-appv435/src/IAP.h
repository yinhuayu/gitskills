#ifndef __IAP_H__
#define __IAP_H__

#include "gd32f30x.h"
#include "FMC.h"



#define  iap_write	 1	
#define  iap_jump	   2	

#define  APP_Mark          0x5555
#define  FLASH_APP1_ADDR	 0x08003000  	//Ӧ�ó�����ʼ��ַ(�����FLASH)
#define  APP_CONFIG_ADDR   0x08030000   //���������������ʼ��ַ

typedef  void (*iapfun)(void);				//����һ���������͵Ĳ���.  
void iap_load_app(uint32_t appxaddr);
void iap_write_appbin(uint32_t appxaddr,uint8_t *appbuf,uint32_t appsize);  
#endif
