#include "IAP.h"



iapfun jump2app; 
uint16_t iapbuf[1024];   

//appxaddr:Ӧ�ó������ʼ��ַ
//appbuf:Ӧ�ó���CODE.
//appsize:Ӧ�ó����С(�ֽ�).
void iap_write_appbin(uint32_t appxaddr,uint8_t *appbuf,uint32_t appsize)
{
	uint16_t t;
	uint16_t i=0;
	uint16_t temp;
	uint32_t fwaddr = appxaddr;//��ǰд��ĵ�ַ
	uint8_t *dfu = appbuf;
  
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(uint16_t)dfu[1]<<8;
		temp+=(uint16_t)dfu[0];	  
		dfu+=2;//ƫ��2���ֽ�
		iapbuf[i++]=temp;	
    
		if(i==GD32F30X_PAGE_SIZE/2)
		{
			i = 0;
			FLASH_Write(fwaddr,iapbuf,GD32F30X_PAGE_SIZE/2);	
			fwaddr+=GD32F30X_PAGE_SIZE;//ƫ��2048  
		}
    
	}
	if(i) FLASH_Write(fwaddr,iapbuf,i);//������һЩ�����ֽ�д��ȥ.  
}

//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
void iap_load_app(uint32_t appxaddr)
{
	
	if(((*(__IO uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
	{ 
		jump2app=(iapfun)*(__IO uint32_t*)(appxaddr+4);		//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
	  __set_MSP(*(__IO uint32_t*) appxaddr);				//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		jump2app();									//��ת��APP.
	}
}		 
