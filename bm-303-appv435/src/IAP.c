#include "IAP.h"



iapfun jump2app; 
uint16_t iapbuf[1024];   

//appxaddr:应用程序的起始地址
//appbuf:应用程序CODE.
//appsize:应用程序大小(字节).
void iap_write_appbin(uint32_t appxaddr,uint8_t *appbuf,uint32_t appsize)
{
	uint16_t t;
	uint16_t i=0;
	uint16_t temp;
	uint32_t fwaddr = appxaddr;//当前写入的地址
	uint8_t *dfu = appbuf;
  
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(uint16_t)dfu[1]<<8;
		temp+=(uint16_t)dfu[0];	  
		dfu+=2;//偏移2个字节
		iapbuf[i++]=temp;	
    
		if(i==GD32F30X_PAGE_SIZE/2)
		{
			i = 0;
			FLASH_Write(fwaddr,iapbuf,GD32F30X_PAGE_SIZE/2);	
			fwaddr+=GD32F30X_PAGE_SIZE;//偏移2048  
		}
    
	}
	if(i) FLASH_Write(fwaddr,iapbuf,i);//将最后的一些内容字节写进去.  
}

//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(uint32_t appxaddr)
{
	
	if(((*(__IO uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
	{ 
		jump2app=(iapfun)*(__IO uint32_t*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
	  __set_MSP(*(__IO uint32_t*) appxaddr);				//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.
	}
}		 
