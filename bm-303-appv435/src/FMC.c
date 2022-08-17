#include "FMC.h"

MC_Params_t MC_Params_u =
{
  .MC_Code_Vision      = 2022010319,
  .MC_APP_Mark         = 0x5555,
  .MC_EleAngle_Offset  = 0xBB55, 
  .MC_CAN_ID           = 1,   
};

MC_Params_t MC_Params_temp = {0};


/*******************************************
* Function Name : FLASH_ReadHalfWord
* Description   : 读取半字
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : None
*******************************************/
uint16_t FLASH_ReadHalfWord(uint32_t faddr)
{
	return *(__IO uint16_t*)(faddr); 
}

/*******************************************
* Function Name : FLASH_ReadHalfWord
* Description   : 读取字
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : None
*******************************************/
uint32_t FLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)(faddr); 
}


/*******************************************
* Function Name : FLASH_GetFlashPage
* Description   : 获取某个地址的扇区
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : None
*******************************************/
uint32_t FLASH_GetFlashPage(uint32_t addr)
{
    uint32_t Page_addr;
  	uint32_t offaddr; 
  
    offaddr = addr - GD32_FLASH_BASE;		        //实际偏移地址.  
    Page_addr = (offaddr/GD32F30X_PAGE_SIZE)*GD32F30X_PAGE_SIZE + GD32_FLASH_BASE;//页首地址  0~127页 for GD32F303RCT6  

    return Page_addr;	
}



/*******************************************
* Function Name : FLASH_Write
* Description   : 从指定地址开始写入指定长度的数据
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : GD32F4只能按扇区擦除或全片擦除，需注意写入新数据时是否会把其他数据擦除
                  如果发现写入地址的FLASH没有被擦除，数据将不会写入
*******************************************/
void FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint32_t NumToWrite)	
{ 
    fmc_state_enum status = FMC_READY;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;	

    if(WriteAddr<GD32_FLASH_BASE||WriteAddr%2)return;	//非法地址

    fmc_unlock();	//解锁 
      
    addrx = WriteAddr;//写入的起始地址

    endaddr = WriteAddr + NumToWrite*2;	//写入的结束地址

    if(addrx<0X1FFF0000)			//判断是否在主存储区内,否则不能擦写
    {
      while(addrx<endaddr)		
      {   
        status = fmc_page_erase(FLASH_GetFlashPage(addrx));//按页擦除
        if(status!=FMC_READY)break;	//发生错误
        addrx += 0x800;
      } 
    }
    if(status==FMC_READY)
    {
      while(WriteAddr<endaddr)//写数据
      {
        if(fmc_halfword_program(WriteAddr,*pBuffer)!=FMC_READY)//写入数据,半字（16位）
        { 
          break;	//写入异常
        }
        
        WriteAddr += 2;
        pBuffer++;
      } 
    }

    fmc_lock();//上锁
} 

/*******************************************
* Function Name : FLASH_Read
* Description   : 从指定地址开始读出指定长度的数据
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : None
*******************************************/
void FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint32_t NumToRead)   	
{
    uint32_t i;
    for(i=0;i<NumToRead;i++)
    {
      pBuffer[i] = FLASH_ReadHalfWord(ReadAddr);//读取2个字节.
      ReadAddr += 2;//偏移2个字节.	
    }
}

/*******************************************
* Function Name : MC_Params_Load
* Description   : 从内存加载相关参数
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月13日11:07:04
* Notes         : None
*******************************************/

uint8_t MC_Params_Load(uint32_t addr)
{
    MC_Params_union  params_union;
  
    unsigned char *ptr = (unsigned char *)addr;
  
    memcpy(&params_union,ptr,sizeof(MC_Params_union));//从flash加载
    memcpy(&MC_Params_temp,&params_union.MC_Params,sizeof(MC_Params_t));//拷贝
  
    return 1;
}

/*******************************************
* Function Name : MC_Params_Save
* Description   : 写入相关参数到内存
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月13日11:07:04
* Notes         : None
*******************************************/

uint8_t MC_Params_Save(MC_Params_t *params)
{
    MC_Params_union  params_union;
  
    memset(&params_union,0,sizeof(MC_Params_union));//清空
    memcpy(&params_union.MC_Params,params,sizeof(MC_Params_t));//拷贝
  
    FLASH_Write(PARAMS_START_ADDR,(uint16_t *)&params_union,sizeof(MC_Params_union)); 
  
    return 1;
}

/*******************************************
* Function Name : *Get_Params
* Description   : 返回地址
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月13日11:07:04
* Notes         : None
*******************************************/
MC_Params_t *Get_Params(void)
{
    return &MC_Params_temp;

}

/*******************************************
* Function Name : Real_time_get
* Description   : 实时时间获取，可作为软件版本
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月16日15:15:51
* Notes         : None
*******************************************/
uint32_t Real_time_get(void)
{
	
    const uint8_t Month_Table[12][3]= {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    uint8_t i;
    uint8_t *_date;
    uint8_t *_time;
    uint8_t mon,day; 
    uint8_t hour,minute; 
    uint16_t year;

    _date =  __DATE__;
    _time =  __TIME__; 
    
    hour  = (_time[0] - '0') * 10 + (_time[1] - '0');//时
    
    if(_time[2] == ':')
    {

      minute = (_time[3] - '0') * 10 + (_time[4] - '0');//分
    }
    
    for(i=0; i<12; i++)
    {
      if((Month_Table[i][0] == _date[0]) && (Month_Table[i][1] == _date[1]) && (Month_Table[i][2] == _date[2]))
      {
        mon = i+1;
      }      
    }
    if(_date[4]==' ')//日
    {
      day = _date[5]-'0';
    }
    else
    {
      day = 10*(_date[4]-'0')+_date[5]-'0';
    }

    year = 10*(_date[9]-'0')+_date[10]-'0';//年 
    
    return (year*100000000 + mon*1000000 + day*10000 + hour * 100 + minute);//2021年9月16日15:16 --> 2109161516
}

/*******************************************
* Function Name : Message_ReadWrite
* Description   : 读或写信息到flash
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月16日17:22:00
* Notes         : None
*******************************************/
uint32_t Message_ReadWrite(void)
{
    static uint8_t Message_status;
  
    MC_Params_Load(PARAMS_START_ADDR);//加载存储的参数
  
    if(MC_Params_temp.MC_Code_Vision!=MC_Params_u.MC_Code_Vision)//版本与存储的不一致代表版本进行了更新或没有保存版本号
    {
      MC_Params_temp.MC_Code_Vision = MC_Params_u.MC_Code_Vision;
      MC_Params_Save(Get_Params());//保存
      delay_1ms(1);
      
      Message_status = 1;
    }

    if(MC_Params_temp.MC_APP_Mark!=MC_Params_u.MC_APP_Mark)//APP mark
    {
      MC_Params_temp.MC_APP_Mark = MC_Params_u.MC_APP_Mark;
      MC_Params_Save(Get_Params());//保存
      delay_1ms(1);
      
      Message_status = 2;
    }    

    if(MC_Params_temp.MC_EleAngle_Offset==(int16_t)0xFFFF)//没有电角度零偏值
    {
      Message_status = 3;
    }

    
    return Message_status;
}

/*******************************************
* Function Name : MC_Load_CAN_ID
* Description   : 加载电机/驱动器ID
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年11月15日17:59:06
* Notes         : None
*******************************************/
uint8_t MC_Load_CAN_ID(void)
{
  if(MC_Params_temp.MC_CAN_ID==0xff) MC_Params_temp.MC_CAN_ID = 1;
    
  return MC_Params_temp.MC_CAN_ID;

}


/*******************************************
* Function Name : MC_Save_CAN_ID
* Description   : 保存设置的ID
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年11月15日17:59:01
* Notes         : None
*******************************************/
uint8_t MC_Save_CAN_ID(uint8_t id)
{
    static uint8_t can_state;
   
    if(MC_Params_temp.MC_CAN_ID!=id)//设置的ID与存储的不一致
    {
      MC_Params_Load(PARAMS_START_ADDR);//加载存储的参数   
      
      MC_Params_temp.MC_CAN_ID = id;
      MC_Params_Save(Get_Params());//保存 

      MC_Params_Load(PARAMS_START_ADDR);//再次读取     
    }  
    
    if(MC_Load_CAN_ID()==id) can_state = 1;
    else can_state = 0;    

    return can_state;    
}

/*******************************************
* Function Name : MC_Load_Angle_offset
* Description   : 加载电角度零偏
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年11月15日17:59:01
* Notes         : None
*******************************************/
int16_t MC_Load_Angle_offset(void)
{
    return MC_Params_temp.MC_EleAngle_Offset;
}


/*******************************************
* Function Name : MC_Save_Angle_offset
* Description   : 保存电角度零偏
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年11月15日17:59:01
* Notes         : None
*******************************************/
uint8_t MC_Save_Angle_offset(int16_t angle)
{
    static uint8_t offset_state;
  
    if(MC_Params_temp.MC_EleAngle_Offset!=angle)
    {
      MC_Params_Load(PARAMS_START_ADDR);//加载存储的参数

      MC_Params_temp.MC_EleAngle_Offset = angle;

      MC_Params_Save(Get_Params());//保存 

      MC_Params_Load(PARAMS_START_ADDR);//加载存储的参数    

    }

    
    if(MC_Params_temp.MC_EleAngle_Offset==angle) offset_state = 1;
    else offset_state = 0; 
    
    return  offset_state;   
}


/*******************************************
* Function Name : iap_Func
* Description   : 跳转到iap
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : None
*******************************************/
void iap_Func(void)
{

    MC_Params_Load(PARAMS_START_ADDR);//加载存储的参数 

    MC_Params_temp.MC_APP_Mark = 0xFFFF;
    MC_Params_Save(Get_Params());//保存
  
    NVIC_SystemReset();
}
