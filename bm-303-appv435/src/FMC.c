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
* Description   : ��ȡ����
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��10��15:07:02
* Notes         : None
*******************************************/
uint16_t FLASH_ReadHalfWord(uint32_t faddr)
{
	return *(__IO uint16_t*)(faddr); 
}

/*******************************************
* Function Name : FLASH_ReadHalfWord
* Description   : ��ȡ��
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��10��15:07:02
* Notes         : None
*******************************************/
uint32_t FLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)(faddr); 
}


/*******************************************
* Function Name : FLASH_GetFlashPage
* Description   : ��ȡĳ����ַ������
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��10��15:07:02
* Notes         : None
*******************************************/
uint32_t FLASH_GetFlashPage(uint32_t addr)
{
    uint32_t Page_addr;
  	uint32_t offaddr; 
  
    offaddr = addr - GD32_FLASH_BASE;		        //ʵ��ƫ�Ƶ�ַ.  
    Page_addr = (offaddr/GD32F30X_PAGE_SIZE)*GD32F30X_PAGE_SIZE + GD32_FLASH_BASE;//ҳ�׵�ַ  0~127ҳ for GD32F303RCT6  

    return Page_addr;	
}



/*******************************************
* Function Name : FLASH_Write
* Description   : ��ָ����ַ��ʼд��ָ�����ȵ�����
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��10��15:07:02
* Notes         : GD32F4ֻ�ܰ�����������ȫƬ��������ע��д��������ʱ�Ƿ����������ݲ���
                  �������д���ַ��FLASHû�б����������ݽ�����д��
*******************************************/
void FLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint32_t NumToWrite)	
{ 
    fmc_state_enum status = FMC_READY;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;	

    if(WriteAddr<GD32_FLASH_BASE||WriteAddr%2)return;	//�Ƿ���ַ

    fmc_unlock();	//���� 
      
    addrx = WriteAddr;//д�����ʼ��ַ

    endaddr = WriteAddr + NumToWrite*2;	//д��Ľ�����ַ

    if(addrx<0X1FFF0000)			//�ж��Ƿ������洢����,�����ܲ�д
    {
      while(addrx<endaddr)		
      {   
        status = fmc_page_erase(FLASH_GetFlashPage(addrx));//��ҳ����
        if(status!=FMC_READY)break;	//��������
        addrx += 0x800;
      } 
    }
    if(status==FMC_READY)
    {
      while(WriteAddr<endaddr)//д����
      {
        if(fmc_halfword_program(WriteAddr,*pBuffer)!=FMC_READY)//д������,���֣�16λ��
        { 
          break;	//д���쳣
        }
        
        WriteAddr += 2;
        pBuffer++;
      } 
    }

    fmc_lock();//����
} 

/*******************************************
* Function Name : FLASH_Read
* Description   : ��ָ����ַ��ʼ����ָ�����ȵ�����
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��10��15:07:02
* Notes         : None
*******************************************/
void FLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint32_t NumToRead)   	
{
    uint32_t i;
    for(i=0;i<NumToRead;i++)
    {
      pBuffer[i] = FLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
      ReadAddr += 2;//ƫ��2���ֽ�.	
    }
}

/*******************************************
* Function Name : MC_Params_Load
* Description   : ���ڴ������ز���
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��13��11:07:04
* Notes         : None
*******************************************/

uint8_t MC_Params_Load(uint32_t addr)
{
    MC_Params_union  params_union;
  
    unsigned char *ptr = (unsigned char *)addr;
  
    memcpy(&params_union,ptr,sizeof(MC_Params_union));//��flash����
    memcpy(&MC_Params_temp,&params_union.MC_Params,sizeof(MC_Params_t));//����
  
    return 1;
}

/*******************************************
* Function Name : MC_Params_Save
* Description   : д����ز������ڴ�
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��13��11:07:04
* Notes         : None
*******************************************/

uint8_t MC_Params_Save(MC_Params_t *params)
{
    MC_Params_union  params_union;
  
    memset(&params_union,0,sizeof(MC_Params_union));//���
    memcpy(&params_union.MC_Params,params,sizeof(MC_Params_t));//����
  
    FLASH_Write(PARAMS_START_ADDR,(uint16_t *)&params_union,sizeof(MC_Params_union)); 
  
    return 1;
}

/*******************************************
* Function Name : *Get_Params
* Description   : ���ص�ַ
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��13��11:07:04
* Notes         : None
*******************************************/
MC_Params_t *Get_Params(void)
{
    return &MC_Params_temp;

}

/*******************************************
* Function Name : Real_time_get
* Description   : ʵʱʱ���ȡ������Ϊ����汾
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��16��15:15:51
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
    
    hour  = (_time[0] - '0') * 10 + (_time[1] - '0');//ʱ
    
    if(_time[2] == ':')
    {

      minute = (_time[3] - '0') * 10 + (_time[4] - '0');//��
    }
    
    for(i=0; i<12; i++)
    {
      if((Month_Table[i][0] == _date[0]) && (Month_Table[i][1] == _date[1]) && (Month_Table[i][2] == _date[2]))
      {
        mon = i+1;
      }      
    }
    if(_date[4]==' ')//��
    {
      day = _date[5]-'0';
    }
    else
    {
      day = 10*(_date[4]-'0')+_date[5]-'0';
    }

    year = 10*(_date[9]-'0')+_date[10]-'0';//�� 
    
    return (year*100000000 + mon*1000000 + day*10000 + hour * 100 + minute);//2021��9��16��15:16 --> 2109161516
}

/*******************************************
* Function Name : Message_ReadWrite
* Description   : ����д��Ϣ��flash
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��16��17:22:00
* Notes         : None
*******************************************/
uint32_t Message_ReadWrite(void)
{
    static uint8_t Message_status;
  
    MC_Params_Load(PARAMS_START_ADDR);//���ش洢�Ĳ���
  
    if(MC_Params_temp.MC_Code_Vision!=MC_Params_u.MC_Code_Vision)//�汾��洢�Ĳ�һ�´���汾�����˸��»�û�б���汾��
    {
      MC_Params_temp.MC_Code_Vision = MC_Params_u.MC_Code_Vision;
      MC_Params_Save(Get_Params());//����
      delay_1ms(1);
      
      Message_status = 1;
    }

    if(MC_Params_temp.MC_APP_Mark!=MC_Params_u.MC_APP_Mark)//APP mark
    {
      MC_Params_temp.MC_APP_Mark = MC_Params_u.MC_APP_Mark;
      MC_Params_Save(Get_Params());//����
      delay_1ms(1);
      
      Message_status = 2;
    }    

    if(MC_Params_temp.MC_EleAngle_Offset==(int16_t)0xFFFF)//û�е�Ƕ���ƫֵ
    {
      Message_status = 3;
    }

    
    return Message_status;
}

/*******************************************
* Function Name : MC_Load_CAN_ID
* Description   : ���ص��/������ID
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��11��15��17:59:06
* Notes         : None
*******************************************/
uint8_t MC_Load_CAN_ID(void)
{
  if(MC_Params_temp.MC_CAN_ID==0xff) MC_Params_temp.MC_CAN_ID = 1;
    
  return MC_Params_temp.MC_CAN_ID;

}


/*******************************************
* Function Name : MC_Save_CAN_ID
* Description   : �������õ�ID
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��11��15��17:59:01
* Notes         : None
*******************************************/
uint8_t MC_Save_CAN_ID(uint8_t id)
{
    static uint8_t can_state;
   
    if(MC_Params_temp.MC_CAN_ID!=id)//���õ�ID��洢�Ĳ�һ��
    {
      MC_Params_Load(PARAMS_START_ADDR);//���ش洢�Ĳ���   
      
      MC_Params_temp.MC_CAN_ID = id;
      MC_Params_Save(Get_Params());//���� 

      MC_Params_Load(PARAMS_START_ADDR);//�ٴζ�ȡ     
    }  
    
    if(MC_Load_CAN_ID()==id) can_state = 1;
    else can_state = 0;    

    return can_state;    
}

/*******************************************
* Function Name : MC_Load_Angle_offset
* Description   : ���ص�Ƕ���ƫ
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��11��15��17:59:01
* Notes         : None
*******************************************/
int16_t MC_Load_Angle_offset(void)
{
    return MC_Params_temp.MC_EleAngle_Offset;
}


/*******************************************
* Function Name : MC_Save_Angle_offset
* Description   : �����Ƕ���ƫ
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��11��15��17:59:01
* Notes         : None
*******************************************/
uint8_t MC_Save_Angle_offset(int16_t angle)
{
    static uint8_t offset_state;
  
    if(MC_Params_temp.MC_EleAngle_Offset!=angle)
    {
      MC_Params_Load(PARAMS_START_ADDR);//���ش洢�Ĳ���

      MC_Params_temp.MC_EleAngle_Offset = angle;

      MC_Params_Save(Get_Params());//���� 

      MC_Params_Load(PARAMS_START_ADDR);//���ش洢�Ĳ���    

    }

    
    if(MC_Params_temp.MC_EleAngle_Offset==angle) offset_state = 1;
    else offset_state = 0; 
    
    return  offset_state;   
}


/*******************************************
* Function Name : iap_Func
* Description   : ��ת��iap
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021��9��10��15:07:02
* Notes         : None
*******************************************/
void iap_Func(void)
{

    MC_Params_Load(PARAMS_START_ADDR);//���ش洢�Ĳ��� 

    MC_Params_temp.MC_APP_Mark = 0xFFFF;
    MC_Params_Save(Get_Params());//����
  
    NVIC_SystemReset();
}
