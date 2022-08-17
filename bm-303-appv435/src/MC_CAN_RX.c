#include "MC_CAN_RX.h"

CAN_RX_Date_t  CAN_RX_Date;
can_receive_message_struct    g_receive_message; 

unsigned char can_Rbuf01[8] = {0};


/*******************************************
* Function Name : CAN_RX_Deal
* Description   : CAN总线数据接收处理
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月27日19:27:50
* Notes         : None
*******************************************/
/*
(驱动指令及反馈使用范围0x32~0XFF,设置指令及反馈使用范围0x100~0x500) 
驱动指令段：    0x32~0x96   100个标识符
驱动指令反馈段：0x97~0xFF   104个标识符
设置指令段：    0x100~0x200 256个标识符
设置指令反馈段：0x201~0x400 511个标识符
设置指令反馈段：0x401~0x500 255个其他标识符
*/

uint8_t CAN_RX_BUF[CAN_REC_LEN] __attribute__ ((at(0X20001000)));//接收缓冲,最大USART_REC_LEN个字节,起始地址为0X20001000.
uint16_t CAN_RX_CNT = 0;
uint8_t Onebuff_flat = 0;
void CAN_RX_Deal(void)
{
    static uint8_t i;
	  static uint16_t CNT;//帧计数
    
    /* check the receive message */
    can_message_receive(CAN0, CAN_FIFO0, &g_receive_message);

    	
    switch(g_receive_message.rx_sfid)//最大7FF
    {
      
      case CAN_ID_UpdateAPP://更新请求指令
        
        for(i=0;i<8;i++)
        {
          can_Rbuf01[i] = g_receive_message.rx_data[i];        		 
        }
        
        CAN_RX_Date.can_receive_flag = SET;
        
        if(can_Rbuf01[0]==0x75 && can_Rbuf01[1]==0x70 && can_Rbuf01[2]==0x64)//0x75 0x70 0x64 0x61 0x74 0x65 0xFF 握手
        {			
          Onebuff_flat = 0x55; 
        }	
        
      break;
        
      case CAN_ID_Updating://更新中
        
        for(i=0;i<8;i++)//一帧8个字节
        {
          
          can_Rbuf01[i] = g_receive_message.rx_data[i]; //单帧缓存
          
          if(i<4)
          {
            CAN_RX_CNT = i+(CNT)*4;//前4个为程序数据
            CAN_RX_BUF[CAN_RX_CNT] = g_receive_message.rx_data[i];//多帧缓存				
          }

          if(i==7)
          {
            CAN_RX_Date.can_receive_flag = SET; 
            CNT++;//帧计数
          }		
         
        }
        if(CNT==512)
        {			
          CNT = 0; 
          Onebuff_flat = 1;  //2048个字节 256帧 1帧8个字节
        }
        
      break;
        

      case CAN_ID_Updatefinish://数据包已经发送完毕
        
        for(i=0;i<8;i++)
        {
          can_Rbuf01[i] = g_receive_message.rx_data[i];//单帧缓存          
          if(i==7)
          {
            CAN_RX_Date.can_receive_flag = SET; 
            CNT++;//帧计数
          }						
        }
        if(can_Rbuf01[0]==0xAA && can_Rbuf01[1]==0x55 && can_Rbuf01[2]==0xAA && CAN_RX_Date.can_receive_flag == SET)
        {			
          Onebuff_flat = 0xFF;  
        }
        
      break;

      case CAN_ID_Jumpapp://跳转至APP指令

        for(i=0;i<8;i++)
        {
          can_Rbuf01[i] = g_receive_message.rx_data[i];//单帧缓存
          if(i==7)
          {
            CAN_RX_Date.can_receive_flag = SET; 
          }								
        }
        if(can_Rbuf01[0]==0xC8 && can_Rbuf01[1]==0x64 && can_Rbuf01[2]==0xAA && CAN_RX_Date.can_receive_flag == SET)
        {			
          Onebuff_flat = 0xAA;  
        }

      break;
        
      default:
        break;
    }
        

}





/*******************************************
* Function Name : *Get_CAN_RX_Date_Addr
* Description   : 返回地址
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月29日10:15:20
* Notes         : None
*******************************************/
CAN_RX_Date_t *Get_CAN_RX_Date_Addr(void)
{
    return &CAN_RX_Date;
}

/*******************************************
* Function Name : StdId_cb
* Description   : 返回地址
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月29日10:15:20
* Notes         : None
*******************************************/
uint32_t StdId_cb(void)
{
    return g_receive_message.rx_sfid;
}



const char CRC8Table[]={  
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,  
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,  
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,  
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,  
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,  
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,  
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,  
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,  
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,  
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,  
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,  
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,  
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,  
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,  
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,  
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53  
};  

unsigned char CRC8_Table(unsigned char *p, int counter)  
{  
    unsigned char crc8 = 0;  
  
    for( ; counter > 0; counter--){  
        crc8 = CRC8Table[crc8^*p];  
        p++;  
    }  
    return(crc8);  
} 

unsigned char crc;
uint8_t crc_check(uint16_t size)
{
  crc = 0;
	crc = CRC8_Table(can_Rbuf01,size);
	if(crc!=can_Rbuf01[size]) return 0;
	
	return 1;
}
