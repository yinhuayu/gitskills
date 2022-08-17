#include "CAN.h"
#define CAN_RECEIVE

/*******************************************
* Function Name : CAN_Config
* Description   : CAN初始化
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月7日15:49:53
* Notes         : None
*******************************************/
void CAN_Config(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp)
{
    can_parameter_struct          can_parameter;
	  can_filter_parameter_struct   can_filter_parameter;
  
  /* enable CAN0 clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF); 
  
    /* configure CAN0 GPIO */
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);//PB8-->CAN RX

    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);//PB9-->CAN TX

    gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);//开启完全重映射
  
    gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);//PB10-->CAN_120Ω_SW  
    gpio_bit_set(GPIOB, GPIO_PIN_10);	
  
//    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
//	  can_struct_para_init(CAN_FILTER_STRUCT, &can_filter_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);

    /* initialize CAN parameters */
    can_parameter.time_triggered         = DISABLE;
    can_parameter.auto_bus_off_recovery  = ENABLE;
    can_parameter.auto_wake_up           = ENABLE;
//    can_parameter.no_auto_retrans        = DISABLE;
    can_parameter.rec_fifo_overwrite     = DISABLE;
    can_parameter.trans_fifo_order       = DISABLE;
    can_parameter.working_mode           = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width      = tsjw;
    can_parameter.time_segment_1         = tbs1;
    can_parameter.time_segment_2         = tbs2;
    can_parameter.prescaler              = brp;
 
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    
    /* initialize filter */ 
    can_filter_parameter.filter_number        = 0;
    can_filter_parameter.filter_mode          = CAN_FILTERMODE_MASK;
    can_filter_parameter.filter_bits          = CAN_FILTERBITS_32BIT;
    can_filter_parameter.filter_list_high     = 0x0000;
    can_filter_parameter.filter_list_low      = 0x0000;
    can_filter_parameter.filter_mask_high     = 0x0000;
    can_filter_parameter.filter_mask_low      = 0x0000;
    can_filter_parameter.filter_fifo_number   = CAN_FIFO0;
    can_filter_parameter.filter_enable        = ENABLE;
    
    can_filter_init(&can_filter_parameter);  

    #if defined (CAN_RECEIVE)  
       can_interrupt_enable(CAN0, CAN_INT_RFNE0);
    #endif
}

/*******************************************
* Function Name : Can_Send_Msg
* Description   : CAN发送
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月7日17:04:10
* Notes         : None
*******************************************/
uint8_t Can_Send_Msg(uint32_t can_periph , uint32_t StdId, char* msg, uint8_t len)
{
    uint8_t mbox;
    uint16_t i=0; 
  
    can_trasnmit_message_struct   g_transmit_message;  
    g_transmit_message.tx_sfid    = StdId;
    g_transmit_message.tx_efid    = 0;
    g_transmit_message.tx_ft      = CAN_FT_DATA;
    g_transmit_message.tx_ff      = CAN_FF_STANDARD;
    g_transmit_message.tx_dlen    = len;
  
    for(i=0;i<len;i++)
    g_transmit_message.tx_data[i] = msg[i];			          
    mbox = can_message_transmit(can_periph, &g_transmit_message);   
    i = 0; 
    while((can_transmit_states(can_periph, mbox)==CAN_TRANSMIT_FAILED)&&(i<0XFFF))i++;	//等待发送结束
    if(i>=0XFFF)return 1;
    return 0;

}

/*
(驱动指令及反馈使用范围0x32~0XFF,设置指令及反馈使用范围0x100~0x500) 
驱动指令段：    0x32~0x96   100个标识符
驱动指令反馈段：0x97~0xFF   104个标识符
设置指令段：    0x100~0x200 256个标识符
设置指令反馈段：0x201~0x400 511个标识符
设置指令反馈段：0x401~0x500 255个其他标识符
*/
/*******************************************
* Function Name : CAN0_RX0_IRQHandler
* Description   : CAN0接收中断
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月7日17:10:28
* Notes         : None
*******************************************/ 
can_receive_message_struct    g_receive_message; 
char can_Rbuf01[8] = {0};
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    static uint8_t i;
    static uint8_t can_id; 
  
    /* check the receive message */
    can_message_receive(CAN0, CAN_FIFO0, &g_receive_message);

    switch(g_receive_message.rx_sfid)//最大7FF
    {
        case CAN_ID_UpdateAPP://更新请求
          for(i=0;i<8;i++)
          {
            can_Rbuf01[i] = g_receive_message.rx_data[i];        		 
          }

          if(can_Rbuf01[0]==0x75 && can_Rbuf01[1]==0x70 && can_Rbuf01[2]==0x64)//0x75 0x70 0x64 0x61 0x74 0x65 0xFF
          {			
            iap_Func();	  
          }  
        break;  
          
      default:
        break;    
    }      
 
  
}
