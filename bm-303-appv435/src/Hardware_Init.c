#include "Hardware_Init.h"


/*******************************************
* Function Name : Hardware_Init
* Description   : 硬件初始化
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年7月28日11:23:49
* Notes         : None
*******************************************/
void Hardware_Init(void)
{

	  /* configure systick */
    systick_config();

	  NVIC_IRQ_Init();
  
    //CAN_Config(CAN_BT_SJW_1TQ,CAN_BT_BS1_5TQ,CAN_BT_BS2_4TQ,6);// CAN 波特率 = 60M/((CAN_BT_BS1_5TQ)+(CAN_BT_BS2_4TQ)+1)/(brp)
		UART1_Init(115200);
	
	  rcu_periph_clock_enable(RCU_GPIOB);
		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);//LED
		gpio_bit_set(GPIOB, GPIO_PIN_2);	
}


