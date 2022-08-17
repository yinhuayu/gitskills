#include "Interrupt.h"
//#include "MC_CAN_TX.h"


/*******************************************
* Function Name : NVIC_IRQ_Init
* Description   : 中断优先级设置
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年7月30日10:57:33
* Notes         : None
*******************************************/
void NVIC_IRQ_Init(void)
{  
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1, 1);
}
		
		
		
		
/*******************************************
* Function Name : TIMER0_UP_TIMER9_IRQHandler
* Description   : TIME0更新中断
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年7月30日10:57:33
* Notes         : None
*******************************************/
void TIMER0_UP_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))
		{
		  timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
		
		
		}

}

