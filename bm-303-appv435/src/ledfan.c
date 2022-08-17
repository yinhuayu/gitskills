#include "ledfan.h"
#include "bsp_adc.h"
#define MAX(a, b) ( (a) >= (b) ? (a) : (b) )




//extern volatile uint16_t adc1_regular[CH_TOTAL_NUM];

void led_fan_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    /* configure led&fan GPIO port */ 
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_11);
}






    // unsigned int PWM = 4;//  0<pwm<9
    // unsigned int fan_cnt;
    // unsigned int PWM1 ;
    // unsigned int PWM2 ;
    
    // short M1_Temp = 25;
    // short M2_Temp = 25;

/* 温度-风扇控制
 */
// void fan_clt() 
// {
//     fan_cnt++;

// /*  ibus_calc_cc6920(adc1_regular[CH_IBUS]);
//     vbus_calc(adc1_regular[CH_VBUS], RES_VCC, RES_GND);
//     get_temp_mos_ntc(adc1_regular[CH_MOS_TEMP]); */

//     M1_Temp = get_temp_motor(adc1_regular[CH_NTC0]);
//     M2_Temp = get_temp_motor(adc1_regular[CH_NTC1]);


//     if (M1_Temp<=30)
//     {
//         PWM1 = 0;
//     }
//     else if (M1_Temp>30 && M1_Temp<90)
//     {
//         PWM1 = M1_Temp/10;      
//     }
//     else if(M1_Temp>=90)
//     {
//         PWM1 = 9;  
//     }
    
//         if (M2_Temp<=30)
//     {
//         PWM2 = 0;
//     }
//     else if (M2_Temp>30 && M1_Temp<90)
//     {
//         PWM2 = M2_Temp/10;      
//     }
//     else if(M2_Temp>=90)
//     {
//         PWM2 = 9;  
//     }
   
//     PWM=MAX(PWM1,PWM2);

//     if ((fan_cnt % 10) <= PWM ) 
//         FAN_ON();
//     else
//         FAN_OFF();
// }