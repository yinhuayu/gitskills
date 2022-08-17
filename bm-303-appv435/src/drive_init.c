// #include "stm32f0xx.h"
#include "tim_port.h"
#include "main.h"
#include "log.h"
// #include "6step.h"
#include "bsp_adc.h"
#include "foc.h"
// #include "hall.h"
// #include "uart_idle_dma.h"
// #include "../dm_protocol/dm_protocol.h"
#include "soft_schd.h"
#include "hw_common.h"
// #include "spi.h"
// #include "electrical_angle.h"

#include "pid.h"
#include "uart_idle_dma.h"
#include "dac.h"


#include "mycan.h"

void drive_init()
{

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    // extern void cpuz_info();
    // cpuz_info();
    __HAL_DBGMCU_FREEZE_TIM1(); //so we can set bkpt when motor running.
    __HAL_DBGMCU_FREEZE_TIM8(); //so we can set bkpt when motor running.
    // LOGW("%08x", DBGMCU->APB2FZ);
    // LOGW("%08x", DBGMCU->CR);
    LOGW("ironmz technology corp. ltd.");
    HAL_Delay(1000);
    // adc_reg_dma_sample_cfg();

    HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
    // HAL_Delay(777);
    // HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    // LOGW("relay1 relay2 set to 1");

#if FOC_HALL
    #include "hall.h"
    hall_hw_init();
    hall_init_observer(0);
    LOGI("hall init state: ");
#endif

#if FOC_SENSOR_TYPE == SENSOR_HALLXY
    //nothing.
#endif

#if SENSOR_MAG
    MX_SPI1_Init();
    LOGI("magnetic angle sensor init. ");
#endif

#if FOC_ENC
    #include "enc.h"
    encoder_hw_init();
    task_func_register(task_update_enc, 1);
    LOGI("encoder sensor init. ");
    // return ;
    // return get_electical_angle_sto();//sensorless
#endif

#if 1 //(USE_FOC)
    LOGI("running mode: foc");
    extern foc_var_int_t fv;
    foc_algorithm_init();
    pwm_init_foc();
    const int div = PWM_FREQ/10000;
    //current loop. max out vd, vq ~= pwm_full * 2
    pid_struct_init_q15(&pid_id, 0.8*PWM_ARR, 0.4*PWM_ARR, 333/div, 64/div, 0);
    pid_struct_init_q15(&pid_iq, 0.8*PWM_ARR, 0.4*PWM_ARR, 333/div, 64/div, 0);
    //iq_ref max ~= 5000mA
    pid_struct_init_q15(&pid_speed, 5000, 2500, 40960, 200, 20000);
    pid_struct_init_q15(&pid_pos, 5000, 2500, 15000, 0, 0);
    LOGI("6pwm init. f_pwm & f_foc = %d Hz, ARR=%d", PWM_FREQ, PWM_ARR);
    
#else
    pwm_init_6step();
    bldc_ctrl_init();
    HAL_TIM_GenerateEvent(&htim_pwm, TIM_EVENTSOURCE_COM);
    LOGI("6pwm init. f_pwm & f_foc = %d Hz, arr=%d", PWM_FREQ, PWM_ARR);
    LOGI("running mode: 6step");
#endif

    
    

    static uint8_t u6_buffer[64];
    // uart_idle_handle_register(&huart6, foc_uart_ctrl);
    // uart_idle_dma_rx_init(&huart6, u6_buffer, sizeof(u6_buffer));
    // langgo_fdcan_init(&hfdcan1, 1000000);
    // task_func_register(task_can_send, 100);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}