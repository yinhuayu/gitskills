#include "gd32f30x.h"
#include "foc_cfg.h"

#include <stdint.h>

#define ENC1_TIM TIMER2
#define ENC1_RCU RCU_TIMER2
#define ENC2_TIM TIMER3
#define ENC2_RCU RCU_TIMER3

typedef struct _enc_ctx
{
    uint32_t timer;
    rcu_periph_enum rcu;
    uint32_t porta;
    uint16_t pina;
    uint32_t portb;
    uint16_t pinb;
    uint16_t pulse_edge_cnt;
} enc_ctx_t;

const enc_ctx_t enc_cfg[] = {
    {
        .timer = TIMER2,
        .rcu = RCU_TIMER2,
        .porta = GPIOB,
        .pina = 1<<4,
        .portb = GPIOB,
        .pinb = 1<<5,
        .pulse_edge_cnt = ENC_MECH_1CIRCLE,
    },

    {
        .timer = TIMER3,
        .rcu = RCU_TIMER3,
        .porta = GPIOB,
        .pina = 1<<6,
        .portb = GPIOB,
        .pinb = 1<<7,
        .pulse_edge_cnt = ENC_MECH_1CIRCLE,
    },
};

void tim_encoder_init()
{
    timer_ic_parameter_struct timer_icinitpara;
    // timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    // timer_break_parameter_struct timer_breakpara;
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);
    
    //fuck dont init gpio!!
    for (int i = 0; i < 2; i++)
    {
        // gpio_init(enc_cfg[i].porta, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, enc_cfg[i].pina);
        // gpio_init(enc_cfg[i].portb, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, enc_cfg[i].pinb);
        
        rcu_periph_clock_enable(enc_cfg[i].rcu);

        timer_deinit(enc_cfg[i].timer);
        /* initialize TIMER init parameter struct */
        // timer_struct_para_init(&timer_initpara);
        /* enc_cfg[i].timer configuration */
        timer_initpara.prescaler = 0;
        timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
        timer_initpara.counterdirection = TIMER_COUNTER_UP;
        timer_initpara.period = enc_cfg[i].pulse_edge_cnt - 1; //20khz
        timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
        timer_initpara.repetitioncounter = 0;
        timer_init(enc_cfg[i].timer, &timer_initpara);
        /* enc_cfg[i].timer  configuration */
        /* initialize TIMER channel input parameter struct */
        // timer_channel_input_struct_para_init(&timer_icinitpara);
        /* enc_cfg[i].timer CH0 input capture configuration */
        timer_icinitpara.icpolarity = TIMER_IC_POLARITY_RISING;
        timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
        timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
        timer_icinitpara.icfilter = 0x0;
        timer_input_capture_config(enc_cfg[i].timer, TIMER_CH_0, &timer_icinitpara);
        timer_input_capture_config(enc_cfg[i].timer, TIMER_CH_1, &timer_icinitpara);

        timer_quadrature_decoder_mode_config(enc_cfg[i].timer,
                                             TIMER_ENCODER_MODE2,
                                             TIMER_IC_POLARITY_RISING,
                                             TIMER_IC_POLARITY_RISING);
        timer_auto_reload_shadow_enable(enc_cfg[i].timer);
        // timer_enable(enc_cfg[i].timer);
    }
    timer_enable(enc_cfg[0].timer);
    timer_enable(enc_cfg[1].timer);

    // timer_quadrature_decoder_mode_config(ENC1_TIM,
    //     TIMER_ENCODER_MODE2,
    //     TIMER_IC_POLARITY_RISING,
    //     TIMER_IC_POLARITY_RISING);

    // rcu_periph_clock_enable(ENC2_RCU);
    // timer_quadrature_decoder_mode_config(ENC2_TIM,
    //     TIMER_ENCODER_MODE2,
    //     TIMER_IC_POLARITY_RISING,
    //     TIMER_IC_POLARITY_RISING);
}