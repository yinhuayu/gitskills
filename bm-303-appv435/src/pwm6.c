#include "gd32f30x.h"
#include "foc_cfg.h"

typedef struct _pwm6_ctx
{
    uint32_t timer;
    rcu_periph_enum rcu;
    uint32_t portah;
    uint16_t pinah;
    uint32_t portbh;
    uint16_t pinbh;
    uint32_t portch;
    uint16_t pinch;

    uint32_t portal;
    uint16_t pinal;
    uint32_t portbl;
    uint16_t pinbl;
    uint32_t portcl;
    uint16_t pincl;
} pwm6_ctx_t;

const pwm6_ctx_t pwm6_cfg[] = {
    {
        .timer = TIMER0,
        .rcu = RCU_TIMER0,
        .portah = GPIOA,
        .pinah = 8,
        .portbh = GPIOA,
        .pinbh = 9,
        .portch = GPIOA,
        .pinch = 10,

        .portal = GPIOB,
        .pinal = 13,
        .portbl = GPIOB,
        .pinbl = 14,
        .portcl = GPIOB,
        .pincl = 15,
    },

    {
        .timer = TIMER7,
        .rcu = RCU_TIMER7,
        .portah = GPIOC,
        .pinah = 6,
        .portbh = GPIOC,
        .pinbh = 7,
        .portch = GPIOC,
        .pinch = 8,

        .portal = GPIOA,
        .pinal = 7,
        .portbl = GPIOB,
        .pinbl = 0,
        .portcl = GPIOB,
        .pincl = 1,
    },
};
 unsigned int aaaaa;
void tim_pwm6_init()
{
    /* -----------------------------------------------------------------------
    TIMER0 configuration:
    generate 3 complementary PWM signal.
    TIMER0CLK is fixed to systemcoreclock, the TIMER0 prescaler is equal to 119 
    so the TIMER0 counter clock used is 1MHz.
    insert a dead time equal to 200/systemcoreclock =1.67us 
    configure the break feature, active at low level, and using the automatic
    output enable feature.
    use the locking parameters level 0.
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    timer_break_parameter_struct timer_breakpara;

    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    // gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);
    // gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10);
    // gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    for (int i = 0; i < 2; i++)
    {
        gpio_init(pwm6_cfg[i].portah, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, 1 << pwm6_cfg[i].pinah);
        gpio_init(pwm6_cfg[i].portbh, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, 1 << pwm6_cfg[i].pinbh);
        gpio_init(pwm6_cfg[i].portch, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, 1 << pwm6_cfg[i].pinch);
        gpio_init(pwm6_cfg[i].portal, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, 1 << pwm6_cfg[i].pinal);
        gpio_init(pwm6_cfg[i].portbl, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, 1 << pwm6_cfg[i].pinbl);
        gpio_init(pwm6_cfg[i].portcl, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, 1 << pwm6_cfg[i].pincl);

        rcu_periph_clock_enable(pwm6_cfg[i].rcu);

        timer_deinit(pwm6_cfg[i].timer);
        /* initialize TIMER init parameter struct */
        // timer_struct_para_init(&timer_initpara);
        /* pwm6_cfg[i].timer configuration */
        timer_initpara.prescaler = 0;
        timer_initpara.alignedmode = TIMER_COUNTER_CENTER_BOTH;
        timer_initpara.counterdirection = TIMER_COUNTER_UP;
        timer_initpara.period = PWM_ARR - 1; //20khz
        timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
        timer_initpara.repetitioncounter = 0;
        timer_init(pwm6_cfg[i].timer, &timer_initpara);
        /* pwm6_cfg[i].timer  configuration */
        /* initialize TIMER channel output parameter struct */
        timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
        timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
        timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
        timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
        timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
        timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

        timer_channel_output_config(pwm6_cfg[i].timer, TIMER_CH_0, &timer_ocintpara);
        timer_channel_output_config(pwm6_cfg[i].timer, TIMER_CH_1, &timer_ocintpara);
        timer_channel_output_config(pwm6_cfg[i].timer, TIMER_CH_2, &timer_ocintpara);
        timer_channel_output_config(pwm6_cfg[i].timer, TIMER_CH_3, &timer_ocintpara); //to trigger adc.

        timer_channel_output_pulse_value_config(pwm6_cfg[i].timer, TIMER_CH_0, 0);
        timer_channel_output_mode_config(pwm6_cfg[i].timer, TIMER_CH_0, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(pwm6_cfg[i].timer, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);

        timer_channel_output_pulse_value_config(pwm6_cfg[i].timer, TIMER_CH_1, 0);
        timer_channel_output_mode_config(pwm6_cfg[i].timer, TIMER_CH_1, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(pwm6_cfg[i].timer, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);

        timer_channel_output_pulse_value_config(pwm6_cfg[i].timer, TIMER_CH_2, 0);
        timer_channel_output_mode_config(pwm6_cfg[i].timer, TIMER_CH_2, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(pwm6_cfg[i].timer, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);

        timer_channel_output_mode_config(pwm6_cfg[i].timer, TIMER_CH_3, TIMER_OC_MODE_PWM0);

        /* automatic output enable, break, dead time and lock configuration*/
        timer_breakpara.runoffstate = TIMER_ROS_STATE_ENABLE;
        timer_breakpara.ideloffstate = TIMER_IOS_STATE_ENABLE;
        timer_breakpara.deadtime = 50;
        timer_breakpara.breakpolarity = TIMER_BREAK_POLARITY_LOW;
        timer_breakpara.outputautostate = TIMER_OUTAUTO_DISABLE;
        timer_breakpara.protectmode = TIMER_CCHP_PROT_OFF;
        timer_breakpara.breakstate = TIMER_BREAK_DISABLE;

       

        aaaaa = TIMER_CCHP(pwm6_cfg[i].timer);
        timer_break_config(pwm6_cfg[i].timer, &timer_breakpara);
        
        aaaaa = TIMER_CCHP(pwm6_cfg[i].timer);
 
        /* pwm6_cfg[i].timer primary output function enable */
        timer_primary_output_config(pwm6_cfg[i].timer, ENABLE);

        timer_master_output_trigger_source_select(pwm6_cfg[i].timer, TIMER_TRI_OUT_SRC_UPDATE); //TRGO输出

        /* pwm6_cfg[i].timer channel control update interrupt enable */
        // timer_interrupt_enable(pwm6_cfg[i].timer, TIMER_INT_CMT);
        /* pwm6_cfg[i].timer break interrupt disable */
        // timer_interrupt_disable(pwm6_cfg[i].timer, TIMER_INT_BRK);

        /* pwm6_cfg[i].timer counter enable */
        timer_enable(pwm6_cfg[i].timer);
    }
    timer_repetition_value_config(TIMER7, 1);//使用重复计数将触发TRGO的频率调节到20k
    // timer_master_output_trigger_source_select(TIMER7, TIMER_TRI_OUT_SRC_UPDATE);
}

//用户手册p294  TIM7_TRGO连接到 TIMER1的ITI1(非互联型产品)
#define TIM_SLAVE TIMER1
void tim1_slave_init()
{
    // timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    // timer_break_parameter_struct timer_breakpara;
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER4);

    timer_deinit(TIM_SLAVE);
    /* TIMER1 configuration */
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_CENTER_BOTH;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 0xffff; //20khz
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIM_SLAVE, &timer_initpara);
    
    /* TIM_SLAVE primary output function enable */
    timer_slave_mode_select(TIM_SLAVE, TIMER_SLAVE_MODE_RESTART);
    timer_master_slave_mode_config(TIM_SLAVE, TIMER_MASTER_SLAVE_MODE_ENABLE);//使能tim1的从模式
    timer_input_trigger_source_select(TIM_SLAVE, TIMER_SMCFG_TRGSEL_ITI1);//选择ITI1 = TIM7_TRGO作为触发源

    // timer_master_output_trigger_source_select(TIM_SLAVE, TIMER_TRI_OUT_SRC_RESET); //TRGO输出
    /* TIM_SLAVE counter enable */
    // timer_enable(TIM_SLAVE);
}