#include "gd32f30x.h"
// #include "adc.h"
// #include "dma.h"
#include <stdint.h>
// #include "foc.h"
// #include "gpio.h"
#include "log.h"
#include "utils.h"
#include "bsp_adc.h"
// #include "drv_flash.h"
//c0 ~c5
//m1so1 ch10 c0
//m1so2 ch11 c1
//m1so3 ch12 c2
//m0so1 ch13 c3
//m0so2 ch14 c4 //没有adc2通道可用！！
//m0so3 ch15 c5

//ntc1  ch0  a0
//ntc2  ch1  a1
//ibus  ch4  a4
//temp  ch5  a5
//vbus  ch6  a6

const uint32_t regular_chs[] = {
    // ADC_CHANNEL_10,
    // ADC_CHANNEL_11,
    // ADC_CHANNEL_12,
    // ADC_CHANNEL_13,
    // ADC_CHANNEL_14,
    // ADC_CHANNEL_15,

    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5,
    ADC_CHANNEL_6,
};

const uint32_t inject_chs[] = {
    ADC_CHANNEL_10,
    ADC_CHANNEL_11,
    // ADC_CHANNEL_12,
    ADC_CHANNEL_13,
    ADC_CHANNEL_14,
    // ADC_CHANNEL_15,
};

static struct adc_pin
{
    uint32_t port;
    uint32_t pin;
} adc_pins[] = {
    {
        .port = GPIOC,
        .pin = GPIO_PIN_0,
    },
    {
        .port = GPIOC,
        .pin = GPIO_PIN_1,
    },
    {
        .port = GPIOC,
        .pin = GPIO_PIN_2,
    },
    {
        .port = GPIOC,
        .pin = GPIO_PIN_3,
    },
    {
        .port = GPIOC,
        .pin = GPIO_PIN_4,
    },
    {
        .port = GPIOC,
        .pin = GPIO_PIN_5,
    },

    {
        .port = GPIOA,
        .pin = GPIO_PIN_0,
    },
    {
        .port = GPIOA,
        .pin = GPIO_PIN_1,
    },
    {
        .port = GPIOA,
        .pin = GPIO_PIN_4,
    },
    {
        .port = GPIOA,
        .pin = GPIO_PIN_5,
    },
    {
        .port = GPIOA,
        .pin = GPIO_PIN_6,
    },
};

const char *ch_name[] = {
    // "CH_CUR_IA1",
    // "CH_CUR_IB1",
    // "CH_CUR_IC1",
    // "CH_CUR_IA0",
    // "CH_CUR_IB0",
    // "CH_CUR_IC0",
    "CH_NTC0",
    "CH_NTC1",
    "CH_IBUS",
    "CH_MOS_TEMP",
    "CH_VBUS",
};

// #define ADC_CONVERTED_DATA_BUFFER_SIZE 4
volatile int16_t shunt[3];
volatile uint16_t adc1_regular[CH_TOTAL_NUM];
volatile uint16_t adc1_inject[4];
typedef struct
{
    int16_t ia;
    int16_t ib;
    int16_t ic;
} cur_t;
// volatile cur_t cur;
#define R_SHUNT (1)   //mohm
#define AMP_GAIN (40) //gain = x40
const uint32_t MA2LSB = (R_SHUNT * AMP_GAIN * 4096 / 3300);
// #define ADC2MA      (1000.0f / MA2LSB)
const int ADC2MA = 1000.0f / MA2LSB;
uint8_t adc_cali_done;
int16_t ma[4];  //ma = raw * 16. 58*16~=1000mA
int16_t raw[4]; //1000mA = 10mV * gain 4.7 = 47mV ~= 58LSB
                //1000mA = 25mV * gain 20 = 500mV ~= 620L1000SB
                //=1.6 so *26/16
// int16_t raw2ma[];

//vbus 分压电阻49.9k+1k 比例k=1/51  vbus/vbus_adc = 51. vbus=51*vbus_adc*3300/4096
//电机NTC 是100k B=3950 下接10k
//MOSNTC  100k B=4419 下接3.3k

//res_vcc 是 上分压电阻， res_gnd是下分压电阻
#define RES_VCC 49900
#define RES_GND 1000
uint32_t vbus_calc(uint16_t vbus_adc, uint32_t res_vcc, uint32_t res_gnd)
{
    const float gain = (res_vcc + res_gnd) / res_gnd;
    uint32_t vbus_mv = 0;
    vbus_mv = gain * vbus_adc * 3300 / 4096;
    return vbus_mv;
}

//ibus采样使用CC6920-50A霍尔电流传感器 可以测+-50A对应
//计算公式VOUT = VCC / 2 + 0.040 × IP(A)   可以认为VCC/2是3.3/2=1.65 灵敏度40mV/A
int32_t ibus_calc_cc6920(uint16_t ibus_adc)
{
    uint32_t ibus_vout_mv = ibus_adc * 3300 / 4096;
    uint32_t ibus_vout_zero = 1650;

    int32_t ibus_delta_mv = ibus_vout_zero - ibus_vout_mv;
    return ibus_delta_mv * 1000 / 40; //1A each 40mv

    // int delta_adc = 2045 - ibus_adc;
    // //40mV = 40*4096/3300 = 50LSB = 1000mA
    // const float _1000ma2lsb = 40 * 4096 / 3300;
    // const float lsb2ma = 1000 / (40 * 4096 / 3300);    //1lsb = 20mA 50lsb = 1A
    // return delta_adc * lsb2ma;
}

//MOSNTC  100k B=4419 下接3.3k  //V1.2版本的硬件，改为相同的采样电路
int16_t get_temp_mos_ntc(uint16_t mos_ntc_adc)
{
    //extern const short ntc_lut_mos[];
    extern const short ntc_lut_motor[];
    //return ntc_lut_mos[mos_ntc_adc / 4];
	return ntc_lut_motor[mos_ntc_adc / 4];
}

int16_t get_temp_motor(uint16_t ntc_adc)
{
    extern const short ntc_lut_motor[];
    return ntc_lut_motor[ntc_adc / 4];
}

/*!
    \brief      DMA configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA0);

    /* initialize DMA channel0 */
    dma_deinit(DMA0, DMA_CH0);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)adc1_regular;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.number = CH_TOTAL_NUM;
    dma_init_struct.periph_addr = (uint32_t) & (ADC_RDATA(ADC0));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH0, dma_init_struct);

    /* configure DMA mode */
    dma_circulation_enable(DMA0, DMA_CH0);
    dma_memory_to_memory_disable(DMA0, DMA_CH0);

    /* enable DMA channel0 */
    dma_channel_enable(DMA0, DMA_CH0);
}

/*!
    \brief      ADC configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, CH_TOTAL_NUM);
    const uint32_t AD_SAMPLE_TIME = ADC_SAMPLETIME_13POINT5;
    /* ADC regular channel config */
    for (size_t i = 0; i < ARRAY_SZ(regular_chs); i++)
    {
        adc_regular_channel_config(ADC0, i, regular_chs[i], AD_SAMPLE_TIME);
    }

    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC external trigger source config */
    //adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_T0_CH0);
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); //software trigger
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    //adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    // ADC_CTL1 |= ADC_CTL1_CTN;    //enable continue conv
    // ADC_CTL0 |= ADC_CTL0_SM;     //enable scan mode
    adc_tempsensor_vrefint_enable();
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
    /* enable ADC interface */
    adc_enable(ADC0);
    /* ADC calibration and reset calibration */
    // adc_calibration_enable(ADC0);
    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);

    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

void adc_inject_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);

    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);

    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6); //120M/6= 20M;

    for (int i = 0; i < ARRAY_SZ(adc_pins); i++)
    {
        gpio_init(adc_pins[i].port, GPIO_MODE_AIN, GPIO_OSPEED_MAX, adc_pins[i].pin);
    }

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2);
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 2);
    const uint32_t AD_SAMPLE_TIME = ADC_SAMPLETIME_7POINT5;

    adc_inserted_channel_config(ADC0, 0, inject_chs[0], AD_SAMPLE_TIME);
    adc_inserted_channel_config(ADC0, 1, inject_chs[1], AD_SAMPLE_TIME);
    adc_inserted_channel_config(ADC1, 0, inject_chs[2], AD_SAMPLE_TIME);
    adc_inserted_channel_config(ADC1, 1, inject_chs[3], AD_SAMPLE_TIME);

    // adc_inserted_channel_offset_config(0, 2035);
    // adc_inserted_channel_offset_config(1, 2055);
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);
    /* ADC external trigger source config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T1_TRGO);
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3);
    //t0-adc0  t7-adc1
    //t7 - t1 - adc1
    //调试记录：根据F303用户手册p238 ADC1的注入触发信号 EXTI15/TIMER7_CH3是外部信号，实测无法触发，难道需要是外部输入？
    //准备使用T7为主 T1为slave 然后由T1触发ADC2的注入采样
    // adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);  //software trigger
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    // ADC_CTL1 |= ADC_CTL1_CTN;   //enable continue conv
    // ADC1_CTL0 |= ADC_CTL0_SM;
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    // adc_special_function_config(ADC_SCAN_MODE, ENABLE);
    adc_tempsensor_vrefint_enable();
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
    adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
    /* enable ADC interface */
    adc_enable(ADC0);
    adc_enable(ADC1);
    for (int time_x = 0; time_x < 6000; time_x++)
    {
    } //需最少2ms
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    adc_calibration_enable(ADC1);
    adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
    // adc_software_trigger_enable(ADC1, ADC_INSERTED_CHANNEL);

    // nvic_irq_enable(ADC_CMP_IRQn, 1U);
    // adc_interrupt_enable(ADC_INT_EOIC);
}

void adc_regular_channels_config()
{
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6); /* ADCCLK = PCLK2/6 */
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    dma_config();
    adc_config();
}

void adc_inject_channels_config()
{
    //120 / 6 = 20mhz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6); /* ADCCLK = PCLK2/6 */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);

    adc_inject_config();
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_EOIC);
    adc_interrupt_enable(ADC1, ADC_INT_EOIC);
    // adc_interrupt_flag_clear(ADC2, ADC_INT_EOIC);
    // adc_interrupt_enable(ADC2, ADC_INT_EOIC);
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
    // nvic_irq_enable(ADC2_IRQn, 0, 0);
}

void adc_sample_get_offset_hook(uint16_t cur[], uint16_t offset[])
{
#define CALI_OFFSET_SIZE (3)
    static uint16_t sample_cnt;
    static uint32_t sample_sum[CALI_OFFSET_SIZE];
    static uint8_t sample_done;
    const uint16_t sample_size = 1024;
    if (!sample_done)
    {
        for (int i = 0; i < CALI_OFFSET_SIZE; ++i)
        {
            sample_sum[i] += cur[i];
        }

        if (sample_cnt++ >= sample_size)
        {
            sample_done = 1;

            for (int i = 0; i < CALI_OFFSET_SIZE; ++i)
            {
                sample_sum[i] += cur[i];
                offset[i] = sample_sum[i] / sample_size;
                LOGI("sample offset calc %d", offset[i]);
                adc_cali_done = 1;
                //maybe we can write it to flash.
            }
        }
    }
}

typedef struct _linear_hall_cali
{
    /* data */
    uint16_t raw;
    uint16_t min;
    uint16_t max;
    uint16_t middle;
    int16_t value;
} linear_hall_cali_t;
linear_hall_cali_t hx = {.min = 1000, .max = 1000};
linear_hall_cali_t hy = {.min = 1000, .max = 1000};

static inline void hall_linear_sample_min_max(uint16_t raw, linear_hall_cali_t *ph)
{
    if (raw < ph->min)
    {
        ph->min = raw;
    }
    if (raw > ph->max)
    {
        ph->max = raw;
    }
    ph->middle = ph->min + (ph->max - ph->min) / 2;
    ph->value = (int16_t)raw - ph->middle;
}

int16_t hall_adc_offset = 450 + (2050 - 450) / 2;
int16_t hall_x;
int16_t hall_y;
int16_t hall_theta_tmp;
uint16_t hall_theta;
uint16_t hall_theta_offset = 44800;
uint16_t manual_save;
uint16_t manual_save_ma;
const uint16_t ma_ofs0[] = {2035, 2055};
const uint16_t ma_ofs1[] = {2030, 2030};
//amp gain = 20.
void adc_raw2ma_calc_regular()
{
#if 1
    // raw[0] = adc1_regular[CH_CUR_IA1];
    // raw[1] = adc1_regular[CH_CUR_IB1];
#else
    raw[0] = adc1_regular[CH_CUR_IA2];
    raw[1] = adc1_regular[CH_CUR_IB2];
#endif
    ma[0] = -(raw[0] - ma_ofs0[0]) * 26 / 16;
    ma[1] = -(raw[1] - ma_ofs0[1]) * 26 / 16;
#if 0
    raw[0] = adc1_regular[CH_CUR_IC];
    raw[1] = adc1_regular[CH_CUR_IB];
    // raw[2] = adc1_regular[CH_CUR_IC];

    // hall_linear_sample_min_max(adc1_regular[CH_HALLX], &hx);
    // hall_linear_sample_min_max(adc1_regular[CH_HALLY], &hy);
    // adc_sample_get_offset_hook(raw, raw_offset);
    if (manual_save) {
        manual_save = 0;
        store_param.hall_raw_offset[0] = hx.middle;
        store_param.hall_raw_offset[1] = hy.middle;
        store_param.param_status |= PARAM_STATUS_HALL_OFFSET;
        write_store_param(&store_param);
    }
    {
        ma[2] = -( raw[0] - store_param.current_raw_offset[2] ) * 4;
        ma[1] = -( raw[1] - store_param.current_raw_offset[1] ) * 4;
        ma[0] = -ma[1] - ma[2]; //ia
#if 0  
        hall_x = adc1_regular[CH_HALLX] - hall_adc_offset;
        hall_y = adc1_regular[CH_HALLY] - hall_adc_offset;
        hall_theta_tmp = -10435 * atan2f(hall_y, hall_x);
#else
            hall_theta_tmp = -10435 * atan2f(hy.value, hx.value);
#endif
        hall_theta = (uint16_t)hall_theta_tmp - hall_theta_offset;
    }
#endif
}
int32_t raw_sum23[2];          /////
uint32_t raw_offset_cnt23 = 0; /////
int32_t raw_offset23[2];       /////
void adc_inject_calc_current_ma(int16_t _ma[])
{
    raw[2] = ADC_IDATA0(ADC1);
    raw[3] = ADC_IDATA1(ADC1);

    if (raw_offset_cnt23 < 1)
    {
        raw_offset_cnt23++;
        raw_sum23[0] += raw[2];
        raw_sum23[1] += raw[3];
    }
    else if (raw_offset_cnt23 == 1)
    {
        raw_offset23[0] = raw_sum23[0] / 1;
        raw_offset23[1] = raw_sum23[1] / 1;
        raw_offset_cnt23++;
    }

    ma[2] = _ma[2] = (raw[2] - raw_offset23[0]) * ADC2MA; //2060
    ma[3] = _ma[3] = (raw[3] - raw_offset23[1]) * ADC2MA; //2060

    //    ma[2] = _ma[2] = ( raw[2] - 2055 ) * ADC2MA;
    //    ma[3] = _ma[3] = ( raw[3] - 2066 ) * ADC2MA;
}

int32_t raw_sum01[2];          /////
uint32_t raw_offset_cnt12 = 0; /////
int32_t raw_offset01[2];       /////
void adc2_inject_calc_current_ma(int16_t _ma[])
{
    raw[0] = ADC_IDATA0(ADC0);
    raw[1] = ADC_IDATA1(ADC0);

    if (raw_offset_cnt12 < 10)
    {
        raw_offset_cnt12++;
        raw_sum01[0] += raw[0];
        raw_sum01[1] += raw[1];
    }
    else if (raw_offset_cnt12 == 10)
    {
        raw_offset01[0] = raw_sum01[0] / 10;
        raw_offset01[1] = raw_sum01[1] / 10;
        raw_offset_cnt12++;
    }

    ma[0] = _ma[0] = (raw[0] - raw_offset01[0]) * ADC2MA; //2060
    ma[1] = _ma[1] = (raw[1] - raw_offset01[1]) * ADC2MA; //2060

    //    ma[0] = _ma[0] = ( raw[0] - 2060 ) * ADC2MA;//2060
    //    ma[1] = _ma[1] = ( raw[1] - 2060 ) * ADC2MA;//2060
}

// void dma_eos_adc_handler_ll(DMA_TypeDef* DMAx)
// {
//     if( LL_DMA_IsActiveFlag_TC1(DMAx) )
//     {
//         /* Clear flag DMA transfer complete */
//         LL_DMA_ClearFlag_TC1(DMAx);

//         /* Call interruption treatment function */
//         // AdcDmaTransferComplete_Callback();
//         // raw[0] = adc1_regular[CH_CUR_IA];
//         raw[1] = adc1_regular[CH_CUR_IB];
//         raw[2] = adc1_regular[CH_CUR_IC];
//         // raw[2] = adc1_regular[CH_CUR_IC];
//         adc_sample_get_offset_hook(raw, store_param.current_raw_offset);
//     }
// }

#include "log.h"

void task_debug_adc_channels(uint32_t arg)
{
    LOGI("\n========[ adc debug ]=========");
    for (int i = 0; i < CH_TOTAL_NUM; ++i)
    {
        LOGI("ch%d [%s\t] = \t%d", i, ch_name[i], adc1_regular[i]);
    }
    adc1_inject[0] = ADC_IDATA0(ADC0);
    adc1_inject[1] = ADC_IDATA1(ADC0);
    adc1_inject[2] = ADC_IDATA0(ADC1);
    adc1_inject[3] = ADC_IDATA1(ADC1);
    // for (int i=0; i<2; ++i)
    {
        LOGW("inject: %d\t %d\t %d\t %d\t ma:%d %d %d %d",
             adc1_inject[0],
             adc1_inject[1],
             adc1_inject[2],
             adc1_inject[3],
             ma[0], ma[1], ma[2], ma[3]);
    }
    LOGW("ibus: %d", ibus_calc_cc6920(adc1_regular[CH_IBUS]));
    LOGW("vbus: %d", vbus_calc(adc1_regular[CH_VBUS], RES_VCC, RES_GND));
    LOGW("mos_temp: %d", get_temp_mos_ntc(adc1_regular[CH_MOS_TEMP]));
    LOGW("motor_temp1: %d", get_temp_motor(adc1_regular[CH_NTC0]));
    LOGW("motor_temp2: %d", get_temp_motor(adc1_regular[CH_NTC1]));
    // float vbus1000 = adc1_regular[CH_VBUS] * 3300 / 4096 * 80 / 12;
    // int16_t wma = -ma[0] - ma[1];
    // LOGW("v bus = \t%d  mV", (int)(vbus1000));
    // LOGW("v bus = \t%d \t%d \t%d mA", ma[0], ma[1], wma);
}