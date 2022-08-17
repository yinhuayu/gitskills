#pragma once
//#ifndef "_bsp_adc_h"
//#def "_bsp_adc_h"
#include <stdint.h>
#define RES_VCC 49900
#define RES_GND 1000
enum adc_ch_e {
    // CH_CUR_IA1 = 0,
    // CH_CUR_IB1,
    // CH_CUR_IC1,
    // CH_CUR_IA0,
    // CH_CUR_IB0,
    // CH_CUR_IC0,
    CH_NTC0,
    CH_NTC1,
    CH_IBUS,
    CH_MOS_TEMP,
    CH_VBUS,

    CH_TOTAL_NUM,
};

//volatile uint16_t adc1_regular[CH_TOTAL_NUM];



void adc_regular_channels_config();
void adc_curr_sample_cfg_after_low_lv(void);
void adc_irq_handler_ll(void);
void adc_reg_dma_sample_cfg(void);
// void dma_channel1_irq_handler_ll(DMA_TypeDef* DMAx);
// void dma_eos_adc_handler_ll(DMA_TypeDef* DMAx);
// void adc_reg_dma_conv_process(void);
void adc_current_sample_prepare_foc(int16_t currents[]);
void task_debug_adc_channels(uint32_t arg);

void adc_sample_get_offset_hook(uint16_t cur[], uint16_t offset[]);
void inline adc_raw2ma_calc_regular(void);
void adc_inject_calc_current_ma(int16_t ma[]);
void adc2_inject_calc_current_ma(int16_t _ma[]);

int16_t wheel2speed_ref();

extern int16_t ma[4];
extern uint16_t hall_theta;

void adc_inject_config(void);
void adc_inject_channels_config();
void adc_raw2ma_calc_inject();

//#endif