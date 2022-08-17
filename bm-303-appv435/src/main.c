#include <stdint.h>
#include "gd32f30x.h"
#include "encoder.h"
#include "systick.h"
#include <stdio.h>
#include "pwm6.h"
#include "foc_cfg.h"
#include "fitness.h"
#include "soft_schd.h"
#include "bsp_adc.h"
#include "log.h"
#include "foc.h"
#include "pid.h"
#include "enc.h"
#include "UART.h"
#include "PROTECT.h"
#include "FMC.h"
#include "CAN.h"

#include "Hardware_Init.h"
#include "Protocol_FrameHandler.h"
#include "IAP.h"

uint32_t timer = 0;

uint8_t cmd_times = 0;

uint8_t is_first_time = 0;

extern fitness_t fit_M1_buf; //来自usart 的设置。特定时机再更新。
extern fitness_t fit_M2_buf; //来自usart 的设置。特定时机再更新。
extern fitness_t fit_M1;     //来自usart 的设置。特定时机再更新。
extern fitness_t fit_M2;     //来自usart 的设置。特定时机再更新。
/**
 * @note :以下三个参数需要按照实际填写，如版本号是1.2，则版本号为0x0102，
 *	@note :APP运行后，版本号以及型号会写入Flash数据区，下次OTA时的文件名必须符合
 *	@note :这三个参数
 */
/* 当前硬件版本号 */
#define HARDWARE_VERSION 0x0103 // 红色风扇接头的板子为1.3
/* 当前APP版本号 */
#define APP_VERSION 0x0403 // 4.x后为支持此0ta的版本4.2
/* 当前型号 */
#define DEVICE_MODEL "M1501J"

/* BL地址，跳转BL用，无需修改 */
#define BLOADER_ADDRESS 0x8000000
/* 当前APP的地址偏移，无需修改 */
#define ADDR_OFFSET_NOW (APP_A_BASE_ADDRESS - GD32_FLASH_BASE)

int16_t m0m1_ma[4];

uint16_t uid[6] = {0};
/* 当前硬件版本号 */
uint16_t HARDWARE_VERSION_t = HARDWARE_VERSION; // 红色风扇接头的板子为1.3
/* 当前APP版本号 */
uint16_t APP_VERSION_t = APP_VERSION; // 4.x后为支持此0ta的版本4.2
/**
 * @breif:APP初始化函数，进入APP第一时间调用
 * @note :包括中断向量表、Flash操作
 */
void app_init()
{
  /* APP Init */
  uint32_t _saving_temp[0x10];

  /* set the NVIC vector table base address to APP code area */
  nvic_vector_table_set(NVIC_VECTTAB_FLASH, ADDR_OFFSET_NOW);

  /* enable global interrupt, the same as __set_PRIMASK(0) */
  __enable_irq();

  /* APP Ready */
  if ((*(__IO uint16_t *)APP_READY) == 0xAA55)
    return;

  /* Saving Data */
  for (uint16_t index = 0; index < 0x10; index++)
  {
    _saving_temp[index] = (*(__IO uint32_t *)(FLASH_DATA_BASE_ADDRESS + index * 4));
  }

  /* Modify Data */
  uint8_t model[16] = DEVICE_MODEL;

  _saving_temp[4] = HARDWARE_VERSION;
  _saving_temp[6] = APP_VERSION;
  _saving_temp[7] = DEVICE_ID;
  _saving_temp[8] = *((uint32_t *)(model));
  _saving_temp[9] = *((uint32_t *)(model + 4));
  _saving_temp[10] = *((uint32_t *)(model + 8));
  _saving_temp[11] = *((uint32_t *)(model + 12));
  _saving_temp[12] = 0xAA55;

  /* Program Flash */
  if (FLASH_IF_Write_Word(FLASH_DATA_BASE_ADDRESS, _saving_temp, 0x10) == FLASHIF_OK)
  {
    is_first_time = 1;
  }
}

/*******************************************
* Function Name : FLASH_IF_Write_Word
* Description   : 按字从指定地址开始写入指定长度的数据
* Input         : None
* Output        : None
* Author        : Ming
* Date          : 2021年9月10日15:07:02
* Notes         : GD32F4只能按扇区擦除或全片擦除，需注意写入新数据时是否会把其他数据擦除
                  如果发现写入地址的FLASH没有被擦除，数据将不会写入
*******************************************/
FLASH_IF_Status_TypeDef FLASH_IF_Write_Word(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)
{
  fmc_state_enum status = FMC_READY;
  FLASH_IF_Status_TypeDef reval = FLASHIF_OK;
  uint32_t addrx = 0;
  uint32_t endaddr = 0;

  if (WriteAddr < GD32_FLASH_BASE || WriteAddr % 2)
    return FLASHIF_ADDR_ERROR; //非法地址

  fmc_unlock(); //解锁

  addrx = WriteAddr; //写入的起始地址

  endaddr = WriteAddr + NumToWrite * 4; //写入的结束地址

  if (addrx < 0X1FFF0000) //判断是否在主存储区内,否则不能擦写
  {
    while (addrx < endaddr)
    {
      if (*(__IO uint32_t *)(addrx) != 0xffffffff)
      {
        uint32_t page_addr = 0x800D800;     // FLASH_GetFlashPage(addrx);
        status = fmc_page_erase(page_addr); //掳麓页虏脕鲁媒
        if (status != FMC_READY)
          break; //路垄珊麓韼?
      }
      addrx += 4;
    }
  }
  if (status == FMC_READY)
  {
    while (WriteAddr < endaddr) //写数据
    {
      if (fmc_word_program(WriteAddr, *pBuffer) != FMC_READY) //写入数据,字（32位）
      {
        reval = FLASHIF_WRITING_ERROR;
        break; //写入异常
      }
      else
      {
        if (*(__IO uint32_t *)(WriteAddr) != *pBuffer)
        {
          reval = FLASHIF_WRITINGCTRL_ERROR;
          break; //读出数据不对
        }
      }

      WriteAddr += 4;
      pBuffer++;
    }
  }
  else
  {
    reval = FLASHIF_ERASE_ERROR;
  }

  fmc_lock(); //上锁
  return reval;
}

/**
 * @brief 返回进入BL成功消息
 * @param
 * @param
 * @param
 * @retval
 */
void CMD_Handler_BL_Report(void)
{
  /* send */
  static uint8_t bl_temp[] = {0x55, 0x64, 0xC8};
  Frame_Send(DEVICE_ID, 0xAA, bl_temp, 3);
}

/**
 * @brief  返回APP烧录成功消息
 * @param
 * @param
 * @param
 * @retval
 */
void CMD_Handler_APP_OK_Report(void)
{
  /* send */
  Frame_Send(DEVICE_ID, 'A', 0, 0);
}

void CMD_Handler_OTA_Requset(void)
{
  /* APP帧处理，此处简化了连续5帧进入BL的方法（只需要累计5帧），应按照连续5帧的要求修改（主要是串口那边的设置我不太清楚）	*/
  if (Frame_Handler(&UART1_Frame_Box) == FRAME_OK)
  {
    if (Protocol_Frame_Box.cmd == 0xAA && Protocol_Frame_Box.data[0] == 0x55 && Protocol_Frame_Box.data[1] == 0x64 && Protocol_Frame_Box.data[2] == 0xC8)
    {
      cmd_times++;
      /* 接收到5帧进入BL的命令 */
      if (cmd_times >= 5)
      {
        /* 此处应加入关闭电调相关的业务 */
        PreDriverM1_OFF();
        PreDriverM2_OFF();

        /* 返回进入BL成功的消息 */
        CMD_Handler_BL_Report();
        /* 等待消息发送完成 */
        delay_1ms(5);
        /* 关闭所有中断，进入BL，如有别的中断，请添加 */
        nvic_irq_disable(USART1_IRQn);

        nvic_irq_disable(ADC0_1_IRQn);

        can_interrupt_disable(CAN0, CAN_INT_RFNE0);
        iap_load_app(BLOADER_ADDRESS);
      }
    }

    /* Receive another frame */
    Protocol_Frame_Box.rx_flag = 0;
    Frame_ReceiveAnotherOne();
  }
}
enc_var_t ev_M1;
enc_var_t ev_M2;
void ADC0_1_IRQHandler()
{
  adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
  if (adc_interrupt_flag_get(ADC0, ADC_INT_EOIC))
  {
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    adc2_inject_calc_current_ma(m0m1_ma);
    foc_compute(&fv_M2, m0m1_ma[0], m0m1_ma[1]);
    // if (fv_M2.iq > 0)
    // {
    //   ev_M2.raw_offset = 1;
    // }
    // else
    // {
    //   ev_M2.raw_offset = -1;
    // }
  }
  if (adc_interrupt_flag_get(ADC1, ADC_INT_EOIC))
  {
    adc_interrupt_flag_clear(ADC1, ADC_INT_EOIC);
    adc_inject_calc_current_ma(m0m1_ma);
    foc_compute(&fv_M1, m0m1_ma[2], m0m1_ma[3]);
    // if (fv_M1.iq > 0)
    // {
    //   ev_M1.raw_offset = 1;
    // }
    // else
    // {
    //   ev_M1.raw_offset = -1;
    // }
  }
}

// 因为电机2的ib ic都没链接到adc2的通道 所以无法触发。
// void ADC2_IRQHandler()
// {
//     if (adc_interrupt_flag_get(ADC2, ADC_INT_EOIC)) {
//         adc_interrupt_flag_clear(ADC2, ADC_INT_EOIC);
//         adc2_inject_calc_current_ma(m0m1_ma);
//         foc_compute(&fv_M2, m0m1_ma[0], m0m1_ma[1]);
//     }
// }
// uint32_t fake_pwm_ccr[4];
// uint16_t rcnt;

uint16_t encoder_get_theta1() { return encoder_get_theta_electrical(&ev_M1); }
uint16_t encoder_get_theta2() { return encoder_get_theta_electrical(&ev_M2); }
uint16_t encoder_get_theta_mechanical1() { return encoder_get_theta_mechanical(&ev_M1); }
uint16_t encoder_get_theta_mechanical2() { return encoder_get_theta_mechanical(&ev_M2); }
uint16_t encoder_reset_zero1() { encoder_reset_raw_cnt(&ev_M1, 0); }
uint16_t encoder_reset_zero2() { encoder_reset_raw_cnt(&ev_M2, 0); }
int32_t encoder_get_odmetry_cnt1() { return encoder_get_odmetry_cnt(&ev_M1); }
int32_t encoder_get_odmetry_cnt2() { return encoder_get_odmetry_cnt(&ev_M2); }

const foc_func_set_t ffs1 = {
    // .tp_pin_set = tp_set,
    .get_sensor_theta_electrical = encoder_get_theta1,
    .get_sensor_theta_mechanical = encoder_get_theta_mechanical1,
    .get_sensor_odmetry = encoder_get_odmetry_cnt1,
    .reset_sensor_zero = encoder_reset_zero1,
};

const foc_func_set_t ffs2 = {
    //.tp_pin_set = tp_set,
    .get_sensor_theta_electrical = encoder_get_theta2,
    .get_sensor_theta_mechanical = encoder_get_theta_mechanical2,
    .get_sensor_odmetry = encoder_get_odmetry_cnt2,
    .reset_sensor_zero = encoder_reset_zero2,
};

int main(void)
{
  app_init();

  // nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x3000);

  LOGI("fuck f303");
  systick_config();

  NVIC_IRQ_Init();

  tim_encoder_init();
  tim_pwm6_init();
  tim1_slave_init();
  UART1_Init(115200);

  /* 更新完第一次进入APP，回复上位机 */
  if (is_first_time)
  {
    /* Report Enter APP OK */
    CMD_Handler_APP_OK_Report();
  }

  led_fan_init();
  fit_init();
  PreDriverM1M2_init();
  adc_regular_channels_config();
  adc_inject_channels_config();
  CAN_Config(CAN_BT_SJW_1TQ, CAN_BT_BS1_5TQ, CAN_BT_BS2_4TQ, 12); // 500K:CAN 波特率 = 60M/((CAN_BT_BS1_5TQ)+(CAN_BT_BS2_4TQ)+1)/(brp)
                                                                  //  delay_1ms(100);
  // task_func_register(led_blink, 100);
  task_func_register(task_debug_adc_channels, 1000);
  task_func_register(fitness_fsm_fv_M1, 10);
  task_func_register(fitness_fsm_fv_M2, 10);
  task_func_register(uart_send_state, 100);
  task_func_register(fan_clt, 10);
  task_func_register(Temp_protect, 100);
  task_func_register(U_I_protect, 10);
  task_func_register(led_blink, 1000);

  LOGI("running mode: foc");

  encoder_reg_cnt_register(&ev_M1, (uint32_t *)&TIMER_CNT(TIMER2)); // support 1 only.
  encoder_reg_cnt_register(&ev_M2, (uint32_t *)&TIMER_CNT(TIMER3));

  foc_register_func_set(&fv_M1, &ffs1);
  foc_register_func_set(&fv_M2, &ffs2);
// foc_register_tp_pin_func( tp_set );
#if 0
        uint32_t tim_ch[4] = {fake_pwm_ccr, fake_pwm_ccr+1, fake_pwm_ccr+2};
#else
  const uint32_t tim_ch[] = {
      &TIMER_CH0CV(TIMER0), // u
      &TIMER_CH1CV(TIMER0), // v
      &TIMER_CH2CV(TIMER0), // w
      &TIMER_CH3CV(TIMER0), // for trigger foc
  };

  const uint32_t tim_ch2[] = {
      &TIMER_CH0CV(TIMER7), // u
      &TIMER_CH1CV(TIMER7), // v
      &TIMER_CH2CV(TIMER7), // w
      &TIMER_CH3CV(TIMER7), // for trigger foc
  };
#endif
  foc_register_pwm_output(&fv_M1, tim_ch);
  foc_register_pwm_output(&fv_M2, tim_ch2);

#if 0
    *fv_M1.pwm[0] = PWM_ARR / 2 + 150;
    *fv_M1.pwm[1] = PWM_ARR / 2 - 150;
    *fv_M1.pwm[2] = PWM_ARR / 2 - 115;

    *fv_M2.pwm[0] = PWM_ARR / 2 + 100;
    *fv_M2.pwm[1] = PWM_ARR / 2 - 100;
    *fv_M2.pwm[2] = PWM_ARR / 2 - 77;
 
    *fv_M1.pwm[0] = PWM_ARR / 2 + 0;
    *fv_M1.pwm[1] = PWM_ARR / 2 - 0;
    *fv_M1.pwm[2] = PWM_ARR / 2 - 0;

    *fv_M2.pwm[0] = PWM_ARR / 2 + 0;
    *fv_M2.pwm[1] = PWM_ARR / 2 - 0;
    *fv_M2.pwm[2] = PWM_ARR / 2 - 0;
#else

  foc_algorithm_init2(&fv_M1);
  foc_algorithm_init2(&fv_M2);
#endif
  const int div = 1; // PWM_FREQ/10000;
  // current loop. max out vd, vq ~= pwm_full * 2
  pid_struct_init_q15(&fv_M1.pid_id, 2 * PWM_ARR, 2 * PWM_ARR, 333 / div, 64 / div, 0);
  pid_struct_init_q15(&fv_M1.pid_iq, 2 * PWM_ARR, 2 * PWM_ARR, 333 / div, 64 / div, 0);
  // iq_ref max ~= 5000mA
  pid_struct_init_q15(&fv_M1.pid_speed, 32767, 10000, 150960, 1000, 20000);
  fv_M1.pid_speed.deadband = 10;
  pid_struct_init_q15(&fv_M1.pid_pos, 32767, 2500, 50000, 0, 0);
  fv_M1.pid_pos.deadband = 100;

  pid_struct_init_q15(&fv_M2.pid_id, 2 * PWM_ARR, 2 * PWM_ARR, 333 / div, 64 / div, 0);
  pid_struct_init_q15(&fv_M2.pid_iq, 2 * PWM_ARR, 2 * PWM_ARR, 333 / div, 64 / div, 0);
  // iq_ref max ~= 5000mA
  pid_struct_init_q15(&fv_M2.pid_speed, 32767, 10000, 150960, 1000, 20000);
  fv_M2.pid_speed.deadband = 10;
  pid_struct_init_q15(&fv_M2.pid_pos, 32767, 2500, 50000, 0, 0);
  fv_M2.pid_pos.deadband = 100;

  // const int div = 1;//PWM_FREQ/10000;
  // //current loop. max out vd, vq ~= pwm_full * 2
  // pid_struct_init_q15(&fv_M1.pid_id,6*PWM_ARR, 0.2*PWM_ARR, 333/div, 64/div, 0);
  // pid_struct_init_q15(&fv_M1.pid_iq, 6*PWM_ARR, 0.2*PWM_ARR, 333/div, 64/div, 0);
  // //iq_ref max ~= 5000mA
  // pid_struct_init_q15(&fv_M1.pid_speed,  5000, 2500, 50960, 1000, 20000);
  // fv_M1.pid_speed.deadband = 100;
  // pid_struct_init_q15(&fv_M1.pid_pos, 5000, 2500, 10000, 0, 0);

  // pid_struct_init_q15(&fv_M2.pid_id, 6*PWM_ARR, 0.2*PWM_ARR, 333/div, 64/div, 0);
  // pid_struct_init_q15(&fv_M2.pid_iq, 6*PWM_ARR, 0.2*PWM_ARR, 333/div, 64/div, 0);
  // //iq_ref max ~= 5000mA
  // pid_struct_init_q15(&fv_M2.pid_speed,  5000, 2500, 50960, 1000, 20000);
  // fv_M2.pid_speed.deadband = 100;
  // pid_struct_init_q15(&fv_M2.pid_pos, 5000, 2500, 10000, 0, 0);

  LOGI("6pwm init. f_pwm & f_foc = %d Hz, ARR=%d", PWM_FREQ, PWM_ARR);

  // TIMER_CH0CV(TIMER0) = PWM_ARR/2;
  // TIMER_CH1CV(TIMER0) = PWM_ARR/2;
  // TIMER_CH2CV(TIMER0) = PWM_ARR/2;
  TIMER_CH3CV(TIMER0) = PWM_ARR - 10;
  TIMER_CH3CV(TIMER7) = PWM_ARR - 10;

  // TIMER_CH0CV(TIMER7) = PWM_ARR/2;
  // TIMER_CH1CV(TIMER7) = PWM_ARR/4;
  // TIMER_CH2CV(TIMER7) = PWM_ARR/8;

  /* 获取保存信息 */
  Message_ReadWrite();

  FLASH_Read(0x1FFFF7E8, uid, 6); //读取芯片UID

  while (1)
  {
    soft_schdule();

    CMD_Handler_OTA_Requset();
  }
}

// /* retarget the C library printf function to the USART */
// int fputc(int ch, FILE *f)
// {
//     usart_data_transmit(USART1, (uint8_t)ch);
//     while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
//     return ch;
// }
