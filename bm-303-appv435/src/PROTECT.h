#pragma once
#include "gd32f30x.h"
#include <stdint.h>
#include <stdio.h>
#include "foc_cfg.h"
#include "fitness.h"
#include "soft_schd.h"
#include "bsp_adc.h"
#include "log.h"
#include "foc.h"
#include "pid.h"

#define LED_ON() GPIO_BOP(GPIOB) = GPIO_PIN_2
#define LED_OFF() GPIO_BC(GPIOB) = GPIO_PIN_2
#define FAN_ON() GPIO_BC(GPIOB) = GPIO_PIN_11
#define FAN_OFF() GPIO_BOP(GPIOB) = GPIO_PIN_11

#define PreDriverM1_ON()    GPIO_BOP(GPIOA) = GPIO_PIN_11
#define PreDriverM2_ON()    GPIO_BOP(GPIOA) = GPIO_PIN_12
#define PreDriverM1_OFF()    GPIO_BC(GPIOA) = GPIO_PIN_11
#define PreDriverM2_OFF()    GPIO_BC(GPIOA) = GPIO_PIN_12

void TEMP_FDBK_GET(void);
void BUS_VOLT_FDBK_GET(void);
void Motor_Sound(float Freq);
void Beep_Sound(float Freq,uint8_t State);
void DRIVER_FAULT_Get(void);
void Curr_Bus_Get(void);
void ERR_Alarm(void);
void BK_SET(uint8_t Mode,uint16_t Vbus,uint8_t duty);
void Curr_Bus_Protect(uint16_t curr);
void MC_LOCK_Protect(uint16_t curr,int16_t Speed);

void U_I_FDBK_GET();
void U_I_protect();
void led_fan_init();
void led_blink();
void fan_clt();
void PreDriverM1M2_init();
void Temp_protect();



