#pragma once

#include "gd32f30x.h"
#include "mc_math.h"
#include "bsp_adc.h"

#define LED_ON()    GPIO_BOP(GPIOB) = GPIO_PIN_2
#define LED_OFF()    GPIO_BC(GPIOB) = GPIO_PIN_2
#define FAN_ON()     GPIO_BC(GPIOB) = GPIO_PIN_11
#define FAN_OFF()    GPIO_BOP(GPIOB) = GPIO_PIN_11

void led_fan_init(void);
void tp_set(int x);
void led_blink();
void fan_clt();

