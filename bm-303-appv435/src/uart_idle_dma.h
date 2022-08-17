/******************************************************************************
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include <stdint.h>
#include "usart.h"

#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

#define RX_BUFF_SIZE (32)
//#define BT_HUART huart2
//#define MIDI_HUART huart2



typedef enum{
    HUART_ESP = 0,
    // HUART_DBUS,
//    HUART_BT = 3,
    HUART_NUM,
}hw_huart_e;

#pragma pack()
typedef struct
{
    int16_t ch1; //each ch value from -364 -- +364
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    uint8_t sw1; //3 value
    uint8_t sw2;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z; //???不明觉厉

        uint8_t l; //左键 按下是1
        uint8_t r; //右键按下1 放开0
    } mouse;

    union {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
        /**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
    } kb;
    //uint16_t kb;

} rc_info_t;

typedef union {

#pragma pack(1)
    struct
    {
        signed long long ch1 : 11;
        signed long long ch2 : 11;
        signed long long ch3 : 11;
        signed long long ch4 : 11;
        signed long long sl : 2;
        signed long long sr : 2;
        signed long long mx : 16;
        //8 bytes

        int          my : 16; //2b
        int          mz : 16; //2b
        unsigned int mpl : 8; //1
        unsigned int mpr : 8; //1
        //4byte

        unsigned int W : 1;
        unsigned     S : 1;
        unsigned     A : 1;
        unsigned     D : 1;
        unsigned     SHIFT : 1;
        unsigned     CTRL : 1;
        unsigned     Q : 1;
        unsigned     E : 1;
        unsigned     R : 1;
        unsigned     F : 1;
        unsigned     G : 1;
        unsigned     Z : 1;
        unsigned     X : 1;
        unsigned     C : 1;
        unsigned     V : 1;

        unsigned : 1; //reserved
        unsigned : 16; //reserved
    } bits;
#pragma pack()

    uint8_t buff[18];

} RC_UnionDef;
//这样写也可以

enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};

typedef struct __RC
{

    int16_t expYaw; //yaw
    int16_t throttle; //throttle
    int16_t expRoll; //roll
    int16_t expPitch; //pit
    uint8_t wdogAlive;

    uint8_t switch1;
    uint8_t switch2;

} LANGO_RC_t;



extern rc_info_t      rc;
// extern CV_BigBuff_t cv_big_buff;
//extern uint8_t bt_rxbuff[];
// extern FloatConvertType rcDataTable[RcTableLength];
// void uart_send_frame(UART_HandleTypeDef* huart, RcTableType id);
// void bt_printf(const char* fmt, ...);
// void uart_send2pc(UART_HandleTypeDef* huart, RcTableType id, float value);
// void uart_send4byte(UART_HandleTypeDef* huart, RcTableType id, void* buff);
void bt_init(void);
// void dbus_init(void);
void dbus_init(rc_info_t* prc);
void manifold_uart_init(void);
void judgement_init(void);
void judge_sys_init(void);
void midi_uart_init(void);
//void esp_uart_init(void);

typedef void (*uart_rx_idle_handle_func)(uint8_t* buf, uint16_t len);
void uart_idle_handle_register(UART_HandleTypeDef* huart, uart_rx_idle_handle_func f);
void uart_idle_dma_rx_init(UART_HandleTypeDef* huart, uint8_t* rx_buff, uint16_t buff_len);
//called in uart irq(it.c)
void langgo_uart_dma_idle_it_callback(UART_HandleTypeDef* huart);

#endif
