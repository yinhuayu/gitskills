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
/**
  * @update 2016年5月24日22:50:28
  * @note  更新了串口接收的方式。
        *				//使用DMA来接收，每当空闲中断的时候，也就是不收数据了的时候处理接收到的数据。
        *				//DMA中断正常情况下是不触发的，
  *要保证MAX-RX-SIZE大于每次的一帧数据长度
        *				//所以DMA的作用就是加快收数据的速率，然后其他没了
        *				//################### 状态机：
  *########################
        *				进入空闲中断--处理收到的数据--清除中断标记
        *				//--重新初始化DMA--等待接收数据直到空闲--进入空闲中断
  *       //################### 状态机： ########################
  * @after	测试过一次，成功！ 后面可能还要测一下
  */

#include "uart_idle_dma.h"
// #include "usart.h"

// NOT USE
#if defined STM32F4
uint32_t         JumpIapFlag __attribute__((at(0x40024000)));
#endif

#define MAX_DMA_COUNT       1024
#define DBUS_RX_MAX_BUFLEN  30
#define BT_RX_MAX_BUFLEN    100 //这个只要比一帧大就行拉
#define MIDI_RX_BUFF_LEN    10

#define ARRAY_SIZE(x)   ((sizeof(x) / sizeof((x)[0])))

rc_info_t           rc;
uint8_t          bt_rx_buff[BT_RX_MAX_BUFLEN];
uint8_t          dbus_buff[DBUS_RX_MAX_BUFLEN];
// uint8_t          transmit_frame[FrameLength];
// uint8_t          midi_buff[MIDI_RX_BUFF_LEN];

typedef struct{
    UART_HandleTypeDef* huart;
    uart_rx_idle_handle_func callback;
}uart_rx_callback_t;
static uart_rx_callback_t uart_rx_idle_callback[5];

void uart_idle_handle_register(UART_HandleTypeDef* huart, uart_rx_idle_handle_func callback){
    for (int i=0; i<ARRAY_SIZE(uart_rx_idle_callback); ++i)
    {
        if (uart_rx_idle_callback[i].huart == NULL)
        {
            uart_rx_idle_callback[i].huart = huart;
            uart_rx_idle_callback[i].callback = callback;
            break;
        }
    }
}

// enable global uart it, do not use DMA transfer done it!!!
//static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
//{
//    uint32_t tmp1 = 0;

//    tmp1 = huart->RxState;
//    if (tmp1 == HAL_UART_STATE_READY)
//    {
//        if ((pData == NULL) || (Size == 0))
//        {
//            return HAL_ERROR;
//        }

//        /* Process Locked */
//        __HAL_LOCK(huart);

//        huart->pRxBuffPtr = pData;
//        huart->RxXferSize = Size;
//        huart->ErrorCode  = HAL_UART_ERROR_NONE;

//        /* Enable the DMA Stream */
//        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR,
//                      (uint32_t)pData, Size);

//        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
//        in the UART CR3 register */
//        huart->Instance->CR3 |= USART_CR3_DMAR;

//        /* Process Unlocked */
//        __HAL_UNLOCK(huart);

//        return HAL_OK;
//    }
//    else
//    {
//        return HAL_BUSY;
//    }
//}


/**
  * @brief Receive an amount of data in DMA mode.
  * @note   When the UART parity is enabled (PCE = 1), the received data contain
  *         the parity bit (MSB position).
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param huart UART handle.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
//static HAL_StatusTypeDef HAL_UART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
//{
//  /* Check that a Rx process is not already ongoing */
//  if (huart->RxState == HAL_UART_STATE_READY)
//  {
//    if ((pData == NULL) || (Size == 0U))
//    {
//      return HAL_ERROR;
//    }

//    __HAL_LOCK(huart);

//    huart->pRxBuffPtr = pData;
//    huart->RxXferSize = Size;

//    huart->ErrorCode = HAL_UART_ERROR_NONE;
//    huart->RxState = HAL_UART_STATE_BUSY_RX;

//    if (huart->hdmarx != NULL)
//    {
//      /* Set the UART DMA transfer complete callback */
//    //   huart->hdmarx->XferCpltCallback = UART_DMAReceiveCplt;

//      /* Set the UART DMA Half transfer complete callback */
//    //   huart->hdmarx->XferHalfCpltCallback = UART_DMARxHalfCplt;

//      /* Set the DMA error callback */
//    //   huart->hdmarx->XferErrorCallback = UART_DMAError;

//      /* Set the DMA abort callback */
//      huart->hdmarx->XferAbortCallback = NULL;

//      /* Enable the DMA channel */
//      if (HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)huart->pRxBuffPtr, Size) != HAL_OK)
//      {
//        /* Set error code to DMA */
//        huart->ErrorCode = HAL_UART_ERROR_DMA;

//        __HAL_UNLOCK(huart);

//        /* Restore huart->gState to ready */
//        huart->gState = HAL_UART_STATE_READY;

//        return HAL_ERROR;
//      }
//    }
//    __HAL_UNLOCK(huart);

//    /* Enable the UART Parity Error Interrupt */
//    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

//    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

//    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
//    in the UART CR3 register */
//    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

//    return HAL_OK;
//  }
//  else
//  {
//    return HAL_BUSY;
//  }
//}

//void bt_init()
//{
//    __HAL_UART_CLEAR_IDLEFLAG(&BT_HUART);
//    __HAL_UART_ENABLE_IT(&BT_HUART, UART_IT_IDLE);
//    UART_Receive_DMA_No_IT(&BT_HUART, bt_rx_buff, BT_RX_MAX_BUFLEN);
//}

//void midi_uart_init()
//{
//    __HAL_UART_CLEAR_IDLEFLAG(&MIDI_HUART);
//    __HAL_UART_ENABLE_IT(&MIDI_HUART, UART_IT_IDLE);
//    UART_Receive_DMA_No_IT(&MIDI_HUART, midi_buff, MIDI_RX_BUFF_LEN);
//}

/**
        *@bref use uart idle it + dma(no it) to receive a frame data.
        */
void uart_idle_dma_rx_init(UART_HandleTypeDef* huart, uint8_t* rx_buff, uint16_t buff_len)
{
//    __HAL_UART_CLEAR_IDLEFLAG(huart);
#ifdef STM32F0
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
#elif defined STM32F1
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);

#elif defined STM32F3
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);

#elif defined STM32F4
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);

#elif defined STM32G4
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
#elif defined STM32H7
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
#else
    #error "fixme"
#endif
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
//    UART_Receive_DMA_No_IT(huart, rx_buff, buff_len);
    //UART_Receive_DMA_No_IT(huart, rx_buff, buff_len);
	HAL_UART_Receive_DMA(huart, rx_buff, buff_len);
}

static void uart_reset_idle_rx_callback(UART_HandleTypeDef* huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        // clear idle it flag
        //重启DMA
        uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(
            huart->hdmarx); //根据串口的不同来选择清除不同的DMA标志位

        __HAL_DMA_DISABLE(huart->hdmarx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, MAX_DMA_COUNT);
        // huart->hdmarx->Instance->CNDTR = MAX_DMA_COUNT;

        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

#if defined(DBUS_HUART)
void dbus_init(rc_info_t* prc)
{
    prc = &rc;
    uart_idle_dma_rx_init(&DBUS_HUART, dbus_buff, DBUS_RX_MAX_BUFLEN);
}
#endif

typedef __packed struct{
    uint16_t freq;
    uint8_t on;
    //uint16_t time;
}midi_t;

midi_t midi;
void dma_rc_dbus_decode_callback(rc_info_t* rc, uint8_t* buff);
//这个函数放在BT对应的 Usart-IRQHandler 里面
void langgo_uart_dma_idle_it_callback(UART_HandleTypeDef* huart)
{

    // FloatConvertType tmp;
    // confirm uart idle it set!
    for (int i=0; i<ARRAY_SIZE(uart_rx_idle_callback); ++i)
    {
        if ( NULL != uart_rx_idle_callback[i].callback 
            && huart == uart_rx_idle_callback[i].huart  
            )
        {
           uint16_t rx_len = MAX_DMA_COUNT - __HAL_DMA_GET_COUNTER(huart->hdmarx);
           
           uart_rx_idle_callback[i].callback(huart->pRxBuffPtr, rx_len);
        }
        else {
            break;
        }
    }
    
#if defined(DBUS_HUART)
    if (huart == &DBUS_HUART) {
        // HAL_UART_Receive_DMA(&huart4, rcUnion.buff, RC_Frame_Lentgh);
        dma_rc_dbus_decode_callback(&rc, dbus_buff);
        // err_detector_hook(DbusTOE);
        //使用DMA来接收，每当空闲中断的时候，也就是不收数据了的时候处理接收到的数据。
    }
#endif
#ifdef CV_HUART
    if (huart == &CV_HUART)
    {
    }
#endif

#ifdef MIDI_HUART
    if (huart == &MIDI_HUART)
    {
        memcpy(&midi, midi_buff, sizeof(midi));
        //if (midi_buff[0] == 'm')
        {
            uint16_t freq = midi_buff[1] << 8 | midi_buff[0];
            uint8_t on = midi_buff[2]; 
            TIM12->ARR = 100000.0f / freq;
            if (on)
                TIM12->CCR1 = TIM12->ARR/2;
            else 
                TIM12->CCR1 = 0;
        }
    }
#endif

    uart_reset_idle_rx_callback(huart);
}

// Warning！ ARMAPI shoule not be remove？！
// ARMAPI void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	//
void dma_rc_dbus_decode_callback(rc_info_t* rc, uint8_t* buff)
{
    //	int six_total = buff[0] + buff[1] + buff[2]
    //								+ buff[3] +
    // buff[4]
    //+
    // buff[5];
    //	if(0 == six_total)return;	//可能出现偶然情况么 - -。。。
    if (buff[0] == 0 && buff[1] == 0 && buff[2] == 0 && buff[3] == 0 && buff[4] == 0 && buff[5] == 0)
        return;
    // err_detector_callback(DbusTOE);	//a hook of dbus rx monitor

    //如果数组前面表示通道的全部为0，说明是错误数据则return;
    //	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12]; // is pressed?
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if (huart == &huart8) {
//      extern uint8_t buff8[32];
//    dma_rc_dbus_decode_callback(&rc, buff8);
//  }
//}

// void uart_send_frame(UART_HandleTypeDef* huart, RcTableType id)
// {
//     FloatConvertType tmp;
//     transmit_frame[Head1]  = 0x55;
//     transmit_frame[Head2]  = 0xAA;
//     transmit_frame[DataID] = id;

//     tmp.F32type = rcDataTable[id].F32type;

//     transmit_frame[Byte0] = tmp.U8type[0];
//     transmit_frame[Byte1] = tmp.U8type[1];
//     transmit_frame[Byte2] = tmp.U8type[2];
//     transmit_frame[Byte3] = tmp.U8type[3];

//     transmit_frame[SumCheck] = (uint8_t)(
//         transmit_frame[DataID] + transmit_frame[Byte0] + transmit_frame[Byte1] + transmit_frame[Byte2] + transmit_frame[Byte3]);
//     transmit_frame[Tail] = 0xFF;

//     HAL_UART_Transmit(huart, transmit_frame, FrameLength, 100);
//     // use blocking mode transmit
// }

// void uart_send2pc(UART_HandleTypeDef* huart, RcTableType id, float value)
// {
//     rcDataTable[id].F32type = value;
//     uart_send_frame(huart, id);
// }

// void uart_send4byte(UART_HandleTypeDef* huart, RcTableType id, void* buff)
// {

//     u8* p                     = buff;
//     rcDataTable[id].U8type[0] = *p;
//     rcDataTable[id].U8type[1] = *(p + 1);
//     rcDataTable[id].U8type[2] = *(p + 2);
//     rcDataTable[id].U8type[3] = *(p + 3);

//     uart_send_frame(huart, id);
// }

// void bt_printf(const char* fmt, ...)
// {
//     char    buff[128] = { 0 };
//     char*   p         = buff;
//     va_list ap;
//     //	__va_list ap;
//     va_start(ap, fmt);
//     vsprintf(buff, fmt, ap);

//     int size = 0;
//     while (*p++)
//     {
//         size++;
//     }

//     HAL_UART_Transmit(&BT_HUART, (u8*)buff, size, 1000);
//     va_end(ap);
// }
