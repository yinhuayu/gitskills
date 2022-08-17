#include "fdcan.h"
#include "stm32h7xx.h"
#include <stdint.h>
#include "log.h"

FDCAN_HandleTypeDef hfdcan2;
uint8_t TxData[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[16];
FDCAN_TxHeaderTypeDef TxHeader;

#if defined(FDCAN1) || defined(FDCAN2)

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) != RESET)
    {
        /* Retreive Rx messages from RX FIFO0 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
        //    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, &RxData[8]);

        LOGI("[CAN] fifo watermark rx%s", RxData);
        // HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

        /* Activate Rx FIFO 0 watermark notification */
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* Retreive Rx messages from RX FIFO0 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
        //    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, &RxData[8]);
        LOGI("[CAN] new msg rx%s", RxData);
        // HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

        /* Activate Rx FIFO 0 watermark notification */
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}
#endif

//current cfg clk = 48mhz
void fdcan_init_baud(FDCAN_GlobalTypeDef *FDCANX, uint32_t can_clk, uint32_t bitrate)
{
    assert_param((FDCANX == FDCAN1) || (FDCANX == FDCAN2));
    FDCAN_HandleTypeDef *hfdcan = FDCANX == FDCAN1 ? &hfdcan1 : &hfdcan2;

    // uint16_t psc = 1;
    uint32_t calc_baud;

    for (int psc = 1; psc < 1024; ++psc)
    {
        for (int bs1 = 1; bs1 <= 16; ++bs1)
        {
            for (int bs2 = 1; bs2 <= 8; ++bs2)
            {
                if (can_clk % (psc * (1 + bs1 + bs2)))
                {
                    continue; //no interger sulution
                }
                calc_baud = can_clk / psc / (1 + bs1 + bs2);
                if (calc_baud <= 1000000 && bitrate == calc_baud)
                {
                    hfdcan->Init.NominalPrescaler = psc;
                    hfdcan->Init.NominalSyncJumpWidth = 1;
                    hfdcan->Init.NominalTimeSeg1 = bs1;
                    hfdcan->Init.NominalTimeSeg2 = bs2;
                    hfdcan->Init.DataPrescaler = psc;
                    hfdcan->Init.DataSyncJumpWidth = 1;
                    hfdcan->Init.DataTimeSeg1 = bs1;
                    hfdcan->Init.DataTimeSeg2 = bs2;
                    LOGI("\tpsc %d \tbs1 %d \tbs2 %d \tbps  %d", psc, bs1, bs2, calc_baud);
                    // HAL_Delay(1);
                    goto found_solution;
                }
            }
        }
    }
    while (1)
        ;
    //fail here.

found_solution:
    //baud = clk / psc / (1+bs1+bs2);
    //let 1+bs1+bs2 = 5.
    hfdcan->Instance = FDCANX;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;
    hfdcan->Init.RxBuffersNbr = DISABLE;
    hfdcan->Init.AutoRetransmission = DISABLE;
    hfdcan->Init.MessageRAMOffset = 0;
    hfdcan->Init.StdFiltersNbr = 2;
    hfdcan->Init.ExtFiltersNbr = 0;
    hfdcan->Init.RxFifo0ElmtsNbr = 1;
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxFifo1ElmtsNbr = 1;
    hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxBuffersNbr = 0;
    hfdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxEventsNbr = 0;
    hfdcan->Init.TxBuffersNbr = 0;
    hfdcan->Init.TxFifoQueueElmtsNbr = 1;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void langgo_can_init(uint32_t can_clk)
{
    fdcan_init_baud(FDCAN1, can_clk, 1000000);
    fdcan_init_baud(FDCAN2, can_clk, 1000000);

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0; //Classic filter: FilterID1 = filter, FilterID2 = mask
    sFilterConfig.FilterID2 = 0; /* For acceptance, MessageID and FilterID1 must match exactly */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* Configure Rx FIFO 0 watermark to 2 */
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 2);
    /* Activate Rx FIFO 0 watermark notification */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);

    /* Configure Rx FIFO 0 watermark to 2 */
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 2);
    /* Activate Rx FIFO 0 watermark notification */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* Prepare Tx Header */
    TxHeader.Identifier = 0x100;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    /* Start the FDCAN module */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
}

volatile uint32_t tx_failed;
int can_send(uint16_t stdid, uint8_t data[], uint8_t len)
{
    TxHeader.Identifier = stdid;
    //HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData)
    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
    if (ret != HAL_OK)
    {
        tx_failed++;
    }
    HAL_StatusTypeDef ret2 = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data);
    if (ret2 != HAL_OK)
    {
        tx_failed++;
    }
    return ret;
}

void set_moto_speed(FDCAN_HandleTypeDef *hcan, uint16_t stdid, int16_t v[])
{
    can_send(0x427, (uint8_t *)v, 8);
}

__WEAK void FDCAN1_IT0_IRQHandler()
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
