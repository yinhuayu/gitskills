#include "systick.h"
#include "usart.h"
#include "foc.h"
#include "fitness.h"
#include "PROTECT.h"

UART_485_Frame_TypeDef UART1_Frame_Box;

uint8_t USART_RX_BUF[10];
uint8_t USART_RX_DATA[10];
uint8_t USART_TX_DATA[23];

int16_t Speed_M1, Speed_M2;
int16_t Distance_M1, Distance_M2;
int16_t Iq_return_M1, Iq_return_M2;
extern uint16_t InitPullBackSpeed;
extern uint16_t MaxSpeedLimit;     //正常运行最大速度限制 不超过200cm/s  （默认PullBackSpeed = 4000  因约为D10cm  所以 200cm/s）
extern uint16_t PreventCrashSpeed; //回收过程中，防止撞击提前减速的速度域值 不超过200cm/s  （默认PullBackSpeed = 4000  因约为D10cm  所以 200cm/s）
extern uint16_t PullBackDistan;
extern uint16_t ForceLimitPos;
extern uint16_t InitTorque;
extern uint16_t ResTorque;
extern uint16_t ConstantSpeedBackTorque;
extern uint8_t RatioUp;
extern uint8_t RatioDown;
uint8_t MC_ID = 1; // motor controler ID  驱动器号
uint32_t Torque_buf = 2000;
float Cycles_buf = 0.1;
int Amplitude_buf = 4;
extern fitness_t fit_M1;
extern fitness_t fit_M2;

extern uint8_t ErrCoder;
extern int Mos_Temp;
extern int M1_Temp;
extern int M2_Temp;
int Pull_Num_M1 = 0;
bool Pull_flag_M1 = 0;
bool Pull_flag_M1_t = 0;

int Pull_Num_M2 = 0;
bool Pull_flag_M2 = 0;
bool Pull_flag_M2_t = 0;
foc_var_int_t fv_M1;
foc_var_int_t fv_M2;
fitness_t fit_M1_buf;
fitness_t fit_M2_buf;
// typedef struct _fitness
// {
//     uint8_t mode;
//     uint32_t torque;    //unit: mA
// 		uint32_t torque_last;    //unit: mA
//     int32_t pos_offset;
//     int32_t circule_cnt;
//     int32_t pull_distance;
//     int32_t pullback_distance;
//     int16_t Parameter
// }fitness_t;
fitness_t fit_M2_buf;

//模型：CRC-8/MAXIM
//多项式：x8 + x5 + x4 + 1
const char CRC8Table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

uint8_t CRC8_Table(unsigned char *p, int counter)
{
    uint8_t crc8 = 0;

    for (; counter > 0; counter--)
    {
        crc8 = CRC8Table[crc8 ^ *p];
        p++;
    }
    return (crc8);
}

uint8_t crc_check(uint8_t size, unsigned char *ONE_Buf)
{
    static uint8_t crc;

    crc = CRC8_Table(ONE_Buf, size);

    return crc;
}

// u1tx pa2 u1rx pa3
void uart1_init()
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);
    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    nvic_irq_enable(USART1_IRQn, 0, 0);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    usart_enable(USART1);
}

void uart_write(uint32_t uartx, uint8_t *data, uint16_t size)
{
    for (int i = 0; i < size; i++)
    {
        usart_data_transmit(USART1, (uint8_t)data[i]);
        while (RESET == usart_flag_get(USART1, USART_FLAG_TBE))
            ;
    }
}

void uart_send_state()
{
    Speed_M1 = -fv_M1.speed_rpm / 20;                          // 转为拉绳速度 单位 cm/s        10rpm/10 *30cm /60s/min
    Distance_M1 = (ABS(-fv_M1.pid_pos.err) - 1500) * 28 >> 11; // 2048/R  D约90mm     L单位CM
    if (Distance_M1 < 0)
    {
        Distance_M1 = 0;
    }
    Iq_return_M1 = fv_M1.iq * 10 >> 12;

    if (Speed_M1 > 20)
    {
        Pull_flag_M1 = 1;
    }

    if (Speed_M1 < -20)
    {
        Pull_flag_M1 = 0;
    }

    if (Pull_flag_M1 != Pull_flag_M1_t)
    {

        Pull_Num_M1++;
    }
    Pull_flag_M1_t = Pull_flag_M1;

    Speed_M2 = fv_M2.speed_rpm / 20;                          // 转为拉绳速度 单位 cm/s        10rpm/10 *30cm /60s/min
    Distance_M2 = (ABS(fv_M2.pid_pos.err) - 1500) * 28 >> 11; /// 2048/R  D约90mm     L单位 cm;

    if (Distance_M2 < 0)
    {
        Distance_M2 = 0;
    }

    Iq_return_M2 = -fv_M2.iq * 10 >> 12; //    转为kg为单位   拉出时转换为正值

    if (Speed_M2 > 20)
    {
        Pull_flag_M2 = 1;
    }

    if (Speed_M2 < -20)
    {
        Pull_flag_M2 = 0;
    }

    if (Pull_flag_M2 != Pull_flag_M2_t)
    {

        Pull_Num_M2++;
    }
    Pull_flag_M2_t = Pull_flag_M2;
    USART_TX_DATA[0] = 0x01;
    USART_TX_DATA[1] = 0x64;

    USART_TX_DATA[2] = Iq_return_M1 >> 8; //
    USART_TX_DATA[3] = Iq_return_M1;      //
    USART_TX_DATA[4] = Speed_M1 >> 8;     //
    USART_TX_DATA[5] = Speed_M1;

    USART_TX_DATA[6] = Distance_M1 >> 8;
    USART_TX_DATA[7] = Distance_M1;
    USART_TX_DATA[8] = Pull_Num_M1 >> 1;
    USART_TX_DATA[9] = ErrCoder;
    ///////////////////////////////////////////////
    USART_TX_DATA[10] = Iq_return_M2 >> 8; //
    USART_TX_DATA[11] = Iq_return_M2;      //
    USART_TX_DATA[12] = Speed_M2 >> 8;     //
    USART_TX_DATA[13] = Speed_M2;

    USART_TX_DATA[14] = Distance_M2 >> 8;
    USART_TX_DATA[15] = Distance_M2;
    USART_TX_DATA[16] = Pull_Num_M2 >> 1;
    if (Mos_Temp > 0)
    {
        USART_TX_DATA[17] = Mos_Temp;
    }
    else
    {
        USART_TX_DATA[17] = 0;
    }
    USART_TX_DATA[18] = fit_M1.mode;
    USART_TX_DATA[19] = fit_M2.mode;
    if (M1_Temp > 0)
    {
        USART_TX_DATA[20] = M1_Temp;
    }
    else
    {
        USART_TX_DATA[20] = 0;
    }
    if (M2_Temp > 0)
    {
        USART_TX_DATA[21] = M2_Temp;
    }
    else
    {
        USART_TX_DATA[21] = 0;
    }

    // USART_TX_DATA[8]  = Fitness_motor.Pull_Num;
    //  USART_TX_DATA[9]  = ERR_CODE[err_code];

    // USART_TX_DATA[18] = fit_M1.mode;

    USART_TX_DATA[22] = crc_check(22, USART_TX_DATA);
    uart_write(USART1, USART_TX_DATA, 23);
}

void USART1_IRQHandler()
{
    static uint8_t Res;
    static uint8_t USART_RX_STA;
    static uint8_t ID_SET_CNT;

    if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
    {
        /* read one byte from the receive data register */
        Res = (uint8_t)usart_data_receive(USART1);
        USART_RX_BUF[USART_RX_STA] = Res;
        USART_RX_STA++;

        if (USART_RX_BUF[0] == MC_ID)
        {
            if (USART_RX_BUF[1] == 0x64)
            {
                USART_RX_DATA[USART_RX_STA - 1] = USART_RX_BUF[USART_RX_STA - 1];

                if (USART_RX_STA == 10 && USART_RX_DATA[9] == crc_check(9, USART_RX_DATA))
                {
                    if ((fit_M1.mode != Star_Mode) && (fit_M2.mode != Star_Mode))
                    {
                        fit_M1_buf.mode = USART_RX_DATA[2];
                        fit_M2_buf.mode = USART_RX_DATA[2];

                        if (USART_RX_DATA[3] > 80)
                        {
                            USART_RX_DATA[3] = 80;
                        }
                        if (USART_RX_DATA[6] > 80)
                        {
                            USART_RX_DATA[6] = 80;
                        }
                        fit_M1_buf.torque = USART_RX_DATA[3] * 198; //电机1 转矩接收 //经过校正，*198能使输入力控制值近似等于静止时的输出力0.5kg为单位 0-80 对应0-40kg
                        fit_M2_buf.torque = USART_RX_DATA[6] * 198; //电机2 转矩接收 //经过校正，*198能使输入力控制值近似等于静止时的输出力0.5kg为单位 0-80 对应0-40kg

                        // if (USART_RX_DATA[4] == 0)
                        // {
                        //     RatioUp = 1;
                        // }
                        // if (USART_RX_DATA[4] >= 100)
                        // {
                        //     RatioUp = 100;
                        // }
                        // RatioUp = USART_RX_DATA[4];

                        if (USART_RX_DATA[7] == 0x01)
                        {
                            Pull_Num_M1 = 0;
                        }
                        if (USART_RX_DATA[7] == 0x02)
                        {
                            Pull_Num_M2 = 0;
                        }

                        USART_RX_DATA[9] = crc_check(9, USART_RX_DATA);
                        uart_write(USART1, USART_RX_DATA, 10);
                        USART_RX_DATA[7] = 0;
                    }
                }
            }

#if 0 // SD LD 山东蓝德的内容段
            if (USART_RX_BUF[1] == 0x70)
            {
                USART_RX_DATA[USART_RX_STA - 1] = USART_RX_BUF[USART_RX_STA - 1];

                if (USART_RX_STA == 10 && USART_RX_DATA[9] == crc_check(9, USART_RX_DATA))
                {
                    if ((fit_M1.mode != Star_Mode) && (fit_M2.mode != Star_Mode))
                    {
                        fit_M1_buf.mode = USART_RX_DATA[2];
                        fit_M2_buf.mode = USART_RX_DATA[2];

                        if (USART_RX_DATA[3] > 200)
                        {
                            USART_RX_DATA[3] = 200;
                        }
                        if (USART_RX_DATA[6] > 200)
                        {
                            USART_RX_DATA[6] = 200;
                        }
                        fit_M1_buf.torque = USART_RX_DATA[3] * 115; //电机1 转矩接收 //经过校正，*能使输入力控制值近似等于静止时的输出力0.5kg为单位 0-80 对应0-40kg
                        fit_M2_buf.torque = USART_RX_DATA[6] * 115; //电机2 转矩接收 //经过校正，*198能使输入力控制值近似等于静止时的输出力0.5kg为单位 0-80 对应0-40kg
                        RatioUp = USART_RX_DATA[4];
                        RatioDown = USART_RX_DATA[5];

                        if (RatioDown == 0)
                        {
                            RatioDown = 1;
                        }
                        if (USART_RX_DATA[7] == 0x01)
                        {
                            Pull_Num_M1 = 0;
                        }
                        if (USART_RX_DATA[7] == 0x02)
                        {
                            Pull_Num_M2 = 0;
                        }

                        USART_RX_DATA[9] = crc_check(9, USART_RX_DATA);
                        uart_write(USART1, USART_RX_DATA, 10);
                    }
                }
            }
#endif
            if (USART_RX_BUF[1] == 0x50)
            {
                USART_RX_DATA[USART_RX_STA - 1] = USART_RX_BUF[USART_RX_STA - 1];

                if (USART_RX_STA == 10 && USART_RX_DATA[9] == crc_check(9, USART_RX_DATA))
                {
                    if (USART_RX_DATA[2] > 30) //初始回绳速度不超过30cm/s
                    {
                        USART_RX_DATA[2] = 30;
                    }
                    InitPullBackSpeed = USART_RX_DATA[2] * 20; //（默认InitPullBackSpeed=300  因D10cm  所以 15cm/s） USART_RX_DATA[2]15  cm/s

                    if (USART_RX_DATA[3] > 200) //回绳速度不超过200cm/s
                    {
                        USART_RX_DATA[3] = 200;
                    }
                    MaxSpeedLimit = USART_RX_DATA[3] * 20; // USART_RX_DATA[3] 200cm/s    （默认PullBackSpeed = 4000  因D10cm  所以 200cm/s）

                    if (USART_RX_DATA[4] > 50) //起始前移距离不超过50cm 默认21CM
                    {
                        USART_RX_DATA[4] = 50;
                    }
                    PullBackDistan = USART_RX_DATA[4] * 2048 / 28; // USART_RX_DATA[4](CM) 默认21CM PullBackDistan = 1500

                    if (USART_RX_DATA[5] > 50) //保护点后移距离不超过50cm 默认7CM
                    {
                        USART_RX_DATA[5] = 50;
                    }
                    ForceLimitPos = USART_RX_DATA[5] * 2048 / 28; // USART_RX_DATA[5](CM) 默认4CM ForceLimitPos = 500

                    if (USART_RX_DATA[6] > 6) //系统阻力补偿值（回绳力补偿）
                    {
                        USART_RX_DATA[6] = 6;
                    }
                    ResTorque = USART_RX_DATA[6] * 198; //默认ResTorque = 396 USART_RX_DATA[6]，2  0-80 对应0-40kg
                                                        // Torque_Set_M1 +=ResTorque * Torque_Set_M1 / 3960; //根据力的大小补偿回绳拉力
                    if (USART_RX_DATA[7] < 4)           //初始保持力 不小于2kg 默认 3.5kg设定，太小电机可能无法初始运转
                    {
                        USART_RX_DATA[7] = 4;
                    }
                    if (USART_RX_DATA[7] > 14) //初始保持力 不大于7kg 默认 2.5kg
                    {
                        USART_RX_DATA[7] = 14;
                    }
                    InitTorque = USART_RX_DATA[7] * 198; //默认InitTorque =1386  USART_RX_DATA[7]，7  0-80 对应0-40kg

                    if (USART_RX_DATA[8] > 40) //初始保持力 不大于20kg
                    {
                        USART_RX_DATA[8] = 40;
                    }
                    if (USART_RX_DATA[8] < 3) //初始保持力 不小于2kg
                    {
                        USART_RX_DATA[8] = 3;
                    }
                    ConstantSpeedBackTorque = USART_RX_DATA[8] * 198; //默认3.5kg ConstantSpeedBackTorque =1386 USART_RX_DATA[8]，7  0-80 对应0-40kg

                    USART_RX_DATA[9] = crc_check(9, USART_RX_DATA);
                    uart_write(USART1, USART_RX_DATA, 10);
                    USART_RX_DATA[7] = 0;
                }
            }
            if (USART_RX_BUF[1] == 0x51)
            {
                USART_RX_DATA[USART_RX_STA - 1] = USART_RX_BUF[USART_RX_STA - 1];

                if (USART_RX_STA == 10 && USART_RX_DATA[9] == crc_check(9, USART_RX_DATA))
                {
                    if (USART_RX_DATA[2] > 200) //初始回绳速度不超过30cm/s
                    {
                        USART_RX_DATA[2] = 200;
                    }
                    PreventCrashSpeed = USART_RX_DATA[2] * 20; // USART_RX_DATA[2]cm/s    （默认PreventCrashSpeed = 2000  因D10cm  所以 100cm/s）

                    USART_RX_DATA[2] = 0;
                    USART_RX_DATA[3] = 0;
                    USART_RX_DATA[4] = 0;
                    USART_RX_DATA[5] = 0;
                    USART_RX_DATA[6] = 0;
                    USART_RX_DATA[7] = 0;
                    USART_RX_DATA[8] = 0;
                    USART_RX_DATA[9] = crc_check(9, USART_RX_DATA);
                    uart_write(USART1, USART_RX_DATA, 10);
                }
            }
#if 0 //调试模式可打开
            if (USART_RX_BUF[1] == 0x25) // FYL测试 模式 无CRC                        //USART_RX_DATA[1]
            {
                USART_RX_DATA[USART_RX_STA - 1] = USART_RX_BUF[USART_RX_STA - 1];
                if (USART_RX_STA == 10)
                {
                    fit_M1_buf.mode = USART_RX_DATA[2];
                    fit_M2_buf.mode = USART_RX_DATA[2];
                    fit_M1_buf.torque = USART_RX_DATA[3] * 32767 / 255; //电机1 转矩接收//USART_RX_DATA[3]
                    fit_M2_buf.torque = USART_RX_DATA[3] * 32767 / 255; //电机2 转矩接收//USART_RX_DATA[3]

                    Cycles_buf = (float)USART_RX_DATA[4] / 10.0f; // USART_RX_DATA[4]
                    Amplitude_buf = USART_RX_DATA[5];             // USART_RX_DATA[5]

                    uart_write(USART1, USART_RX_DATA, 10);
                }
            }
#endif
        }
        else
        {
            USART_RX_STA = 0;
        }

        if (USART_RX_STA >= 10)
        {
            USART_RX_STA = 0;
        }
        else if (USART_RX_STA == 2 && USART_RX_BUF[1] != 0x70 && USART_RX_BUF[1] != 0x64 && USART_RX_BUF[1] != 0x05 && USART_RX_BUF[1] != 0x50)
        {
            USART_RX_STA = 0;
        }
    }
}
