#include "PROTECT.h"

extern volatile uint16_t adc1_regular[CH_TOTAL_NUM];

// led = pb2

unsigned int PWM = 4; //  0<pwm<9
unsigned int fan_cnt;

unsigned int TP_Mos_cnt = 0;
unsigned int TP_M1_cnt = 0;
unsigned int TP_M2_cnt = 0;
int MTempMax = 0;
int Mos_Temp = 25;
int M1_Temp = 25;
int M2_Temp = 25;
int I_bus = 0;	   // mA
int U_bus = 0;	   // mV
uint8_t ErrM1 = 0; // 0b     XX                   XX                 XX                  XX
uint8_t ErrM2 = 0; //     U限制/警报           I限制/警报      mos温度限制/警报     线圈温度限制/警报
uint8_t ErrCoder = 0;
uint8_t TemHiflag = 0;
uint8_t Err_F_T_M1 = 0;	 // 0x0：无错位   	0x:11：M1高温警告       0x12：M1高温限制      0x13：M1高温失效
uint8_t Err_F_T_M2 = 0;	 //					0x:01：M2高温警告       0x02：M1高温限制      0x03：M1高温失效
uint8_t Err_F_T_PCB = 0; //              	0x:01：PCBA高温警告     0x02：PCB高温限制     0x03：PCB高温失效
uint8_t Err_F_Sp = 0;	 //              		0x:01：超速警告     	0x02：超速限制     	  0x03：超速失效
uint8_t Err_F_Vol = 0;	 //              		0x:01：高电压警告     	0x02：高电压限制      0x03：高电压失效
uint8_t Err_F_Cur = 0;	 //              		0x:01：高电流警告     	0x02：高电流限制      0x03：高电流失效
extern  fitness_t fit_M1;
extern  fitness_t fit_M2;
void U_I_protect()
{
	U_bus = vbus_calc(adc1_regular[CH_VBUS], RES_VCC, RES_GND);
	I_bus = ibus_calc_cc6920(adc1_regular[CH_IBUS]);

	if (U_bus > 60000)
	{
		ErrM1 &= ~(0b11 << 6);
		ErrM1 |= (0x10 << 6);
		ErrM2 &= ~(0b11 << 6);
		ErrM2 |= (0x10 << 6);
		Err_F_Vol = 3;
	}

	else if (U_bus > 57000) //>56V
	{
		ErrM1 |= (0x11 << 6);
		ErrM2 |= (0x11 << 6);
		Err_F_Vol = 2;
	}
	else if ((U_bus <= 57000) && (U_bus > 54500)) //>57V 54V
	{
		ErrM1 &= ~(0b11 << 6);
		ErrM1 |= (0x01 << 6);
		ErrM2 &= ~(0b11 << 6);
		ErrM2 |= (0x01 << 6);
		Err_F_Vol = 1;
	}
	else
	{
		ErrM1 &= ~(0b11 << 6);
		ErrM2 &= ~(0b11 << 6);
		Err_F_Vol = 0;
	}

	/////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	if (I_bus > 25000)
	{
		ErrM1 &= ~(0b11 << 4);
		ErrM1 |= (0x10 << 4);
		ErrM2 &= ~(0b11 << 4);
		ErrM2 |= (0x10 << 4);
		Err_F_Cur = 3;
	}
	else if (I_bus > 20000) //>25A
	{

		ErrM1 |= (0x11 << 4);
		ErrM2 |= (0x11 << 4);
		Err_F_Cur = 2;
	}
	else if ((I_bus <= 20000) && (I_bus > 18000)) //>20A
	{

		ErrM1 &= ~(0b11 << 4);
		ErrM1 |= (0x01 << 4);
		ErrM2 &= ~(0b11 << 4);
		ErrM2 |= (0x01 << 4);
		Err_F_Cur = 1;
	}
	else
	{
		ErrM1 &= ~(0b11 << 4);
		ErrM2 &= ~(0b11 << 4);
		Err_F_Cur = 0;
	}
}
uint8_t TimePortect = 0; // 记次器
int32_t Mos_Temp_array[11];
int32_t M1_Temp_array[11];
int32_t M2_Temp_array[11];

void Temp_protect()
{
	if (TimePortect < 100) // 10S
	{
		TimePortect++;
	}
	else
	{
		TimePortect = 0;
	}

	Mos_Temp = get_temp_mos_ntc(adc1_regular[CH_MOS_TEMP]);
	M1_Temp = get_temp_motor(adc1_regular[CH_NTC0]);
	M2_Temp = get_temp_motor(adc1_regular[CH_NTC1]);

	//移动滤波
	Mos_Temp_array[10] -= Mos_Temp_array[TimePortect % 10];
	Mos_Temp_array[TimePortect % 10] = Mos_Temp;
	Mos_Temp_array[10] += Mos_Temp;
	Mos_Temp = Mos_Temp_array[10] / 10;

	M1_Temp_array[10] -= M1_Temp_array[TimePortect % 10];
	M1_Temp_array[TimePortect % 10] = M1_Temp;
	M1_Temp_array[10] += M1_Temp;
	M1_Temp = M1_Temp_array[10] / 10;

	M2_Temp_array[10] -= M2_Temp_array[TimePortect % 10];
	M2_Temp_array[TimePortect % 10] = M2_Temp;
	M2_Temp_array[10] += M2_Temp;
	M2_Temp = M2_Temp_array[10] / 10;

	//温度保护判定
	if (Mos_Temp > 100)
	{
		ErrM1 &= ~(0b11 << 2); // 2.3位置零
		ErrM1 |= (0x10 << 2);  // 3位置1
		ErrM2 &= ~(0b11 << 2); // 2.3位置零
		ErrM2 |= (0x10 << 2);  // 3位置1
		Err_F_T_PCB = 3;
	}
	else if (Mos_Temp > 90)
	{
		ErrM1 |= (0x11 << 2); // 3位置1
		ErrM2 |= (0x11 << 2); // 3位置1
		Err_F_T_PCB = 2;
	}
	else if ((Mos_Temp <= 90) && (Mos_Temp > 85))
	{
		ErrM1 &= ~(0b11 << 2); // 2.3位置零
		ErrM1 |= (0x01 << 2);  // 2位置1
		ErrM2 &= ~(0b11 << 2); // 2.3位置零
		ErrM2 |= (0x01 << 2);  // 2位置1
		Err_F_T_PCB = 1;
	}
	else // Mos_Temp < 85
	{
		ErrM1 &= ~(0b11 << 2); // 2.3两位置零
		ErrM2 &= ~(0b11 << 2); // 2.3两位置零
		Err_F_T_PCB = 0;
	}
	//////////////////////////////////////////////////
	if (M1_Temp > 110)
	{
		ErrM1 &= ~(0b11); // 0.1位置零
		ErrM1 |= (0x10);  // 1位置1
		Err_F_T_M1 = 3;
	}
	else if (M1_Temp > 100)
	{
		ErrM1 |= (0x11); // 0,1位置1
		Err_F_T_M1 = 2;
	}
	else if ((M1_Temp <= 100) && (M1_Temp > 90))
	{
		ErrM1 &= ~(0b11); // 0.1位置零
		ErrM1 |= (0x01);  // 1位置1
		Err_F_T_M1 = 1;
	}
	else
	{
		ErrM1 &= ~(0b11); // 0.1两位置零
		Err_F_T_M1 = 0;
	}
	////////////////////////////////////////////////////
	if (M2_Temp > 110)
	{
		ErrM2 &= ~(0b11); // 0.1位置零
		ErrM2 |= (0x10);  // 1位置1
		Err_F_T_M2 = 3;
	}
	else if (M2_Temp > 100)
	{
		ErrM2 |= (0x11); // 0,1位置1
		Err_F_T_M2 = 2;
	}
	else if ((M2_Temp <= 100) && (M2_Temp > 90))
	{
		ErrM2 &= ~(0b11); // 0.1位置零
		ErrM2 |= (0x01);  // 1位置1
		Err_F_T_M2 = 1;
	}
	else
	{
		ErrM2 &= ~(0b11); // 0.1两位置零
		Err_F_T_M2 = 0;
	}

	if ((Err_F_T_M1 == 3) || (Err_F_T_M2 == 3) || (Err_F_T_PCB == 3))
	{
		ErrCoder = 3;
		TemHiflag = 1;
	}
	else if ((Err_F_T_M1 == 2) || (Err_F_T_M2 == 2) || (Err_F_T_PCB == 2))
	{
		ErrCoder = 2;
		TemHiflag = 1;
	}
	else if ((Err_F_T_M1 == 1) || (Err_F_T_M2 == 1) || (Err_F_T_PCB == 1))//温度保护滞回 
	{

		if (TemHiflag == 1)
		{
			ErrCoder = 2;
		}
		else
		{
			ErrCoder = 1;
		}
	}
	else
	{
		ErrCoder = 0;
		TemHiflag = 0;
	}
}

void fan_clt()
{
	fan_cnt++;

	MTempMax = MAX(M1_Temp, M2_Temp);
	MTempMax = MAX(MTempMax, Mos_Temp);

	if (MTempMax <= 35)
	{
		PWM = 0;
	}
	else if (MTempMax > 35 && MTempMax < 75)
	{
		PWM = (MTempMax - 35) / 4;
	}
	else if (MTempMax >= 75)
	{
		PWM = 9;
	}

	if ((fan_cnt % 10) < PWM)
		FAN_ON();
	else
		FAN_OFF();

	if (fit_M1.mode == Eco_Mode && fit_M2.mode == Eco_Mode )
	{
		FAN_OFF();
	}
		
}

void led_fan_init()
{
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure led&fan GPIO port */
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	LED_ON();
	FAN_OFF();
}

void PreDriverM1M2_init()
{
	rcu_periph_clock_enable(RCU_GPIOA);
	/* configure led&fan GPIO port */
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
	PreDriverM1_ON();
	PreDriverM2_ON();
}
// void tp_set(int x) {
//     if (x) {
//         GPIO_BOP(GPIOB) = GPIO_PIN_11;
//     }
//     else {
//         GPIO_BC(GPIOB) = GPIO_PIN_11;
//     }
// }
void led_blink()
{
	static int tog;
	tog ^= 1;
	if (tog)
		LED_ON();
	else
		LED_OFF();
}
