
#include "tim.h"
#include "tim_port.h"
#include "foc.h"
#include "6step.h"
#include "hall.h"

#define T1C4_ADC_INJECT 1

//这里存放硬件（芯片平台）相关的代码
//也许可以使用registration callback的方式会更加美观高效。

#if 0 //defined(USE_FOC)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // htim_hall_xor_capture_callback_foc(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // htim_update_callback_foc(htim);
}

#else
//handle hall xor it.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    htim_hall_xor_capture_callback_foc(htim);
    // set_pwm_ch(hall.raw, 0);
}

//handle svpwm 3ch & foc compute.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // htim_update_callback_6step(htim);
    #if SEE_FOC_UPDATE_PIN
        if (htim == &htim1)
        {
            update_tp_pin_set(1);
            for (int i = 0; i < 10; ++i)
            {
                __NOP();
            }
            update_tp_pin_set(0);
        }
    #endif

    #if FOC_HALL
        if (&htim_hall == htim)
        {
            hall_ovf_update_handler();
        }
    #endif
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    htim_update_callback_foc(htim);
}

#endif

//pwm center align mode.
void pwm_init_foc()
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim_pwm.Instance = TIM_PWM_INSTANCE;
    htim_pwm.Init.Prescaler = 0;
    htim_pwm.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim_pwm.Init.Period = PWM_ARR;
    htim_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_pwm.Init.RepetitionCounter = 0;
    htim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_Base_Init(&htim_pwm) != HAL_OK)
    if (HAL_TIM_PWM_Init(&htim_pwm) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim_pwm, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    // sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_pwm, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; //PWM_ARR/2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; //L6398 fuck truth table.
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0; //PWM_ARR/4;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0; //PWM_ARR/8;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_ARR - 10;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 50;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    // sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    // sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    // sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    // sBreakDeadTimeConfig.Break2Filter = 0;
    // sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim_pwm, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim_pwm);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    __HAL_TIM_ENABLE_IT(&htim_pwm, TIM_IT_UPDATE);

    // HAL_TIMEx_ConfigCommutEvent_IT(&htim_pwm, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim_pwm, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim_pwm, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim_pwm, TIM_CHANNEL_3);

    //trigger injected ADC for current sample or
#if 1   // T1C4_ADC_INJECT
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_4);
#elif 1 //t1 cc so we can adjust trigger time?
    // HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    // HAL_TIM_PWM_Start_IT(&htim_pwm, TIM_CHANNEL_4);

#elif 1
    // htim_pwm.PeriodElapsedCallback = HAL_TIM_PeriodElapsedCallback;
    // htim_pwm.PeriodElapsedCallback = htim_update_callback_foc;
    // htim_pwm.OC_DelayElapsedCallback = htim_update_callback_foc;

    //trigger an foc compute.
    HAL_TIM_Base_Start_IT(&htim_pwm);
#endif

    //after timer started, write to rcr
    htim_pwm.Instance->RCR = 1;
}

void pwm_init_6step()
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim_pwm.Instance = TIM_PWM_INSTANCE;
    htim_pwm.Init.Prescaler = 0;
    htim_pwm.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim_pwm.Init.Period = PWM_ARR;
    htim_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_pwm.Init.RepetitionCounter = 1;
    htim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim_pwm) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim_pwm, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim_pwm) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    // sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_pwm, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; //PWM_ARR/2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0; //PWM_ARR/4;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0; //PWM_ARR/8;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_ARR - 1;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    if (HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 50;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    // sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    // sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    // sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    // sBreakDeadTimeConfig.Break2Filter = 0;
    // sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim_pwm, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim_pwm);

    HAL_TIMEx_ConfigCommutEvent_IT(&htim_pwm, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim_pwm, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim_pwm, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim_pwm, TIM_CHANNEL_3);

    __HAL_TIM_ENABLE_IT(&htim_pwm, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start_IT(&htim_pwm, TIM_CHANNEL_4); //trigger injected ADC for current sample

    // //try start spin motor.
    // HAL_TIM_GenerateEvent(&htim_pwm, TIM_EVENTSOURCE_COM);
}

#ifdef FOC_HALL

void hall_hw_msp_init(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**TIM4 GPIO Configuration    
        PD12    ------> TIM4_CH1
        PD13    ------> TIM4_CH2 
        PD14    ------> TIM4_CH3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}

void hall_hw_init()
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_HallSensor_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim_hall.Instance = HALL_TIMER;
    htim_hall.Init.Prescaler = HALL_TIM_PSC - 1;
    htim_hall.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_hall.Init.Period = 65535;
    htim_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_hall.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    hall_hw_msp_init(&htim_hall);
    if (HAL_TIM_Base_Init(&htim_hall) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim_hall, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 10;
    sConfig.Commutation_Delay = 0;
    if (HAL_TIMEx_HallSensor_Init(&htim_hall, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF; //in hall mode this param unchangeable.
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_hall, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_TIM_ENABLE_IT(&htim_hall, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&htim_hall, TIM_IT_UPDATE); //cnt overflow
    __HAL_TIM_URS_ENABLE(&htim_hall);               //only UPDATE when overflow
    HAL_TIMEx_HallSensor_Start(&htim_hall);
}

void TIM4_IRQHandler()
{
    HAL_TIM_IRQHandler(&htim4);//此处打断点可以知道UEV发生在上溢还是下�?
}
#endif

#if FOC_ENC

void encoder_hw_init()
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim_enc.Instance = TIM_ENC_INSTANCE;
    htim_enc.Init.Prescaler = 0;
    htim_enc.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_enc.Init.Period = ENC_MECH_1CIRCLE-1; //0xffff;
    htim_enc.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_enc.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 11;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 11;
    if (HAL_TIM_Encoder_Init(&htim_enc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_enc, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Encoder_Start(&htim_enc, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim_enc, TIM_CHANNEL_2);
}
#endif // FOC_ENC
// #include "spi.h"
// SPI_HandleTypeDef   hspi1;
// #define hspi_mag    hspi1
// #define MAG_SPI_INSTANCE    SPI1

void magnetic_hw_init()
{
    //   hspi_mag.Instance = MAG_SPI_INSTANCE;
    //   hspi_mag.Init.Mode = SPI_MODE_MASTER;
    //   hspi_mag.Init.Direction = SPI_DIRECTION_2LINES;
    //   hspi_mag.Init.DataSize = SPI_DATASIZE_16BIT;
    //   hspi_mag.Init.CLKPolarity = SPI_POLARITY_LOW;
    //   hspi_mag.Init.CLKPhase = SPI_PHASE_2EDGE;
    //   hspi_mag.Init.NSS = SPI_NSS_SOFT;
    //   hspi_mag.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  //spi clock = apb2 64mhz
    //   hspi_mag.Init.FirstBit = SPI_FIRSTBIT_MSB;
    //   hspi_mag.Init.TIMode = SPI_TIMODE_DISABLE;
    //   hspi_mag.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    //   hspi_mag.Init.CRCPolynomial = 7;
    //   hspi_mag.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    //   hspi_mag.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    //   if (HAL_SPI_Init(&hspi_mag) != HAL_OK)
    //   {
    //     Error_Handler();
    //   }
}

#define FOC_TP_PORT (GPIOB)
#define FOC_TP_PIN (GPIO_PIN_12)

#define TP_H FOC_TP_PORT->ODR |= FOC_TP_PIN
#define TP_L FOC_TP_PORT->ODR &= ~FOC_TP_PIN

#define UPDATE_TP_PORT (GPIOB)
#define UPDATE_TP_PIN (GPIO_PIN_13)
#define UPDATE_H UPDATE_TP_PORT->ODR |= UPDATE_TP_PIN
#define UPDATE_L UPDATE_TP_PORT->ODR &= ~UPDATE_TP_PIN

void foc_tp_pin_set(int v)
{
    if (v)
        TP_H;
    else
    {
        TP_L;
    }
}
//for sample point and calc time visual
void foc_tp_pin_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = FOC_TP_PIN | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(FOC_TP_PORT, &GPIO_InitStruct);
}

void update_tp_pin_set(int v)
{
    if (v)
        UPDATE_H;
    else
    {
        UPDATE_L;
    }
}