
#include "gd32f30x.h"

#define FOC_RECORDER 0
#define FOC_ENC 1
#define FOC_HALL 0
#define POLE_PAIR (20)

#define CPU_FREQ_MHZ (120)
#define PWM_FREQ (20000u) // also = freq of foc.
#define PWM_CKTIM (CPU_FREQ_MHZ * 1000000u)
#define PWM_ARR (PWM_CKTIM / 2 / PWM_FREQ) //  = 3000
// #define SIX_STEP_ARR        PWM_ARR//(PWM_CKTIM / PWM_PSC / PWM_FREQ)
// 6 step close loop for current. also need sample at pwm open
#define FOC_FREQ PWM_FREQ
// #define CURRENT_CTRL_FREQ		(1000u)
#define SPEED_CTRL_FREQ (1000)
#define POSITION_CTRL_FREQ (1000)

#define htim_pwm htim1
#define TIM_PWM_INSTANCE TIMER0

#ifdef FOC_HALL
#define htim_hall htim4
#define HALL_TIMER TIM4
#define HALL_TIM_PSC CPU_FREQ_MHZ
#define HALL_CKTIM (CPU_FREQ_MHZ * 1000000u)
#endif

#ifdef FOC_ENC
#define htim_enc htim3
#define TIM_ENC_INSTANCE TIM3
// #define ENC_MECH_1CIRCLE     (2048)   //本末新版电机换512线 100圈均值约2046.88
// #define ENC_MECH_1CIRCLE     (1536)   //本末旧版电机384线 （绿色编码器线
#define ENC_MECH_1CIRCLE (2048) //本末旧版电机384线 （白色编码器线
#endif