#include "enc.h"

#include <stdint.h>
#include <math.h>

#include "foc_cfg.h"
// #include "motor_param.h"
#include "mc_math.h"

enc_var_t ev;

const float enc2uint16 = 1.0f * (0x10000) / ENC_MECH_1CIRCLE; // expand to 0x10000

void encoder_reg_cnt_register(enc_var_t *pev, uint32_t *cnt_addr)
{
    pev->p_enc_cnt = cnt_addr;
}

void encoder_reset_raw_cnt(enc_var_t *pev, uint16_t cnt)
{
    *pev->p_enc_cnt = cnt;
}

int32_t encoder_get_odmetry_cnt(enc_var_t *pev)
{
    return pev->raw_odmetry;
}

//获取机械角度 [0, 2pi] = [0, 65535]
uint16_t encoder_get_theta_mechanical(enc_var_t *pev)
{
    return pev->raw_mechanical;
}
uint16_t encoder_get_theta_electrical(enc_var_t *pev) 
{
    if (pev->p_enc_cnt)
    {
        #if 1
        pev->raw_cnt = *pev->p_enc_cnt;
        #else
        pev->raw_cnt = ENC_MECH_1CIRCLE - *pev->p_enc_cnt;
        #endif
    }
    pev->raw_delta = calc_theta_delta_raw(&pev->raw_cnt_last,
                                        pev->raw_cnt, ENC_MECH_1CIRCLE);
    pev->raw_cnt_last = pev->raw_cnt;
    pev->raw_odmetry += pev->raw_delta;
#if 1 //SWAP UVW or change to reverse enc dir.
    pev->raw_mechanical = (pev->raw_cnt) * enc2uint16;
#else
    pev->raw_mechanical = (ENC_MECH_1CIRCLE - pev->raw_cnt) * enc2uint16;
#endif
    pev->enc_theta = pev->raw_mechanical * POLE_PAIR;
    return pev->enc_theta;
}

// uint16_t encoder_get_theta_electrical(enc_var_t *pev)
// {
//     if (pev->p_enc_cnt)
//     {
// #if 1
//         if (pev->raw_offset >= 0)
//         {
//             pev->raw_cnt = *pev->p_enc_cnt + pev->raw_offset;
//             if (pev->raw_cnt >= ENC_MECH_1CIRCLE)
//             {
//                 pev->raw_cnt -= ENC_MECH_1CIRCLE;
//             }
//         }
//         else
//         {

//             if (*pev->p_enc_cnt + pev->raw_offset >= ENC_MECH_1CIRCLE)
//             {
//                 pev->raw_cnt = ENC_MECH_1CIRCLE - (0x100000000 - *pev->p_enc_cnt - pev->raw_offset);
//             }
//             else
//             {
//                 pev->raw_cnt = *pev->p_enc_cnt + pev->raw_offset;
//             }
//         }

// #else
//         pev->raw_cnt = ENC_MECH_1CIRCLE - *pev->p_enc_cnt;
// #endif
//     }
//     pev->raw_delta = calc_theta_delta_raw(&pev->raw_cnt_last,
//                                           pev->raw_cnt, ENC_MECH_1CIRCLE);


                                         
//     pev->raw_cnt_last = pev->raw_cnt;
//     pev->raw_odmetry += pev->raw_delta;
// #if 1 // SWAP UVW or change to reverse enc dir.
//     pev->raw_mechanical = (pev->raw_cnt) * enc2uint16;
// #else
//     pev->raw_mechanical = (ENC_MECH_1CIRCLE - pev->raw_cnt) * enc2uint16;
// #endif
//     pev->enc_theta = pev->raw_mechanical * POLE_PAIR;
//     return pev->enc_theta;
// }


// const float ratio2int16 = 65536.0f / ((float)ENC_MECH_1CIRCLE / POLE_PAIR);

//获取机械角度 [0, 2pi] = [0, 65535]
uint16_t get_theta_mechanical()
{
    return ev.raw_mechanical;
}

int32_t get_odmetry_cnt()
{
    return ev.raw_odmetry;
}

void task_update_enc(uint32_t time)
{
    // _update_enc(&ev);
}

// this function will called when align to angle 0.
void enc_zero_trigger()
{
    // if (!is_enc_ready) {
    //   TIM3->CNT = 0;
    //   is_enc_ready = 1;
    // }
    *ev.p_enc_cnt = 0; //(ENC_MECH_1CIRCLE);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
#warning "FIXME when multi pin event triggers."
    // enc_zero_trigger();
}
