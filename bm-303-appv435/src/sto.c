#include "sto.h"

#include "mc_math.h"
// #include "hw_math.h"

#include <stdint.h>
#include <math.h>
#include "motor_param.h"

sto_var_t sv;

#define Ts (0.0001f) //10k
// #define Ls (0.00235f)   //1khz measure LCR
#define Ls          PHASE_LS
#define TsDivLs (Ts / Ls)
#define TsDivLs_Q15 ((int16_t)(TsDivLs*32768))
// #define K 100 
// #define Rs (0.75f)
#define Rs          PHASE_RS
#define Rs_Q15 ((int16_t)(32768*Rs))
#define Ls_Q15 ((int16_t)77) //0.00235*32768
#define LPF_P1 (0.90f)  //old
#define LPF_P2 (1.0f - LPF_P1)
#define LPF_P1_Q15 ((int16_t)29491)
#define LPF_P2_Q15 ((int16_t)3277) //p2 = 1-p1
float K = 100.0f;
float a = 1e-4;
float lpf1 = 0.9f;
int32_t low_pass_filter_q15(int32_t *last_filtered, int32_t input)
{
#warning "implement me !!!"

    // static int16_t last_flitered;
    int32_t part1 = LPF_P1_Q15 * *last_filtered / 32768;
    int32_t part2 = LPF_P2_Q15 * input / 32768;
    *last_filtered = part1 + part2;

    return *last_filtered;
}

float low_pass_filter_float(float last_filtered, float input)
{
    float part1 = lpf1 * last_filtered;
    float part2 = (1 - lpf1) * input;

    float filtered = part1 + part2;

    return filtered;
}

static void inline _sto_update(int16_t ua,
                               int16_t ia,
                               int32_t *ia_est,
                               int32_t *ea_est)
{
    int32_t delta_ia = *ia_est - ia;
    int16_t sig_fa = sigmoid_q15(delta_ia);

    *ea_est = K * sig_fa;

    int32_t stuff_a1 = -Rs_Q15 * *ia_est / 32768;
    int32_t stuff_a2 = -K * sig_fa; //will it overflow???
    int32_t stuff_a = stuff_a1 + stuff_a2 + ua;

    *ia_est += stuff_a * TsDivLs_Q15 / 32768;
}

static void inline _sto_update_float(int16_t ua,
                                     int16_t ia,
                                     float *ia_est,
                                     float *ea_est)
{
    float ia_estf = *ia_est;
    float delta_ia = ia_estf - ia;
    float sig_fa = a * delta_ia;
    // float sig_fa = 0.5f * delta_ia;//(delta_ia);

    *ea_est = K * sig_fa;

    float stuff_a1 = -Rs * ia_estf;
    float stuff_a2 = -K * sig_fa;
    float stuff_a = stuff_a1 + stuff_a2 + ua;
     
    *ia_est += stuff_a * TsDivLs;
}

void sto_input_param(int16_t va,
                     int16_t vb,
                     int16_t ia,
                     int16_t ib)
{
    sv.valfa = va;
    sv.vbeta = vb;
    sv.ialfa = ia;
    sv.ibeta = ib;
}

void sto_update_ab(sto_var_t *psto)
{
    _sto_update(psto->valfa,
                psto->ialfa,
                &psto->ialfa_est,
                &psto->ealfa_est);

    _sto_update(psto->vbeta,
                psto->ibeta,
                &psto->ibeta_est,
                &psto->ebeta_est);
}

void sto_update_ab_float(sto_var_t *psto)
{
    _sto_update_float(psto->valfa,
                psto->ialfa,
                &psto->ialfa_estf,
                &psto->ealfa_estf);

    _sto_update_float(psto->vbeta,
                psto->ibeta,
                &psto->ibeta_estf,
                &psto->ebeta_estf);
}
// float sto_thetaf;
uint16_t sto_theta;
int sv_ovf_cnt;

uint16_t sto_update_10khz()
{
#if 0
    sto_update_ab(&sv);
    sv.ealfa_est = low_pass_filter_q15(&sv.ealfa_est_filtered, 
                                        sv.ealfa_est);

    sv.ebeta_est = low_pass_filter_q15(&sv.ebeta_est_filtered, 
                                        sv.ebeta_est);

    if (fabs(sv.ealfa_est) > 32767)
    {
        sv_ovf_cnt++;
    }

    if (fabs(sv.ebeta_est) > 32767)
    {
        sv_ovf_cnt++;
    }

    sv.theta_estf = atan2f(sv.ebeta_est / 32768.0f, 
                             sv.ealfa_est / 32768.0f);
    sv.theta_estf += sv.theta_estf < 0 ? 2*3.1416f : 0;
#else

    sto_update_ab_float(&sv);
    #if 0
        sv.ealfa_estf_filtered
            = low_pass_filter_float(sv.ealfa_estf_filtered, 
                                    sv.ealfa_estf);

        sv.ebeta_estf_filtered
            = low_pass_filter_float(sv.ebeta_estf_filtered, 
                                    sv.ebeta_estf);

        sv.theta_estf = atan2f(sv.ebeta_estf_filtered, 
                                sv.ealfa_estf_filtered);
    #elif 1
        sv.theta_estf = -atan2f(sv.ealfa_estf, sv.ebeta_estf);//
                                // sv.ealfa_estf);
    #else
        sv.theta_estf = -atan2f(sv.ebeta_estf,
                                sv.ealfa_estf);
    #endif

    sv.theta_estf += sv.theta_estf < 0 ? 2*3.1416f : 0;
#endif

    // sv.ealfa_estf = sv.ealfa_est / 32768.0f;
    // sv.ebeta_estf = sv.ebeta_est / 32768.0f;
    // sv.theta_estf = atan2f(-sv.ealfa_estf, sv.ebeta_estf);
    // sv.theta_estf = atan2f(-sv.ealfa_estf, sv.ebeta_estf);
    sv.sto_theta = sv.theta_estf * 10430;
    return sv.sto_theta;
}

//10k update call.
void sto_update(sto_var_t *psto)
{
    //q15 - q15 maybe overflowed. use 32bit q15
    int32_t delta_ialfa = psto->ialfa_est - psto->ialfa;
    int32_t delta_ibeta = psto->ibeta_est - psto->ibeta;
    int16_t sig_fa = sigmoid_q15(delta_ialfa);
    int16_t sig_fb = sigmoid_q15(delta_ibeta);

    psto->ealfa_est = K * sig_fa;
    psto->ebeta_est = K * sig_fb;

    int32_t stuff_a1 = -Rs_Q15 * psto->ialfa_est / 32768;
    int32_t stuff_a2 = -K * sig_fa;
    int32_t stuff_a = stuff_a1 + stuff_a2 + psto->valfa;

    psto->ialfa_est += psto->ialfa_est * TsDivLs_Q15 / 32768;
}