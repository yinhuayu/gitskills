#ifndef __STO_H
#define __STO_H

#include <stdint.h>

typedef struct _sto_var
{
    int16_t valfa;
    int16_t vbeta;
    int16_t ialfa;
    int16_t ibeta;

    int32_t ialfa_est;
    int32_t ibeta_est;

    int32_t ealfa_est;
    int32_t ebeta_est;

    int32_t ealfa_est_filtered;
    int32_t ebeta_est_filtered;

    float ealfa_estf_filtered;
    float ebeta_estf_filtered;

    float ialfa_estf;
    float ibeta_estf;

    float ealfa_estf;
    float ebeta_estf;

    int16_t theta_est;
    float theta_estf;
    uint16_t sto_theta;
} sto_var_t;



void sto_input_param(int16_t va,
                    int16_t vb,
                    int16_t ia,
                    int16_t ib);

void sto_update_ab(sto_var_t *psto);

uint16_t sto_update_10khz(void);


#endif
