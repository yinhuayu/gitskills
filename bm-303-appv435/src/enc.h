#include <stdint.h>

typedef struct _enc_var
{
    uint32_t *p_enc_cnt; //pointer to reg.
    uint16_t raw_cnt;
    uint16_t raw_cnt_last;
    int32_t raw_delta;
    int16_t raw_offset;
    uint16_t raw_mechanical;
    uint16_t enc_theta;
    int32_t raw_odmetry; //累计里程计
} enc_var_t;

// void encoder_reg_cnt_register(uint32_t *cnt_addr);
void task_update_enc(uint32_t time);
void enc_zero_trigger();
int32_t get_odmetry_cnt();
uint16_t get_theta_mechanical();



void encoder_reg_cnt_register(enc_var_t *pev, uint32_t *cnt_addr);
void encoder_reset_raw_cnt(enc_var_t *pev, uint16_t cnt);
int32_t encoder_get_odmetry_cnt(enc_var_t *pev);
uint16_t encoder_get_theta_electrical(enc_var_t *pev);