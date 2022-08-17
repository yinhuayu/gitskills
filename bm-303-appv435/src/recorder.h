
#pragma once
#include <stdint.h>

#define ENABLE_THETA_RECORD

typedef enum {
    REC_IDLE = 0,
    REC_START,  //record should started at 0.
    REC_END,
    REC_PLAYING,
}rec_state_e;

typedef struct _rec_var
{
    /* data */
    rec_state_e rec_state;
    int record_cnt;     //each foc = 1cnt
    int record_step;    //
    uint16_t current_theta_play;
    uint8_t toggle;
}record_var_t;

void recorder_init();
void recorder_start_play();
void recorder_start_rec();
uint16_t recorder_fsm(uint16_t theta);
uint16_t recorder_play_poly_fit();
uint16_t recorder_play_poly_fit_fan();