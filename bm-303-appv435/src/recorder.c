
#include "recorder.h"

#include <stdint.h>

#include "drv_flash.h"
#include "log.h"

// #define FLASH_RECORD_ADDR   (0x08000000 + FLASH_PAGE_SIZE * 30)


record_var_t rv;
const int record_step_cnt = 10;     //10khz //1ms 1dot
const int record_step_max = RECORDER_FLASH_CNT;
// int16_t record_theta_buffer[1024];  //1ms 1dot, max 4s
int confirm_save = 0;
static float poly_x;
// uint16_t get_recorder_current_theta_play()
// {
//     return rv.current_theta_play;
// }
void recorder_start_play()
{
    rv.rec_state = REC_PLAYING;
    rv.record_step = 0;
    poly_x = 0;
}

void recorder_start_rec()
{
    rv.rec_state = REC_START;
    rv.record_cnt = 0;
    rv.record_step = 0;
}
//when recording, input = real theta.
//when playing, return expected theta.
uint16_t recorder_fsm(uint16_t theta)
{
    static uint16_t theta_play;
    switch (rv.rec_state) 
    {
        case REC_IDLE:
        {
            
        }
        break;

        case REC_START:
        {
            if (++rv.record_cnt >= record_step_cnt) {
                rv.record_cnt = 0;

                store_param.record_theta_tab[rv.record_step++] = theta;
            }
            
            if (rv.record_step == record_step_max-1) {
                rv.record_step = 0;
                rv.rec_state = REC_END;
                LOGI("record reach max, end of rec.");
            }
        }
        break;

        case REC_END:
        {
            if (confirm_save) {
                confirm_save = 0;
                save_theta_buffer_to_flash(&store_param);

                rv.record_step = 0;
                rv.rec_state = REC_IDLE;
            }
        }
        break;

        case REC_PLAYING:
            if (++rv.record_cnt >= record_step_cnt) {
                rv.record_cnt = 0;

                rv.toggle = !rv.toggle;
                theta_play = store_param.record_theta_tab[rv.record_step++];
            }
            
            if (rv.record_step == record_step_max-1) {
                rv.record_step = 0;
                rv.rec_state = REC_END;
                LOGI("play reach max, end of play.");
            }
        
        default:
        break;
    }
    return theta_play;
}

// Linear model Poly3:
//      f(x) = p1*x^3 + p2*x^2 + p3*x + p4
// Coefficients (with 95% confidence bounds):
//        p1 =    -0.04117  (-0.04262, -0.03972)
//        p2 =       24.36  (23.92, 24.8)
//        p3 =       -81.3  (-119.7, -42.91)
//        p4 =       47.61  (-845.6, 940.8)

// Goodness of fit:
//   SSE: 4.839e+08
//   R-square: 0.9999
//   Adjusted R-square: 0.9999
//   RMSE: 1571

uint16_t recorder_play_poly_fit()
{
    float outputf;
    uint16_t output;
    poly_x += 0.1f;
    float x = poly_x;
    // if (x < 120.f) {
        // const float p1 = 15.79;
        // const float p2 = 133;
        // const float p3 = 755.3;
        const float p1 = 13.655;
        const float p2 = 115;
        const float p3 = 1.1822e3;
        outputf = p1 * x * x + p2 * x + p3;
    // } 
    // else if (x < 220) {
    //     const float p1 = 14.83;
    //     const float p2 = 488.2;
    //     const float p3 = -2.749e4;
    //     outputf = p1 * x * x + p2 * x + p3;
    // }
    // else {
    //     const float p1 = -0.1612;
    //     const float p2 = 7333;
    //     const float p3 = -8.179e5;
    //     // outputf = p2 * x + p3;
    //     outputf = p1 * x * x + p2 * x + p3;
    // }
    
    output = outputf;
    return output % 0x10000;
}

uint16_t recorder_play_poly_fit_fan()
{
    float outputf;
    uint16_t output;
    poly_x += 0.1f;
    float x = poly_x;
    const float p1 = 2.8879;
    const float p2 = 23.1246;
    const float p3 = 1.1489e3;
    outputf = p1 * x * x + p2 * x + p3;
    output = outputf;
    return output % 0x10000;
}

//maybe should call in main thread. not in it.
//flash should erase before write.
void read_theta_buffer_from_flash(store_param_t *sp)
{
#if defined ENABLE_THETA_RECORD
    read_store_param(sp);
#endif
}

void save_theta_buffer_to_flash(store_param_t *sp)
{
#if defined ENABLE_THETA_RECORD

    sp->param_status |= PARAM_STATUS_THETA_TAB;
    write_store_param(sp);
#endif
}

void recorder_init()
{
    read_store_param(&store_param);
    LOGW("recorder read theta table buffer.");
}