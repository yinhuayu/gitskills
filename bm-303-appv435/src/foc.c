#include "foc.h"

#include "foc_cfg.h"
#include "log.h"
// #include "nr_micro_shell.h"
// #include "startup_tab.h"
#include "mc_math.h"
#include "pid.h"
#include "enc.h"
#if FOC_RECORDER
#include "recorder.h"
#endif

// #define __DEBUG
// #define __FORCE_ROTATE_SVPWM

// #define ALIGN_VD        (600)   //use vd , theta = 0
#define ALIGN_VQ (-1500)    // use vq , theta = -90.
#define ALIGN_THETA (49152) //
#define STARTUP_VQ (500)
#define IQ_REF (6000)
// #define VQ_FROM_IQ(x)   ()  //(vdd/2/(Rshunt*gain))
//#define MomentIertia_para (1000)   // Moment of inertia parameter
int32_t MomentIertia_para = 70;
const uint32_t align_continues_time = 600; // ms
const uint32_t switch_to_sensorless_time = 3000;
const uint32_t speed_loop_stable_time = 2000;
uint32_t speed_loop_start_timestamp;

#if 0
// const int16_t angle_est_limit = 65536 / 6;

void foc_start_align_then_sensor_current()
{
    fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_CURRENT;
    sl.align_start_timestamp = HAL_GetTick();
}

void foc_start_align_then_sensor_speed()
{
    fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_SPEED;
    sl.align_start_timestamp = HAL_GetTick();
}

void foc_start_align_then_sensorless()
{
    fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSORLESS;
    sl.align_start_timestamp = HAL_GetTick();
}

void foc_start_cali()
{
    fv.foc_cali_status = FOC_CALI_BUSY;
    foc_start_align_then_sensor();
}

void foc_stop()
{
    fv.iq_ref = 0;
    fv.id_ref = 0;
    fv.vd = 0;
    fv.vq = 0;
    LOGW("foc stop... ");
    fv.fsm_state = FSM_STATE_IDLE;
}

void foc_can_ctrl(can_proto_t *pcc)
{
    if (pcc->cmd != CMD_SENSORLESS_ADJ && !pcc->enable) {
        foc_stop(0,0);
        return;
    }
    switch (pcc->cmd) {
        case CMD_ROTOR_ALIGN:
            LOGW("rotor align and hold");
            fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_CURRENT;
        break;

        case CMD_OPENLOOP_VQ:
            fv.vd = 0;
            fv.vq = pcc->param;
            fv.fsm_state = FSM_STATE_SENSOR_OPENLOOP;
            LOGW("start sensor foc openloop.. ");
        break;

        case CMD_CURRENT_IQ:
            fv.iq_ref = pcc->param;
            fv.id_ref = 0;
            fv.fsm_state = FSM_STATE_SENSOR_CURRENT;
            // foc_start_align_then_sensor_current();
            LOGW("start sensor foc.. current loop");
        break;

        case CMD_SPEED_CTRL:
            fv.speed_ref = pcc->param;// MAP(pcc->param, 1200, 4095, 0, 3000);
            foc_start_align_then_sensor_speed();
            LOGW("start sensor foc.. speed loop");
        break;

        case CMD_SENSORLESS:
            foc_start_align_then_sensorless();
            LOGW("start foc.. sensorless align run");
        break;

        case CMD_SENSORLESS_ADJ:
            fv.speed_ref = pcc->param;
            LOGW("foc speed ref set %d", pcc->param);
        break;
    }
}
// void (*uart_rx_idle_handle_func)(uint8_t* buf, uint16_t len);
void foc_uart_ctrl(uint8_t* buf, uint16_t len )
{
    can_proto_t *pcc = (can_proto_t*)buf;
    foc_can_ctrl(pcc);
}

//write fv.foc_cali_status = 1 start cali.
void foc_calibrate_hook(foc_var_int_t *pfv, int16_t theta_fb)
{
    static uint32_t cali_ok_cnt;
    const uint32_t theta_variant = 2000;
    const uint32_t theta_check_cnt = 20000; //1s
    static uint32_t start_timestamp;
    static int32_t offset_sum = 0;
    int32_t offset_max, offset_min;
    if (FOC_CALI_BUSY == pfv->foc_cali_status)
    {
        if ( 0 == start_timestamp ) {
            start_timestamp = HAL_GetTick();
        }
        
        //add vd so rotor will stuck in 0 deg.
        pfv->vq = 0;
        pfv->vd = pfv->pwm_full / 4; //25% of full
        pfv->theta = 0;              //freeze theta.

        //we assume 500ms rotor will back to zero position.
        if (0 != start_timestamp &&
            HAL_GetTick() - start_timestamp > 500) {
            if (offset_sum == 0) {
                offset_min = theta_fb;
                offset_max = theta_fb;
            }
            
            if (theta_fb > offset_max) {
                offset_max = theta_fb;
            }
            if (theta_fb < offset_min) {
                offset_min = theta_fb;
            }

            offset_sum += theta_fb;

            if (cali_ok_cnt++ > theta_check_cnt)
            {
                cali_ok_cnt = 0;
                offset_sum = 0;
                start_timestamp = 0;
                pfv->theta_offset = offset_sum / theta_check_cnt;
                pfv->foc_cali_status = FOC_CALI_PASS; //calied ok.
                pfv->theta_offset = theta_fb;
                LOGI("motor theta 0 cali ofs:%d", pfv->theta_offset);
                LOGI("motor theta 0 cali max:%d", offset_max);
                LOGI("motor theta 0 cali min:%d", offset_min);
                //if variant too big. cali = fail.
            }
        }
    }
}
#endif

void foc_register_pwm_output(foc_var_int_t *pfv, uint32_t ch[])
{
    if (ch)
    {
        pfv->pwm[0] = ch[0]; // U
        pfv->pwm[1] = ch[1]; // V
        pfv->pwm[2] = ch[2]; // W
    }
    for (int i = 0; i < 3; ++i)
    {
        LOGI("register pwm ch addr=%x ", pfv->pwm[i]);
    }
    pfv->pwm_full = PWM_ARR;
}

// void foc_register_get_theta_func(uint16_t (*func)(), void(*cali_func)(void) )
// {
//     fv.get_sensor_theta = func;
//     fv.calibrate_sensor_theta_offset = cali_func;
// }

void foc_register_func_set(foc_var_int_t *pfv, foc_func_set_t *func_set)
{
    pfv->get_sensor_theta = func_set->get_sensor_theta_electrical;
    pfv->get_sensor_theta_mechanical = func_set->get_sensor_theta_mechanical;
    pfv->get_sensor_odmetry = func_set->get_sensor_odmetry;
    pfv->reset_sensor_zero = func_set->reset_sensor_zero;
    pfv->calibrate_sensor_theta_offset = func_set->calibrate_sensor_theta_offset;
    // pfv->tp_pin_set = func_set->tp_pin_set;
}

// TODO: implement me.
void foc_register_iab_current_sample(foc_var_int_t *pfv,
                                     int16_t *pia,
                                     int16_t *pib)
{
}

// this function should move to outside.
//  void foc_algorithm_init()
//  {
//      #if FOC_RECORDER
//          recorder_init();
//      #endif

//     fv.theta = ALIGN_THETA;
//     fv.speed_ref = 200;
//     fv.iq_ref = IQ_REF;
//     // fv.fsm_state = FSM_STATE_ROTOR_ALIGN;
//     // fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_CURRENT;
//     fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_SPEED;
//     // fv.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION;
//     // fv.fsm_state = FSM_STATE_SENSOR_CURRENT;
//     // fv.fsm_state = FSM_STATE_ROTOR_ALIGN_VQ;
//     fv.sl.align_start_timestamp = HAL_GetTick();
//     fv.out_en = 1;

//     lpf_1rd_init(&fv.lpf1_speed, 0.1, 0);
// }

void foc_algorithm_init2(foc_var_int_t *pfv)
{

    pfv->theta = ALIGN_THETA;
    pfv->speed_ref = 1000;
    pfv->iq_ref = IQ_REF;
    // pfv->fsm_state = FSM_STATE_ROTOR_ALIGN;
    // pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_CURRENT;
    // pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_SPEED;
    pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION;
    // pfv->fsm_state = FSM_STATE_SENSOR_CURRENT;
    // pfv->fsm_state = FSM_STATE_ROTOR_ALIGN_VQ;
    pfv->sl.align_start_timestamp = HAL_GetTick();
    pfv->out_en = 1;

    lpf_1rd_init(&pfv->lpf1_speed, 0.1, 0);
    lpf_1rd_init(&pfv->lpf1_MomentI, 0.1, 0);
}

static volatile int fuck_bug;
static inline void foc_speed_calc(foc_var_int_t *pfv)
{
    // static uint16_t last_theta;
    // static int freq1k_cnt;
    // static int16_t speed_buffer[16];
    // static slide_filter_t sf = {
    //     .len = 16,
    //     .pfilter_buffer = speed_buffer,
    // };
    // static int16_t speed_raw;
    // static int16_t speed_raw10;

    // 500hz 2ms
    if (++pfv->speed_calc_div_cnt == (PWM_FREQ / SPEED_CTRL_FREQ))
    {
        pfv->speed_calc_div_cnt = 0;
        pfv->theta_mechanical = pfv->get_sensor_theta_mechanical ? pfv->get_sensor_theta_mechanical() : 0;
        pfv->theta_delta_mechanical =
            calc_theta_delta_raw(&pfv->last_theta_mechanical,
                                 pfv->theta_mechanical, 0x10000);
        pfv->speed_radps_mechanical = pfv->theta_delta_mechanical * SPEED_CTRL_FREQ;
        pfv->speed_rpm_mechanical = pfv->speed_radps_mechanical * 60 / 65536;
        // omega = delta theta / dt. 增大了10435
        pfv->speed_rpm10 = pfv->speed_rpm_mechanical * 10;
        if (pfv->speed_rpm10 < 200)
        {
            fuck_bug++;
        }
        pfv->last_theta_mechanical = pfv->theta_mechanical;
        // pfv->theta_delta_mechanical * SPEED_CTRL_FREQ * 6000 / 65535;
        // pfv->theta_delta = calc_theta_delta_raw(&pfv->last_theta, pfv->theta, 0xffff);
        // speed_raw100 = (int32_t)pfv->theta_delta * SPEED_CTRL_FREQ * 6000 / POLE_PAIR / 65536;//1circle = 0xffff, dt = 1ms
        pfv->speed_rpm =
            lpf_1rd_calc(&pfv->lpf1_speed, pfv->speed_rpm10);

// fv.speed_rpm100 = slide_filter_int16(
//     speed_raw100,
//     &sf.idx,
//     sf.pfilter_buffer,
//     sf.len);
#warning "FIXME???????"
    }
}
static inline void foc_inertia_calc(foc_var_int_t *pfv)
{

    pfv->inertia_compensation = MomentIertia_para * lpf_1rd_calc(&pfv->lpf1_MomentI, pfv->pid_speed.dout);

    if (pfv->inertia_compensation > 600)
    {
        pfv->inertia_compensation = 600;
    }
    if (pfv->inertia_compensation < -600)
    {
        pfv->inertia_compensation = -600;
    }
}
static inline void foc_speed_ctrl(foc_var_int_t *pfv)
{
    if (++pfv->speed_ctrl_div_cnt == (PWM_FREQ / SPEED_CTRL_FREQ))
    {
        pfv->speed_ctrl_div_cnt = 0;

        if (FSM_STATE_SENSOR_SPEED == pfv->fsm_state ||
            FSM_STATE_SENSOR_POSITION == pfv->fsm_state ||
            FSM_STATE_SENSORLESS_CLOSELOOP == pfv->fsm_state)
        {
            pfv->iq_ref =
                pid_calculate_q15(&pfv->pid_speed,
                                  pfv->speed_rpm,
                                  pfv->speed_ref);
        }
    }
}

static inline void foc_pos_ctrl(foc_var_int_t *pfv)
{
    if (++pfv->pos_ctrl_div_cnt == (PWM_FREQ / POSITION_CTRL_FREQ))
    {
        pfv->pos_ctrl_div_cnt = 0;

        if (FSM_STATE_SENSOR_POSITION == pfv->fsm_state)
        {
            pfv->speed_ref =
                pid_calculate_q15(&pfv->pid_pos,
                                  pfv->pos_fdbk,
                                  pfv->pos_ref);
        }
    }
}
int16_t vq_tfyl = 0;
void foc_fsm_loop(foc_var_int_t *pfv)
{
    switch (pfv->fsm_state)
    {
    case FSM_STATE_IDLE:
    {
    }
    break;

    case FSM_STATE_ROTOR_ALIGN:
    {
        pfv->iq_ref = IQ_REF;
        pfv->id_ref = 0;
        // pfv->theta = ALIGN_THETA;
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_VQ:
    {
        // pfv->vq = STARTUP_VQ;
        // pfv->vd = 0;
        // pfv->theta = ALIGN_THETA;

        pfv->theta = pfv->get_sensor_theta ? pfv->get_sensor_theta() : 0;
        pfv->pos_fdbk = pfv->get_sensor_odmetry ? pfv->get_sensor_odmetry() : 0;

        if (HAL_GetTick() - pfv->sl.align_start_timestamp > align_continues_time)
        {
            pfv->vq = vq_tfyl;
            pfv->sl.try_startup_timestamp = HAL_GetTick();
#if FOC_RECORDER
            recorder_start_play();
#endif
            // pfv->fsm_state = FSM_STATE_SENSOR_OPENLOOP;
            // LOGW("goto sensor openloop");
        }
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_SENSORLESS:
    {
        float ramp;
        const int ramp_end = 300;
        if (HAL_GetTick() - pfv->sl.align_start_timestamp <= ramp_end)
        {
            ramp = (HAL_GetTick() - pfv->sl.align_start_timestamp) * 1.0f / (ramp_end);
        }
        else
        {
            ramp = 1;
        }
        pfv->iq_ref = IQ_REF * ramp;
        pfv->id_ref = 0;
        pfv->theta = ALIGN_THETA;
        if (HAL_GetTick() - pfv->sl.align_start_timestamp > align_continues_time)
        {
            pfv->sl.try_startup_timestamp = HAL_GetTick();
#if FOC_RECORDER
            recorder_start_play();
#endif

            pfv->fsm_state = FSM_STATE_TRY_STARTUP;
            LOGW("goto try startup");
        }
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_SENSOR_CURRENT:
    {
        pfv->iq_ref = IQ_REF;
        pfv->id_ref = 0;
        pfv->theta = ALIGN_THETA;

        if (HAL_GetTick() - pfv->sl.align_start_timestamp > align_continues_time)
        {
            if (pfv->reset_sensor_zero)
                pfv->reset_sensor_zero();
            pfv->fsm_state = FSM_STATE_SENSOR_CURRENT;
            pfv->sl.start_speed_timestamp = HAL_GetTick();
            // recorder_start_rec();
        }
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_SENSOR_SPEED:
    {
        pfv->iq_ref = IQ_REF;
        pfv->id_ref = 0;
        pfv->theta = ALIGN_THETA;

        if (HAL_GetTick() - pfv->sl.align_start_timestamp > align_continues_time)
        {
            if (pfv->reset_sensor_zero)
                pfv->reset_sensor_zero();
            pfv->fsm_state = FSM_STATE_SENSOR_SPEED;
            pfv->sl.start_speed_timestamp = HAL_GetTick();

            // recorder_start_rec();
        }
    }
    break;

    case FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION:
    {
        pfv->iq_ref = IQ_REF;
        pfv->id_ref = 0;
        pfv->theta = ALIGN_THETA;

        if (HAL_GetTick() - pfv->sl.align_start_timestamp > align_continues_time)
        {
            if (pfv->reset_sensor_zero)
                pfv->reset_sensor_zero();
            pfv->fsm_state = FSM_STATE_SENSOR_POSITION;
            pfv->sl.start_speed_timestamp = HAL_GetTick();
            speed_loop_start_timestamp = HAL_GetTick();
        }
    }
    break;

    case FSM_STATE_TRY_STARTUP:
    {
        static uint16_t theta_fit;
// start theta look up table.
#if 1
#if FOC_RECORDER
        pfv->theta = recorder_fsm(0); // playing
#endif

#else
        // recorder_fsm(0);    //playing
        theta_fit = recorder_play_poly_fit_fan(); // poly fit play
        pfv->theta = theta_fit;
        theta_fit++;
#endif
        if (HAL_GetTick() - pfv->sl.try_startup_timestamp > 4000)
        {
            pfv->fsm_state = FSM_STATE_SENSORLESS_CLOSELOOP;
            // pfv->fsm_state = FSM_STATE_SENSORLESS_OPENLOOP;
            LOGW("start sensorless");
        }
    }
    break;

    case FSM_STATE_SENSORLESS_OPENLOOP:
    {
        pfv->vq = STARTUP_VQ;
        pfv->vd = 0;
        pfv->theta = pfv->sto_theta; // sto_angle_err;
    }
    break;

    case FSM_STATE_SENSORLESS_CLOSELOOP:
    {
        // now we use sto theta => foc.
        // 17ms , 10khz = 170 period from 0 ~ 32768
        pfv->theta = pfv->sto_theta;
        foc_speed_ctrl(pfv);
        // if (HAL_GetTick() - sl.try_startup_timestamp > 1200) {
        //     if (pfv->speed_rpm < 700 || pfv->speed_rpm > 1200) {
        //         // start fail. reset.
        //         // HAL_NVIC_SystemReset();
        //     }
        // }
    }
    break;

    case FSM_STATE_SENSOR_OPENLOOP:
    {
        pfv->theta = pfv->get_sensor_theta ? pfv->get_sensor_theta() : 0;
        // pfv->theta = get_electical_angle_hall();
    }
    break;

    case FSM_STATE_SENSOR_CURRENT:
    {
        pfv->theta = pfv->get_sensor_theta ? pfv->get_sensor_theta() : 0;
// pfv->theta = get_electical_angle_hall();
#if FOC_RECORDER
        recorder_fsm(pfv->theta); // playing
#endif
    }
    break;

    case FSM_STATE_SENSOR_SPEED:
    {
        pfv->theta = pfv->get_sensor_theta ? pfv->get_sensor_theta() : 0;
        // pfv->theta = get_electical_angle_hall();
        foc_speed_ctrl(pfv);
#if FOC_RECORDER
        recorder_fsm(pfv->theta); // playing
#endif
    }
    break;

    case FSM_STATE_SENSOR_POSITION:
    {
        //位置环需要等速度环稳定后再执行。
        pfv->theta = pfv->get_sensor_theta ? pfv->get_sensor_theta() : 0;
        pfv->pos_fdbk = pfv->get_sensor_odmetry ? pfv->get_sensor_odmetry() : 0;
        // if (HAL_GetTick() - speed_loop_start_timestamp > speed_loop_stable_time)

        foc_inertia_calc(pfv);

        foc_pos_ctrl(pfv);

        foc_speed_ctrl(pfv);
        // pid_speed.max_out = IQ_REF;//adjust torque limit.
    }
    break;

    default:
        break;
    }
}
#if 0
const uint32_t start_run_ts = 0;
const uint32_t run_stop_ts = 20000;
const uint32_t reset_ts = 30000;
const int32_t speed_low = 900;
const int32_t speed_high = 1300;
int16_t speed1;
int16_t speed2;
uint32_t test_tick;
uint16_t test_ok;
uint16_t test_fail;

uint8_t is_in_range(int16_t x, int16_t x1, int16_t x2)
{
    return x >= x1 && x <= x2;
}
void foc_test_fsm()
{
    if (test_tick == start_run_ts) {
        foc_start_align_then_sensorless();
    }

    if (test_tick == (run_stop_ts-2)) {
        speed1 = fv.speed_rpm;
    }
    if (test_tick == (run_stop_ts-1)) {
        speed2 = fv.speed_rpm;
        if (is_in_range(speed1, speed_low, speed_high)
        && is_in_range(speed2, speed_low, speed_high) )
        {
            LOGI("nice. test ok. %d, fail:%d", test_ok++, test_fail);
        }
        else {
            LOGE("fuck. test fail. %d, speed=%d", test_fail++, speed2);
        }
    }

    if (test_tick == run_stop_ts) {
        foc_stop();
        fv.fsm_state = FSM_STATE_IDLE;
    }

    if (++test_tick >= reset_ts) {
        test_tick = 0;
    }
}
int adcp = PWM_ARR - 1;
#endif
// Vd为+时Id+， ia+ 则ok. 电机中点往外为ia正方向
void foc_compute(foc_var_int_t *pfv, int16_t _ia, int16_t _ib)
{
    // if (pfv->tp_pin_set) pfv->tp_pin_set(1);

    foc_fsm_loop(pfv);
    foc_speed_calc(pfv);

    pfv->ia = _ia;
    pfv->ib = _ib;

    // sto_input_param(pfv->valfa, pfv->vbeta, pfv->ialfa, pfv->ibeta);
    // pfv->sto_theta = sto_update_10khz();

    clarke_q15(pfv->ia, pfv->ib, &pfv->ialfa, &pfv->ibeta);

    // when θ = 0, id = ialfa, iq = ibeta.
    park_q15(pfv->ialfa, pfv->ibeta, pfv->theta, &pfv->id, &pfv->iq);

    if (pfv->fsm_state > FSM_STATE_IDLE &&
        FSM_STATE_ROTOR_ALIGN_VQ != pfv->fsm_state &&
        FSM_STATE_SENSOR_OPENLOOP != pfv->fsm_state &&
        FSM_STATE_SENSORLESS_OPENLOOP != pfv->fsm_state)
    // if (  FSM_STATE_MAG_SENSOR_OPENLOOP != pfv->fsm_state )
    {
        // need to implement pid regulatator
        pfv->vd = pid_calculate_q15(&pfv->pid_id, pfv->id, pfv->id_ref);
        pfv->vq = pid_calculate_q15(&pfv->pid_iq, pfv->iq, pfv->iq_ref);
    }
    pfv->vd = Value_Limit(pfv->vd, -5900, 5900); //取值范围-6000~6000,不能超,ming
    pfv->vq = Value_Limit(pfv->vq, -5900, 5900);
    if (FSM_STATE_SENSOR_OPENLOOP == pfv->fsm_state)
    {
        pfv->vd = 0;
        pfv->vq = 0;
    }
    // foc_calibrate_hook(&fv, get_electrical_angle_raw());

    /* vq,vq => valfa vbeta IPARK*/
    inverse_park_q15(pfv->vq, pfv->vd, pfv->theta, &pfv->valfa, &pfv->vbeta);

    svpwm_q15(pfv->valfa, pfv->vbeta, pfv->tabc);
    if (pfv->out_en)
    {
        // ta tb tc range[-1, 1] = [-ARR/2, ARR/2]
        *pfv->pwm[0] = PWM_ARR / 2 + pfv->tabc[0] / 4;
        *pfv->pwm[1] = PWM_ARR / 2 + pfv->tabc[1] / 4;
        *pfv->pwm[2] = PWM_ARR / 2 + pfv->tabc[2] / 4;
// ch4 = max(ch1,ch2,ch3) + adj;
#if 1
        if (pfv->pwm[3])
        {
            *pfv->pwm[3] = PWM_ARR - 10;
        }
#else
        max_of_3(*pfv->pwm[0], *pfv->pwm[1], *pfv->pwm[2]) + 1500;
#endif
    }
    // else
    // {
    //     *pfv->pwm[0] = 0;
    //     *pfv->pwm[1] = 0;
    //     *pfv->pwm[2] = 0;
    // }

    // if (pfv->tp_pin_set) pfv->tp_pin_set(0);
}

// //hall tim overflow.
// void htim_update_callback_foc(void *htim)
// {
//     if (&htim_pwm == htim)
//     {
//         // TP_H;
//         fv.tp_pin_set(1);
//         adc_raw2ma_calc_regular();
//         foc_compute(ma[0], ma[1]);
//         fv.tp_pin_set(0);
//         // TP_L;
//     }
// }