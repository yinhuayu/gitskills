#pragma once
#include <stdint.h>
#include <math.h>
#include "mc_math.h"
#include "pid.h"

enum
{
    FOC_CALI_IDLE = 0,
    FOC_CALI_BUSY,
    FOC_CALI_PASS,
    FOC_CALI_FAIL,
};

// uint32_t status.       ctrl mode | with_sensor | |
// ctrl mode : idle / openloop / current / speed
// sensor type : hall / mag / linear-hall / enc-abz / unknown / sensorless
enum
{
    FOC_MODE_IDLE = 0,
    // FOC_MODE_ALIGN_HOLD,
    // FOC_MODE_STARTUP,
    FOC_MODE_OPENLOOP,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
};

enum
{
    FOC_SENSOR_TYPE_HALL = 0,
    FOC_SENSOR_TYPE_MAGNETIC,
    FOC_SENSOR_TYPE_LINEAR_HALL,
    FOC_SENSOR_TYPE_ENCORDER,
    FOC_SENSOR_TYPE_STO_SENSORLESS,
    FOC_SENSOR_TYPE_UNDEFINED,
};

enum
{
    FSM_STATE_IDLE = 0, // motor cutoff output
    // FSM_STATE_ROTOR_ALIGN_CALI, //align and calib.
    FSM_STATE_ROTOR_ALIGN, // align and hold. no auto jump.
    FSM_STATE_ROTOR_ALIGN_VQ,
    FSM_STATE_ROTOR_ALIGN_SENSOR_CURRENT,  // align and start sensor run.
    FSM_STATE_ROTOR_ALIGN_SENSOR_SPEED,    // align and start sensor run.
    FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION, // align and start sensor run.
    FSM_STATE_ROTOR_ALIGN_SENSORLESS,      // align and sensorless.
    FSM_STATE_ROTOR_ALIGN2MAG,             // align and goto mag.
    FSM_STATE_ROTOR_ALIGN2HALLXY,          // align and goto hallxy foc.
    FSM_STATE_TRY_STARTUP,                 // openloop
    FSM_STATE_SENSORLESS_OPENLOOP,         // vd const openloop
    FSM_STATE_SENSORLESS_CLOSELOOP,        // speed ctrl
    FSM_STATE_HALLXY_OPENLOOP,
    FSM_STATE_HALLXY_CURRENT,
    FSM_STATE_HALLXY_SPEED,
    FSM_STATE_SENSOR_OPENLOOP,
    FSM_STATE_SENSOR_CURRENT,
    FSM_STATE_SENSOR_SPEED,
    FSM_STATE_SENSOR_POSITION,
};

typedef struct _foc_func_set
{
    uint16_t (*get_sensor_theta_electrical)(); //获取电角度
    uint16_t (*get_sensor_theta_mechanical)(); //获取机械角度
    int32_t (*get_sensor_odmetry)();           //传感器里程计
    void (*reset_sensor_zero)();               //重设传感器角度，广编归零
    void (*calibrate_sensor_theta_offset)();   //校准记录传感器0位
    void (*tp_pin_set)(int v);
} foc_func_set_t;

typedef struct _sensorless_var
{
    /* data */
    uint16_t sto_angle;
    uint16_t sto_last_angle;
    int32_t sto_delta;
    int16_t sto_speed;
    int16_t sto_angle_err;
    int16_t sto_speed_rpm;
    int16_t sto_speed_fil;

    uint32_t align_start_timestamp;
    uint32_t try_startup_timestamp;
    uint32_t start_speed_timestamp;
} sensorless_var_t;

typedef struct _foc_var_int_t
{

    int16_t ia;
    int16_t ib;
    int16_t ic;

    int16_t ialfa;
    int16_t ibeta;

    int16_t id;
    int16_t iq;

    int16_t id_ref;
    int16_t iq_ref;

    int16_t vd; //=k*id
    int16_t vq;

    int16_t valfa;
    int16_t vbeta;

    uint16_t theta; // electical angle.
    uint16_t last_theta;
    uint16_t theta_mechanical; // mechanical angle
    uint16_t last_theta_mechanical;
    uint16_t sto_theta;
    uint16_t last_sto_theta;
    // int16_t sin_theta;
    // int16_t cos_theta;

    int16_t tabc[3]; // each channel switch time
    uint32_t *pwm[3];
    uint32_t pwm_full;

    uint8_t sector; // space vector [1~6]

    int16_t theta_offset;
    int16_t out_en;
    uint8_t foc_cali_status; // 0=idle. 1=calibrating. 2=ok.3=fail
    uint8_t fsm_state;

    // speed ctrl.
    int32_t theta_delta;
    int32_t theta_delta_mechanical;
    int32_t speed_radps_mechanical; // omega, rad/s
    int32_t speed_rpm;
    int32_t speed_rpm10; //低速电机使用 max +-327 rpm
    int32_t speed_rpm_mechanical;
    int32_t speed_ref;
    // int32_t speed_ref100;
    lpf1_t lpf1_speed;
    // int16_t speed_pid_out;
     lpf1_t   lpf1_MomentI;
    // position ctrl
    int32_t pos_fdbk;
    int32_t pos_ref;
    //Moment of Inertia caculate
    int32_t inertia_compensation;

    uint8_t ctrl_mode;
    uint8_t sensor_type;

    uint32_t align_start_timestamp;

    void (*tp_pin_set)(int v);
    uint16_t (*get_sensor_theta)();
    uint16_t (*get_sensor_theta_mechanical)();
    int32_t (*get_sensor_odmetry)();
    void (*reset_sensor_zero)();
    void (*calibrate_sensor_theta_offset)();

    // motor param.
    uint8_t pole_pair;

    pid_q15_t pid_id;
    pid_q15_t pid_iq;
    pid_q15_t pid_speed;
    pid_q15_t pid_pos;

    uint32_t speed_calc_div_cnt;
    uint32_t speed_ctrl_div_cnt; //用于调节速度环相对电流环的控制频率
    uint32_t pos_ctrl_div_cnt;

    sensorless_var_t sl;
} foc_var_int_t;

extern foc_var_int_t fv_M1;
extern foc_var_int_t fv_M2;

typedef struct
{
    uint32_t align_start_timestamp;

} foc_sensorless_var_t;

enum
{
    CMD_ROTOR_ALIGN = 0,
    CMD_OPENLOOP_VQ,
    CMD_CURRENT_IQ,
    CMD_SPEED_CTRL,
    CMD_SENSORLESS,
    CMD_SENSORLESS_ADJ,
};

typedef struct can_protocol
{
    uint8_t cmd;
    uint8_t enable;
    uint16_t param;
} can_proto_t;

void foc_can_ctrl(can_proto_t *pcc);

// void pmsm_foc_pwm_init(void);
// void foc_register_sensor_func_set(foc_var_int_t *pfv, foc_sensor_func_set_t *func_set);
void foc_register_func_set(foc_var_int_t *pfv, foc_func_set_t *func_set);
void foc_compute(foc_var_int_t *pfv, int16_t _ia, int16_t _ib);
// void foc_compute(int16_t _ia, int16_t _ib);
void htim_update_callback_foc(void *htim);
void foc_register_pwm_output(foc_var_int_t *pfv, uint32_t ch[]);
void foc_register_tp_pin_func(void (*tp_func)(int));
void foc_algorithm_init();
void foc_algorithm_init2(foc_var_int_t *pfv);
int32_t calc_magnetic_encoder_speed_raw(uint16_t *last, uint16_t cur);

// void foc_mag_openloop(char argc, char* argv);
// void sw2closeloop_function(char argc, char *argv);
// void foc_sensorless_start(char argc, char *argv);
// void foc_align_rotor(char argc, char* argv);
// void foc_start_mag(char argc, char* argv);
// void foc_stop(char argc, char* argv);
// void foc_align_rotor_add(char argc, char* argv);
// void foc_align_rotor_calib_mag(char argc, char* argv);
// void foc_start_enc(char argc, char* argv);
// void foc_start_enc_current(char argc, char* argv);
// void foc_start_enc_speed(char argc, char* argv);
// void foc_speed_inc(char argc, char* argv);
// void foc_speed_dec(char argc, char* argv);
// void foc_record_to_flash(char argc, char* argv);

void foc_button_ctrl();
void foc_uart_ctrl(uint8_t *buf, uint16_t len);

void foc_start_cali();
void foc_start_align_then_sensor();
void foc_start_align_then_sensorless();
void foc_start_cali();
void foc_stop();