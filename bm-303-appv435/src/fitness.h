#pragma once

#include <stdint.h>

// typedef enum {
//     FIT_IDLE = 0,
//     FIT_INIT_PULL_ROPE,  //启动收回绳子, 速度模式, 监控力矩iq
//     FIT_MODE_STANDARD,
//     FIT_MODE_CENTER,
//     FIT_MODE_SPRING,
//     FIT_MODE_SET_POS,
// }fitness_mode_e;


typedef enum {
    Standard_Mode       = 0x00,    //标准恒力模式
    Centred_Mode        = 0x01,    //向心模式
    Centrifugal_Mode    = 0x02,    //离心模式
    ConstantSpeed_Mode  = 0x03,    //等速模式
    Spring_Mode         = 0x04,    //弹簧模式
    PosOffset_Mode      = 0X0F,    //位置设置模式
    Eco_Mode            = 0x1A,    //节能模式
    Stop_Mode           = 0x1F,    //停止模式
    Enable_Mode         = 0xAA,    //使能模式
    CalebraPos_Mode     = 0xCB,    //位置校准模式
    LOWPOWER_Mode       = 0xDD,    //低功耗模式
    Disable_Mode        = 0x55,    //失能模式
    Star_Mode           = 0xEE,    //开始模式

    Test_Mode         = 0xF0, //位置测试模式
    Test_Mode_current = 0xF1, //电流测试模式
    Test_Mode_speed   = 0xF2, //速度测试模式
    Test_Mode_pos     = 0xF3, //位置测试模式
    
    Idle_Mode           = 0xFE,     //待机模式    
}fitness_mode_e;

typedef struct _fitness
{
    uint8_t mode; 
    uint32_t torque;    //unit: mA
		uint32_t torque_last;    //unit: mA
    int32_t pos_offset;
    int32_t circule_cnt;
    int32_t pull_distance;
    int32_t pullback_distance;
    int16_t Parameter
}fitness_t;


 void fit_init();
void fitness_fsm_fv_M1();
void fitness_fsm_fv_M2();




