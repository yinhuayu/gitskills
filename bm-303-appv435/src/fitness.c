#include "fitness.h"
#include "enc.h"
#include "foc.h"
#include "foc_cfg.h"
#include "PROTECT.h"

#include "pwm6.h"
extern enc_var_t ev_M1;
extern enc_var_t ev_M2;
static unsigned int DelayCnt_M1 = 0;
static unsigned int DelayCnt_M2 = 0;
uint32_t CalbrtimeM1 = 0;
uint32_t CalbrtimeM2 = 0;
extern const foc_func_set_t ffs1;
extern const foc_func_set_t ffs2;
uint16_t PullBackSpeed = 4000; //正常回绳速度 不超过200cm/s  （默认PullBackSpeed = 4000  因约为D10cm  所以 200cm/s）

uint16_t MaxSpeedLimit = 4000; //正常运行最大速度限制 不超过200cm/s  （默认MaxSpeedLimit = 4000  因约为D10cm  所以 200cm/s）
uint16_t before_CalebraPos_ModevM1;
uint16_t before_CalebraPos_ModevM2;
uint16_t PreventCrashSpeed = 2600; // 3000; //回收过程中，防止撞击提前减速的速度域值 不超过200cm/s  （默认PreventCrashSpeed = 3000  因约为D10cm  所以 150cm/s）
uint16_t InitPullBackSpeed = 300;  //初始化回绳速度 不超过30cm/s  （默认InitPullBackSpeed = 300  因D10cm  所以 15cm/s）
uint16_t PullBackDistan = 1500;    //起始前移距离 不超过50cm 默认21CM  (值*28/2048（cm）)
uint16_t ForceLimitPos = 146;      // 292;      //保护点后移距离 不超过50cm 默认4cm (值*28/2048（cm）)
uint16_t LinerForceChagePos = 365; // 650;  //365->5cm    650->8cm
uint16_t PreventCrashPos = 1300;
uint16_t InitTorque = 990;               //基础力 (值/198  = x （0.5kg）)
uint16_t ResTorque = 396;                // 1440;               //系统阻力补偿 (值/198  = x （0.5kg）)
uint16_t ConstantSpeedBackTorque = 1980; //等速模式下的回绳力 (值/198  = x （0.5kg）)
uint16_t ConstantSpeedSet = 600;         //等速模式速度设定 (0-2000->0-100cm/s) 默认30cm/s
uint8_t RatioUp = 55;
uint8_t RatioDown = 100;
int32_t Power_Out_limit = 1500; // 1500W   1W 为单位
int32_t X_M1;
int32_t X_M2;
int pos_expect = 0; //位置设置模式下的设置值
int32_t Power_Out_M1 = 0;
int32_t Power_Out_M2 = 0;

int Amplitude1 = 4;  //幅值圈数  位置控制的sin波 单位（6圈）
float Cycles1 = 0.1; //周期  位置控制的sin波 单位（10秒）
int Amplitude2 = 4;  //幅值圈数  位置控制的sin波 单位（6圈）
float Cycles2 = 0.1; //周期  位置控制的sin波 单位（10秒）

int32_t TimeCltCntM1 = 0; // M1主动机模式时，波形发生时钟，可设置到1000后置零 10s
int32_t TimeCltCntM2 = 0; // M2主动机模式时，波形发生时钟，可设置到1000后置零 10s

int32_t TimeM1 = 0;       // 记次器
int32_t TimeM2 = 0;       // 记次器
extern int Amplitude_buf; //来自usart 的设置。特定时机再更新。
extern float Cycles_buf;  //来自usart 的设置。特定时机再更新。

extern const short FYLcos[];
uint32_t Torque_Set_M1 = 1000;
uint32_t Torque_Set_M1_last = 0;
uint32_t Torque_Set_M2 = 1000;
uint32_t Torque_Set_M2_last = 0;
int8_t Stop_Flag_M1 = 0;
int8_t Stop_Flag_M2 = 0;
int8_t Disable_flag_M1 = 0;
int8_t Disable_flag_M2 = 0;
int8_t Vibra_privent_flag_M1 = 0;
int8_t Vibra_privent_flag_M2 = 0;
int8_t ReduceSpeed_M1 = 0;
int8_t ReduceSpeed_M2 = 0;
int32_t Torque_Offset_M1 = 0;
int32_t Torque_Offset_M2 = 0;
int8_t Run_State_M1 = 0; // M1运行状态  0停  1正转 -1反转
int8_t Run_State_M2 = 0; // M2运行状态  0停  1正转 -1反转

fitness_t fit_M1;
fitness_t fit_M2;

extern uint8_t ErrM1;
extern uint8_t ErrM2;
extern uint8_t ErrCoder;
extern uint8_t Err_F_T_M1;   // 0x0：无错位   	0x:01：M1高温警告       0x02：M1高温限制      0x03：M1高温失效
extern uint8_t Err_F_T_M2;   //					        0x:01：M2高温警告       0x02：M1高温限制      0x03：M1高温失效
extern uint8_t Err_F_T_PCB;  //              	  0x:01：PCBA高温警告     0x02：PCB高温限制     0x03：PCB高温失效
extern uint8_t Err_F_Sp;     //              		  0x:01：超速警告     	0x02：超速限制     	  0x03：超速失效
extern uint8_t Err_F_Vol;    //              		0x:01：高电压警告     	0x02：高电压限制      0x03：高电压失效
extern uint8_t Err_F_Cur;    //              		0x:01：高电流警告     	0x02：高电流限制      0x03：高电流失效
extern fitness_t fit_M1_buf; //来自usart 的设置。特定时机再更新。
extern fitness_t fit_M2_buf; //来自usart 的设置。特定时机再更新。
extern int M1_Temp;
extern int M2_Temp;

int32_t M1Speed[11];
int32_t M2Speed[11];

void fit_init()
{
  fit_M1.mode = Idle_Mode;
  fit_M2.mode = Idle_Mode;
  fit_M1_buf.mode = Idle_Mode;
  fit_M2_buf.mode = Idle_Mode;
  fit_M1_buf.torque = InitTorque;
  fit_M1_buf.torque_last = InitTorque;
  fit_M2_buf.torque = InitTorque;
  fit_M2_buf.torque_last = InitTorque;
}
// called each 10ms.
uint32_t fit_Prv_Shake(uint32_t T_Set_M, uint32_t T_Set_M_last, uint32_t stepNum)
{
  if (T_Set_M_last > (T_Set_M + stepNum))
  {
    T_Set_M = T_Set_M_last - stepNum;
  }
  else if (T_Set_M_last < (T_Set_M - stepNum))
  {
    T_Set_M = T_Set_M_last + stepNum;
  }
  else
  {
    T_Set_M = T_Set_M_last;
  }
  return T_Set_M;
}
void fitness_fsm_fv_M1()
{

  if (TimeM1 < 1000) // 10S
  {
    TimeM1++;
  }
  else
  {
    TimeM1 = 0;
  }

  fit_M1.torque = fit_M1_buf.torque;

  switch (fit_M1.mode)
  {
  //恒力模式
  case Standard_Mode: // 0x00
  {
    Torque_Set_M1 = fit_M1.torque; // max range [-32767 ~ 32767]
  }
  break;

  case Centred_Mode: // 0x01向心模式
  {
    RatioUp = 46;

    if (fv_M1.speed_rpm < -20) //如果 速度>0   如果 拉出
    {
      Torque_Set_M1 = fit_M1.torque; // max range [-32767 ~ 32767]
			Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 900);
    }
    else
    {
      Torque_Set_M1 = fit_M1.torque * RatioUp / RatioDown; // max range [-32767 ~ 32767]
			Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 60);
    }
    
  }
  break;

  case Centrifugal_Mode: // 0x02离心模式
  {
    RatioUp = 65;
    if (fv_M1.speed_rpm > 20) //如果 速度小于0   如果 回收
    {
      Torque_Set_M1 = fit_M1.torque; // max range [-32767 ~ 32767]
    }
    else
    {
      Torque_Set_M1 = fit_M1.torque * RatioUp / RatioDown; // max range [-32767 ~ 32767]
    }
    Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 900);
  }
  break;

  case ConstantSpeed_Mode: // 0x03等速模式
  {
    ConstantSpeedSet = 1320; //(fit_M1.torque / 198) * 20;
    if (ConstantSpeedSet < 200)
    {
      ConstantSpeedSet = 200;
    }
    else if (ConstantSpeedSet > 1600)
    {
      ConstantSpeedSet = 1600;
    }

    if (fv_M1.speed_rpm < -ConstantSpeedSet) //拉出  且速度大于10rpm  5cm/s
    {                                        // Torque_Set_M1 = InitTorque + (ABS(fv_M1.speed_rpm) - ConstantSpeedSet) * fit_M1.torque / 1024;
      Torque_Set_M1 = InitTorque + (ABS(fv_M1.speed_rpm) - ConstantSpeedSet) * 10;
      Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 1500);
    }
    else
    {
      Torque_Set_M1 = ConstantSpeedBackTorque;
      Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 256);
    }
  }
  break;

  //根据位置误差 增加最大力矩
  case Spring_Mode: // 0x04
  {

    X_M1 = ABS(fv_M1.pid_pos.err) - PullBackDistan - ForceLimitPos;
    if (X_M1 > 0)
    {
      Torque_Set_M1 = InitTorque + (X_M1 * fit_M1.torque / 8192);
    }
    else
    {
      Torque_Set_M1 = InitTorque;
    }

    if (Torque_Set_M1 > 80 * 198)
    {
      Torque_Set_M1 = 80 * 198;
    }
  }
  break;

  case PosOffset_Mode: // 0x0F
  {

    // fv_M1.fsm_state = FSM_STATE_SENSOR_POSITION;
    // fv_M1.pos_ref = pos_expect; //设置位置参考值即可
  }
  break;

  case Eco_Mode: // 0x1A 节能模式，应IMB要求，要有一个保持机出力，且不能被设置的模式，此时风扇要关闭
  {

    Torque_Set_M1 = InitTorque; 
  }
  break;
  case Stop_Mode: // 0x1f
  {

    if (Stop_Flag_M1 == 0)
    {
      Stop_Flag_M1 = 1;
      fv_M1.pos_ref = fv_M1.pos_ref - fv_M1.pid_pos.err;
      Torque_Set_M1 = 25000;
      DelayCnt_M1 = 0;
    }
  }
  break;
  case LOWPOWER_Mode: // 0xDD
  {
    fv_M1.fsm_state = FSM_STATE_SENSOR_OPENLOOP;
    fv_M1.iq_ref = 0;
    fv_M1.id_ref = 0;
    DelayCnt_M1 = 0;
    PreDriverM1_OFF();
  }
  break;
  case Disable_Mode: // 0x55
  {
    fv_M1.fsm_state = FSM_STATE_SENSOR_OPENLOOP;
    fv_M1.iq_ref = 0;
    fv_M1.id_ref = 0;
    if (Disable_flag_M1 == 0)
    {
      Disable_flag_M1 = 1;
      fit_M1.mode = Disable_Mode;
      fit_M1_buf.mode = Disable_Mode;
      DelayCnt_M1 = 0;
    }
  }
  break;
  case Enable_Mode: // 0xAA
  {

    fit_M1.mode = Idle_Mode;
    fit_M1_buf.mode = Idle_Mode;
    fit_M1_buf.torque = InitTorque;
    fit_M1_buf.torque_last = InitTorque;
    fv_M1.pos_ref = 0;     //设定当前位置为参考点。
    fv_M1.pos_fdbk = 0;    //位置反馈归零
    ev_M1.raw_odmetry = 0; //光编里程计归
    TimeCltCntM1 = 0;
    DelayCnt_M1 = 0;
    PreDriverM1_ON();
    Stop_Flag_M1 = 0;
    Disable_flag_M1 = 0;
    foc_register_func_set(&fv_M1, &ffs1);
    foc_algorithm_init2(&fv_M1);
  }
  break;
  case CalebraPos_Mode: // 0xCB
  {
    // fit_M1.torque = InitTorque;
    fv_M1.pos_ref = fv_M1.pos_fdbk + PullBackDistan; //设定当前位置为参考点。
    fv_M1.pos_fdbk = 0;                              //位置反馈归零
    ev_M1.raw_odmetry = 0;                           //光编里程计归零

    // fv_M1.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION;
    fv_M1.fsm_state = FSM_STATE_SENSOR_POSITION;
    fit_M1.mode = before_CalebraPos_ModevM1;
    fit_M1_buf.mode = before_CalebraPos_ModevM1;
    TimeCltCntM1 = 0;
    DelayCnt_M1 = 0;
  }
  break;

    //启动收回绳子, 速度模式, 监控力矩iq
  case Star_Mode: // 0xEE
  {
    fv_M1.fsm_state = FSM_STATE_SENSOR_SPEED;
    fv_M1.speed_ref = InitPullBackSpeed; //给定+30rpm慢速收回

    //同时检查fv_M1的iq， 直到大于设定阈值后认为堵转 收回ok
    if (fv_M1.iq > 2400 || fv_M1.iq < -2400)
    {
      //切换为位置环

      DelayCnt_M1++;
      if (DelayCnt_M1 > 50) //持续0.8s
      {
        fit_M1.torque = InitTorque;
        // fv_M1.pos_ref = fv_M1.pos_fdbk;							//设定当前位置为参考点。
        fv_M1.pos_ref = fv_M1.pos_fdbk + PullBackDistan; //设定当前位置为参考点。
        fv_M1.pos_fdbk = 0;                              //位置反馈归零
        ev_M1.raw_odmetry = 0;                           //光编里程计归零

        // fv_M1.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION;
        fv_M1.fsm_state = FSM_STATE_SENSOR_POSITION;
        fit_M1.mode = Standard_Mode;
        fit_M1_buf.mode = Standard_Mode;
        TimeCltCntM1 = 0;
        DelayCnt_M1 = 0;
      }
    }
    else
    {
      DelayCnt_M1 = 0;
    }
  }
  break;

  case Test_Mode: // 0xF0
  {
    fv_M1.fsm_state = FSM_STATE_ROTOR_ALIGN_VQ;
  }
  break;
  case Test_Mode_current: // 0xF1
  {
    fv_M1.fsm_state = FSM_STATE_SENSOR_CURRENT;
  }
  break;
  case Test_Mode_speed: // 0xF2
  {
    fv_M1.fsm_state = FSM_STATE_SENSOR_SPEED;
  }
  break;
  case Test_Mode_pos: // 0xF3
  {
    fv_M1.fsm_state = FSM_STATE_SENSOR_POSITION;
    // // fv_M1.pos_ref = Amplitude1 * 4 * FYLcos[(int)(Cycles1 * TimeCltCntM1) % 100] - Amplitude1 * 1024;
    // Torque_Set_M1 = fit_M1.torque;
    // if ((int)(Cycles1 * TimeCltCntM1) % 100 < 50)
    // {
    //   fv_M1.pos_ref = -Amplitude1 * 1024;
    // }
    // else
    // {
    //   fv_M1.pos_ref = 0;
    // }

    // if ((int)(Cycles1 * TimeCltCntM1) % 100 == 0 && ((Amplitude1 != Amplitude_buf) || (Cycles1 != Cycles_buf) || (Torque_Set_M1 != fit_M1.torque)))
    // {
    //   Amplitude1 = Amplitude_buf;
    //   Cycles1 = Cycles_buf;
    //   Torque_Set_M1 = fit_M1.torque;
    // }
    // if (TimeCltCntM1++ >= 1000) // 10S
    // {
    //   TimeCltCntM1 = 0;
    // }
  }
  break;

  case Idle_Mode: // 0xFE
  {
    fv_M1.id_ref = 0;
    fv_M1.iq_ref = 0;
    fv_M1.speed_ref = 0;
    fv_M1.pos_ref = 0;

    DelayCnt_M1++;
    if (DelayCnt_M1 > 100) //等待3s 进入下一状态DelayCnt_M1 = 0 ;(恒力模式)
    {
      fit_M1.mode = Star_Mode;
      fit_M1_buf.mode = Star_Mode;
      DelayCnt_M1 = 0;
    }
    // do nothing.
  }
  break;
  default:
    break;
  }

  //正常运转的模式
  if (fv_M1.fsm_state == FSM_STATE_SENSOR_POSITION)
  {
    if (Stop_Flag_M1 != 1)
    {
      if (Torque_Set_M1 < InitTorque)
      {
        Torque_Set_M1 = InitTorque;
      }

      ////回收和拉出速度限制控制
      fv_M1.pid_pos.max_out = MaxSpeedLimit; //回收和拉出 运行速度限制,6000->600RPM  约为 3m/s; 4000->400rpm约为 2m/s

      // L3段 的力随距离渐变控制
      if ((fv_M1.pos_fdbk > (-ForceLimitPos - LinerForceChagePos)) && (fv_M1.pos_fdbk < -ForceLimitPos) && fit_M1.mode != ConstantSpeed_Mode && fit_M1.mode != Spring_Mode)
      {
        if (Torque_Set_M1 > InitTorque)
        {
          Torque_Set_M1 = InitTorque + (ABS(fv_M1.pos_fdbk) - ForceLimitPos) * (Torque_Set_M1 - InitTorque) / LinerForceChagePos;
        }
      }
      // L2段和L2段之前(含L1段和反向被拉走)  只有基础力
      if (fv_M1.pos_fdbk >= (-ForceLimitPos))
      {
        Torque_Set_M1 = InitTorque;
      }
      //移动滤波
      M1Speed[10] -= M1Speed[TimeM1 % 10];
      M1Speed[TimeM1 % 10] = fv_M1.speed_rpm;
      M1Speed[10] += fv_M1.speed_rpm;

      //方向判断（利用速度值判断方向，也处理了力量切变抖动）
      if (M1Speed[10] > 400) //如果 速度大于4rpm（2cm/s）   如果 回收
      {
        Run_State_M1 = -1;

        if (fit_M1.mode == Standard_Mode || fit_M1.mode == Spring_Mode)
        {
          Torque_Offset_M1 = ResTorque * Torque_Set_M1 / 3960 + 594;
          Torque_Set_M1 += Torque_Offset_M1; //根据力的大小补偿回绳拉力
        }

        //末段回收速度过快，做减速保护
        if (fv_M1.pos_fdbk >= (-PreventCrashPos))
        {
          if (M1Speed[10] > PreventCrashSpeed * 10) // PreventCrashSpeed   20000->  >1m/s
          {
            ReduceSpeed_M1 = 100;
            // Torque_Set_M1 = InitTorque;
          }
          if (ReduceSpeed_M1 != 0)
          {
            fv_M1.pid_pos.max_out = 100; //速度限制
          }
        }
        Vibra_privent_flag_M1 = 1;
      }
      if (M1Speed[10] <= 400 && M1Speed[10] >= -400) //如果 速度小于等于2rpm   防止切变震动
      {

        CalbrtimeM1++;
        Run_State_M1 = 0;
        if (ReduceSpeed_M1 != 0)
        {
          // ReduceSpeed_M1--;
          fv_M1.pid_pos.max_out = 100; //速度限制
        }
        if (Vibra_privent_flag_M1 == 1)
        {
          if (fit_M1.mode == Standard_Mode || fit_M1.mode == Spring_Mode)
          {
            Torque_Offset_M1 = ResTorque * Torque_Set_M1 / 3960 + 594;
            Torque_Set_M1 += Torque_Offset_M1; //根据力的大小补偿回绳拉力
          }
          // if (M1Speed[10] > 20000 && fv_M1.pos_fdbk >= -4 * LinerForceChagePos) //    20000->  >1m/s
          // {
          //   Torque_Set_M1 = InitTorque;
          //   fv_M1.pid_pos.max_out = 300; //速度限制
          // }
        }
        else if (Vibra_privent_flag_M1 == 0)
        {
          if (fit_M1.mode == Standard_Mode || fit_M1.mode == Spring_Mode)
          {
            Torque_Offset_M1 = ResTorque * Torque_Set_M1 / 3960 + 594;
            Torque_Set_M1 += Torque_Offset_M1; //根据力的大小补偿回绳拉力
          }
        }
      }
      else
      {
        CalbrtimeM1 = 0;
        CalbrtimeM2 = 0;
      }
      if (M1Speed[10] < -400) //如果 速度大于2rpm   如果 拉出
      {
        Run_State_M1 = 1;
        if (ReduceSpeed_M1 != 0)
        {
          ReduceSpeed_M1--;
        }
        Vibra_privent_flag_M1 = 0;
      }

      if (CalbrtimeM1 > 1000 && (fv_M1.pos_fdbk >= 11 * (-ForceLimitPos)))
      {
        before_CalebraPos_ModevM1 = fit_M1.mode;
        fit_M1.mode = CalebraPos_Mode;
        fit_M1_buf.mode = CalebraPos_Mode;
        CalbrtimeM1 = 0;
      }

      // //功率限制控制  Power = speed(m/s) * F(kg*9.8) = fv_M1.speed_rpm/2000 * (fv_M1.iq * 9 >> 12)*10   （W） ;
      // //                   = fv_M1.speed_rpm* (fv_M1.iq * 900 >> 15) /25 （0.01W）
      // //                   = fv_M1.speed_rpm* fv_M1.iq * 36 >> 16 （0.01W  10mW）
      // Power_Out_M1 = ABS( M1Speed[10]/20000 * (fv_M1.iq * 10 >> 12)*10); //单位10mW
      // if (Power_Out_M1 > Power_Out_limit)
      // {
      // //Torque_Set_M1/2/198 *10(N)  *  M1Speed[10]/20000 (m/s)  = Power_Out_limit
      // //Torque_Set_M1 = Power_Out_limit*2000*2*198/ M1Speed[10]
      //   // Torque_Set_M1 = (Power_Out_limit<<16 /36 / fv_M1.speed_rpm) *9 >>12 *2*198
      //   //               = Power_Out_limit<<4 /9 / fv_M1.speed_rpm *99
      //   Torque_Set_M1 = ABS(Power_Out_limit/M1Speed[10]* 18); //使用最大的功率限制结合当前速度，计算出应当设置的力值
      // }
    }

    //力值设定
    fv_M1.pid_speed.max_out = Torque_Set_M1 - fv_M1.inertia_compensation; //电流限制/力量调节    //max range [-32767.0 ~ 32767.9999]
  }
  if (fv_M1.fsm_state == FSM_STATE_SENSOR_SPEED)
  {
    fv_M1.pid_speed.max_out = 3000; //电流限制/力量调节   //max range [-32767.0 ~ 32767.9999]
  }

  //力保护
  if (Torque_Set_M1 >= 6000)
  {
    switch (ErrCoder)
    {
    case 3:
      Torque_Set_M1 = InitTorque;
      Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 144);
      if (Torque_Set_M1 <= InitTorque)
      {

        fit_M1.mode = Disable_Mode;
        fit_M1_buf.mode = Disable_Mode;
      }
      break;
    case 2:

      Torque_Set_M1 = 6000;
      fit_M1_buf.torque = 6000;
      Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 72); //、、、

      break;
    case 1:
      Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 72); //、、、

      break;

    default:
      break;
    }
    fv_M1.pid_speed.max_out = Torque_Set_M1; //电流限制/力量调节    //max range [-32767.0 ~ 32767.9999]
  }
  else
  {
    if (ErrCoder == 3)
    {

      Torque_Set_M1 = InitTorque;
      Torque_Set_M1 = fit_Prv_Shake(Torque_Set_M1, Torque_Set_M1_last, 144);
      if (Torque_Set_M1 <= InitTorque)
      {

        fit_M1.mode = Disable_Mode;
        fit_M1_buf.mode = Disable_Mode;
      }
    }
  }

  Torque_Set_M1_last = Torque_Set_M1;

  //状态更新条件，力更新条件
  if ((fv_M1.pos_fdbk >= -3 * (ForceLimitPos + LinerForceChagePos)) && (fit_M1.mode != fit_M1_buf.mode)) //状态更新 只在回零位附近才执行
  {
    fit_M1.mode = fit_M1_buf.mode;
  }
  if ((fv_M1.pos_fdbk >= -3 * (ForceLimitPos + LinerForceChagePos)) && (fit_M1.torque != fit_M1_buf.torque)) //状态更新 只在回零位附近才执行
  {
    fit_M1.torque = fit_M1_buf.torque;
  }
  if ((fit_M1_buf.mode == Disable_Mode) || (fit_M1_buf.mode == Stop_Mode) || (fit_M1_buf.mode == Enable_Mode) || (fit_M1_buf.mode == LOWPOWER_Mode))
  {
    fit_M1.mode = fit_M1_buf.mode;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// called each 10ms.
void fitness_fsm_fv_M2()
{
  if (TimeM2 < 1000) // 10S
  {
    TimeM2++;
  }
  else
  {
    TimeM2 = 0;
  }
  fit_M2.torque = fit_M2_buf.torque;

  switch (fit_M2.mode)
  {
  //恒力模式
  case Standard_Mode: // 0x00
  {
    Torque_Set_M2 = fit_M2.torque; // max range [-32767 ~ 32767]
  }
  break;

  case Centred_Mode: // 0x01向心模式
  {
    RatioUp = 46;
    if (fv_M2.speed_rpm > 20) //如果 速度>0   如果 拉出
    {
      Torque_Set_M2 = fit_M2.torque; // max range [-32767 ~ 32767]
			Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 900);
    }
    else
    {
      Torque_Set_M2 = fit_M2.torque * RatioUp / RatioDown; // max range [-32767 ~ 32767]
			Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 60);
    }
    
  }
  break;

  case Centrifugal_Mode: // 0x02离心模式
  {
    RatioUp = 65;
    if (fv_M2.speed_rpm < -20) //如果 速度小于0   如果 回收
    {
      Torque_Set_M2 = fit_M2.torque; // max range [-32767 ~ 32767]
    }
    else
    {
      Torque_Set_M2 = fit_M2.torque * RatioUp / RatioDown; // max range [-32767 ~ 32767]
    }
    Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 900);
  }
  break;

  case ConstantSpeed_Mode: // 0x03等速模式
  {

    ConstantSpeedSet = 1320; // (fit_M2.torque / 198) * 20;
    if (ConstantSpeedSet < 200)
    {
      ConstantSpeedSet = 200;
    }
    else if (ConstantSpeedSet > 1600)
    {
      ConstantSpeedSet = 1600;
    }

    if (fv_M2.speed_rpm > ConstantSpeedSet)
    // Torque_Set_M2 = InitTorque + (ABS(fv_M2.speed_rpm) - ConstantSpeedSet) * fit_M2.torque / 1024;
    {
      Torque_Set_M2 = InitTorque + (ABS(fv_M2.speed_rpm) - ConstantSpeedSet) * 10;
      Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 1500);
    }
    else
    {
      Torque_Set_M2 = ConstantSpeedBackTorque;
      Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 256);
    }
  }
  break;

  //根据位置误差 增加最大力矩
  case Spring_Mode: // 0x04
  {

    X_M2 = ABS(fv_M2.pid_pos.err) - PullBackDistan - ForceLimitPos;

    if (X_M2 > 0)
    {
      Torque_Set_M2 = InitTorque + (X_M2 * fit_M2.torque / 8192);
    }
    else
    {
      Torque_Set_M2 = InitTorque;
    }
    if (Torque_Set_M2 > 80 * 198)
    {
      Torque_Set_M2 = 80 * 198;
    }
  }
  break;

  case PosOffset_Mode: // 0x0F
  {

    // fv_M2.fsm_state = FSM_STATE_SENSOR_POSITION;
    // fv_M2.pos_ref = pos_expect; //设置位置参考值即可
  }
  break;

  case Eco_Mode: // 0x1A  节能模式，应IMB要求，要有一个保持机出力，且不能被设置的模式，此时风扇要关闭
  {

    Torque_Set_M2 = InitTorque; // max range [-32767 ~ 32767]
  }
  break;
  case Stop_Mode: // 0x1f
  {
    if (Stop_Flag_M2 == 0)
    {
      Stop_Flag_M2 = 1;
      fv_M2.pos_ref = fv_M2.pos_ref - fv_M2.pid_pos.err;
      Torque_Set_M2 = 25000;
      DelayCnt_M2 = 0;
    }
  }
  break;
  case LOWPOWER_Mode: // 0xDD
  {
    fv_M2.fsm_state = FSM_STATE_SENSOR_OPENLOOP;
    fv_M2.iq_ref = 0;
    fv_M2.id_ref = 0;
    DelayCnt_M2 = 0;
    PreDriverM2_OFF();
  }
  break;
  case Disable_Mode: // 0x55
  {
    fv_M2.fsm_state = FSM_STATE_SENSOR_OPENLOOP;
    fv_M2.iq_ref = 0;
    fv_M2.id_ref = 0;
    if (Disable_flag_M2 == 0)
    {
      Disable_flag_M2 = 1;
      fit_M2.mode = Disable_Mode;
      fit_M2_buf.mode = Disable_Mode;
      DelayCnt_M2 = 0;
    }
  }
  break;

  case Enable_Mode: // 0xAA
  {
    fit_M2.mode = Idle_Mode;
    fit_M2_buf.mode = Idle_Mode;
    fit_M2_buf.torque = InitTorque;
    fit_M2_buf.torque_last = InitTorque;
    fv_M2.pos_ref = 0;     //设定当前位置为参考点。
    fv_M2.pos_fdbk = 0;    //设定当前位置为参考0点。
    ev_M2.raw_odmetry = 0; //光编里程计归零
    TimeCltCntM2 = 0;
    DelayCnt_M2 = 0;
    Stop_Flag_M2 = 0;
    Disable_flag_M2 = 0;
    PreDriverM2_ON();
    foc_register_func_set(&fv_M2, &ffs2);
    foc_algorithm_init2(&fv_M2);
  }
  break;
  case CalebraPos_Mode: // 0xCB
  {
    // fit_M2.torque = InitTorque;
    fv_M2.pos_ref = fv_M2.pos_fdbk - PullBackDistan; //设定当前位置为参考点。
    fv_M2.pos_fdbk = 0;                              //位置反馈归零
    ev_M2.raw_odmetry = 0;                           //光编里程计归零

    // fv_M2.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION;
    fv_M2.fsm_state = FSM_STATE_SENSOR_POSITION;
    fit_M2.mode = before_CalebraPos_ModevM2;
    fit_M2_buf.mode = before_CalebraPos_ModevM2;
    TimeCltCntM2 = 0;
    DelayCnt_M2 = 0;
  }
  break;

    //启动收回绳子, 速度模式, 监控力矩iq
  case Star_Mode: // 0xEE
  {
    fv_M2.fsm_state = FSM_STATE_SENSOR_SPEED;
    fv_M2.speed_ref = -InitPullBackSpeed; //给定30rpm慢速收回

    //同时检查fv_M2的iq， 直到大于设定阈值后认为堵转 收回ok
    if (fv_M2.iq > 2400 || fv_M2.iq < -2400)
    {
      //切换为位置环

      DelayCnt_M2++;
      if (DelayCnt_M2 > 50) //持续1s
      {
        fit_M2.torque = InitTorque;
        // fv_M2.pos_ref = fv_M2.pos_fdbk;					//设定当前位置为参考点。
        fv_M2.pos_ref = fv_M2.pos_fdbk - PullBackDistan; //设定当前位置为参考点。
        fv_M2.pos_fdbk = 0;                              //设定当前位置为参考0点。
        ev_M2.raw_odmetry = 0;                           //光编里程计归零

        // fv_M2.fsm_state = FSM_STATE_ROTOR_ALIGN_SENSOR_POSITION;
        fv_M2.fsm_state = FSM_STATE_SENSOR_POSITION;
        fit_M2.mode = Standard_Mode;
        fit_M2_buf.mode = Standard_Mode;
        TimeCltCntM2 = 0;
        DelayCnt_M2 = 0;
      }
    }
    else
    {
      DelayCnt_M2 = 0;
    }
  }
  break;

  case Test_Mode: // 0xF0
  {
    fv_M2.fsm_state = Star_Mode;
  }
  break;
  case Test_Mode_current: // 0xF1
  {
    fv_M2.fsm_state = FSM_STATE_SENSOR_CURRENT;
  }
  break;
  case Test_Mode_speed: // 0xF2
  {
    fv_M2.fsm_state = FSM_STATE_SENSOR_SPEED;
  }
  break;
  case Test_Mode_pos: // 0xF3
  {
    fv_M2.fsm_state = FSM_STATE_SENSOR_POSITION;
    // Torque_Set_M2 = fit_M2.torque;
    // // fv_M2.pos_ref = Amplitude2 * 4 * FYLcos[(int)(Cycles2 * TimeCltCntM2) % 100] - Amplitude2 * 1024;
    // if ((int)(Cycles2 * TimeCltCntM2) % 100 < 50)
    // {
    //   fv_M2.pos_ref = Amplitude2 * 1024;
    // }
    // else
    // {
    //   fv_M2.pos_ref = 0;
    // }

    // if ((int)(Cycles2 * TimeCltCntM2) % 100 == 0 && ((Amplitude2 != Amplitude_buf) || (Cycles2 != Cycles_buf) || (Torque_Set_M2 != fit_M2.torque)))
    // {
    //   Amplitude2 = Amplitude_buf;
    //   Cycles2 = Cycles_buf;
    //   Torque_Set_M2 = fit_M2.torque;
    // }

    // if (TimeCltCntM2++ >= 1000) // 3S
    // {
    //   TimeCltCntM2 = 0;
    // }
  }
  break;

  case Idle_Mode: // 0xFE
  {
    fv_M2.id_ref = 0;
    fv_M2.iq_ref = 0;
    fv_M2.speed_ref = 0;
    fv_M2.pos_ref = 0;

    DelayCnt_M2++;
    if (DelayCnt_M2 > 200) //等待3s 进入下一状态
    {
      fit_M2.mode = Star_Mode;
      fit_M2_buf.mode = Star_Mode;
      DelayCnt_M2 = 0;
    }
    // do nothing.
  }
  break;
  default:
    break;
  }

  if (fv_M2.fsm_state == FSM_STATE_SENSOR_POSITION)
  {
    if (Stop_Flag_M2 != 1)
    {
      if (Torque_Set_M2 < InitTorque) // 保持基本的力量
      {
        Torque_Set_M2 = InitTorque;
      }
      //回收和拉出速度限制控制
      fv_M2.pid_pos.max_out = MaxSpeedLimit; //拉出 运行速度限制,6000->600RPM  约为 3m/s; 4000->400rpm约为 2m/s
      // L3段 的力随距离渐变控制
      if (fv_M2.pos_fdbk < (ForceLimitPos + LinerForceChagePos) && fv_M2.pos_fdbk > ForceLimitPos && fit_M2.mode != ConstantSpeed_Mode && fit_M2.mode != Spring_Mode)
      {
        if (fit_M2.torque > InitTorque)
        {
          Torque_Set_M2 = InitTorque + (fv_M2.pos_fdbk - ForceLimitPos) * (Torque_Set_M2 - InitTorque) / LinerForceChagePos;
        }
      }
      // L2段之前(含L1段和反向被拉走)  只有基础力
      if (fv_M2.pos_fdbk <= ForceLimitPos)
      {
        Torque_Set_M2 = InitTorque;
      }
      //移动滤波 结果为累加值 比原值大10倍
      M2Speed[10] -= M2Speed[TimeM2 % 10];
      M2Speed[TimeM2 % 10] = fv_M2.speed_rpm;
      M2Speed[10] += fv_M2.speed_rpm;

      if (M2Speed[10] < -400) //如果 速度小于-4rpm 约为 0.02m/s    如果 回收
      {
        Run_State_M2 = -1;
        if (fit_M2.mode == Standard_Mode || fit_M2.mode == Spring_Mode)
        {
          Torque_Offset_M2 = ResTorque * Torque_Set_M2 / 3960 + 594;
          Torque_Set_M2 += Torque_Offset_M2; //根据力的大小补偿回绳拉力
        }
        //末段回收速度过快，做减速保护
        if (fv_M2.pos_fdbk <= PreventCrashPos)
        {
          if (M2Speed[10] < -(PreventCrashSpeed * 10)) //  PreventCrashSpeed=?   20000->  >1m/s
          {
            ReduceSpeed_M2 = 100;
            // Torque_Set_M2 = InitTorque;
          }
          if (ReduceSpeed_M2 != 0)
          {
            fv_M2.pid_pos.max_out = 100; //速度限制
          }
        }
        Vibra_privent_flag_M2 = 1;
      }
      if (M2Speed[10] <= 400 && M2Speed[10] >= -400) //如果 速度小于等于2rpm   防止切变震动
      {
        CalbrtimeM2++;
        Run_State_M2 = 0;
        if (ReduceSpeed_M2 != 0)
        {
          // ReduceSpeed_M2--;
          fv_M2.pid_pos.max_out = 100; //速度限制
        }
        if (Vibra_privent_flag_M2 == 1)
        {
          if (fit_M2.mode == Standard_Mode || fit_M2.mode == Spring_Mode)
          {
            Torque_Offset_M2 = ResTorque * Torque_Set_M2 / 3960 + 594;
            Torque_Set_M2 += Torque_Offset_M2; //根据力的大小补偿回绳拉力
          }
          // {
          //   Torque_Set_M2 = InitTorque;
          //   fv_M2.pid_pos.max_out = 30; //归位速度限制
          // }
        }
        else if (Vibra_privent_flag_M2 == 0)
        {
          if (fit_M2.mode == Standard_Mode || fit_M2.mode == Spring_Mode)
          {
            Torque_Offset_M2 = ResTorque * Torque_Set_M2 / 3960 + 594;
            Torque_Set_M2 += Torque_Offset_M2; //根据力的大小补偿回绳拉力
          }
        }
      }
      else
      {
        CalbrtimeM1 = 0;
        CalbrtimeM2 = 0;
      }

      if (M2Speed[10] > 400) //如果 速度大于4rpm   如果 拉出
      {
        Run_State_M2 = -1;
        if (ReduceSpeed_M2 != 0)
        {
          ReduceSpeed_M2--;
        }
        Vibra_privent_flag_M2 = 0;
      }

      if (CalbrtimeM2 > 1000 && (fv_M2.pos_fdbk <= 11 * ForceLimitPos))
      {

        before_CalebraPos_ModevM2 = fit_M2.mode;

        fit_M2.mode = CalebraPos_Mode;
        fit_M2_buf.mode = CalebraPos_Mode;
        CalbrtimeM2 = 0;
      }

      // //功率限制控制  Power = speed(m/s) * F(kg*9.8) = M2Speed[10]/20000 * (fv_M2.iq * 10 >> 12)*10   （W） ;

      // Power_Out_M2 = ABS( M2Speed[10]/20000 * (fv_M2.iq * 10 >> 12)*10); //单位10mW
      // if (Power_Out_M2 > Power_Out_limit)
      // {

      //   // Torque_Set_M2 = (Power_Out_limit<<16 /36 / fv_M2.speed_rpm) *9 >>12 *2*198
      //   //               = Power_Out_limit<<4 /9 / fv_M2.speed_rpm *99
      //   Torque_Set_M2 = ABS(Power_Out_limit*M2Speed[10]* 18); //使用最大的功率限制结合当前速度，计算出应当设置的力值
      // }
    }
    //力值设定
    fv_M2.pid_speed.max_out = Torque_Set_M2 + fv_M2.inertia_compensation; //电流限制/力量调节    //max range [-32767.0 ~ 32767.9999]
  }
  if (fv_M2.fsm_state == FSM_STATE_SENSOR_SPEED)
  {
    fv_M2.pid_speed.max_out = 3000; //电流限制/力量调节    //max range [-32767.0 ~ 32767.9999]
  }

  //力量保护模式
  if (Torque_Set_M2 >= 6000)
  {
    switch (ErrCoder)
    {
    case 3:
      Torque_Set_M2 = InitTorque;
      Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 144);
      if (Torque_Set_M2 <= InitTorque)
      {

        fit_M2.mode = Disable_Mode;
        fit_M2_buf.mode = Disable_Mode;
      }
      break;
    case 2:
      Torque_Set_M2 = 6000;
      fit_M2_buf.torque = 6000;
      Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 72); //、、、

    case 1:
      Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 72); //、、、
      break;
    default:
      break;
    }
    fv_M2.pid_speed.max_out = Torque_Set_M2; //电流限制/力量调节    //max range [-32767.0 ~ 32767.9999]
  }
  else
  {
    if (ErrCoder == 3)
    {

      Torque_Set_M2 = InitTorque;
      Torque_Set_M2 = fit_Prv_Shake(Torque_Set_M2, Torque_Set_M2_last, 144);
      if (Torque_Set_M2 <= InitTorque)
      {
        fit_M2.mode = Disable_Mode;
        fit_M2_buf.mode = Disable_Mode;
      }
    }
  }

  Torque_Set_M2_last = Torque_Set_M2;

  if ((fv_M2.pos_fdbk <= 3 * (ForceLimitPos + LinerForceChagePos)) && (fit_M2.mode != fit_M2_buf.mode)) //状态更新 只在回零位附近才执行
  {
    fit_M2.mode = fit_M2_buf.mode;
  }

  if ((fv_M2.pos_fdbk <= 3 * (ForceLimitPos + LinerForceChagePos)) && (fit_M2.torque != fit_M2_buf.torque)) //状态更新 只在回零位附近才执行
  {
    fit_M2.torque = fit_M2_buf.torque;
  }

  if ((fit_M2_buf.mode == Disable_Mode) || (fit_M2_buf.mode == Stop_Mode) || (fit_M2_buf.mode == Enable_Mode) || (fit_M2_buf.mode == LOWPOWER_Mode))
  {
    fit_M2.mode = fit_M2_buf.mode;
  }
}
