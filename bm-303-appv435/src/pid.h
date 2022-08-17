/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __PID_H__
#define __PID_H__

#ifdef PID_H_GLOBAL
#define PID_H_EXTERN
#else
#define PID_H_EXTERN extern
#endif

#include <stdint.h>
// typedef struct pid *pid_t;

struct pid_param
{
  float p;
  float i;
  float d;
  float input_max_err;

  float max_out;
  float inte_limit;
};

struct pid
{
  struct pid_param param;

  float set;
  float get;

  float err;
  float last_err;

  float pout;
  float iout;
  float dout;
  float out;

  void (*f_param_init)(struct pid *pid,
                       float max_output,
                       float inte_limit,
                       float p,
                       float i,
                       float d);
  void (*f_pid_reset)(struct pid *pid, float p, float i, float d);
};

typedef struct pid_q15
{
  int32_t p;
  int32_t i;
  int32_t d;
  // int16_t input_max_err;

  int32_t deadband; //if abs(err) < deadband output = 0.
  int32_t max_out;              
  int32_t integral_limit;
  int32_t set;
  int32_t get;
  int32_t err;
  int32_t last_err;

  int32_t pout;
  int32_t iout;
  int32_t dout;
  int32_t out;

  // void (*f_param_init)(struct pid_q15 *pid,
  //                      int16_t max_output,
  //                      int16_t inte_limit,
  //                      int16_t p,
  //                      int16_t i,
  //                      int16_t d);
  // void (*f_pid_reset)(struct pid_q15 *pid, int16_t p, int16_t i, int16_t d);
}pid_q15_t; //max range [-32767.0 ~ 32767.9999]

// typedef struct pid_q15
// {
//   int32_t p;
//   int32_t i;
//   int32_t d;

//   int32_t max_out;
//   int32_t integral_limit;
//   int32_t set;
//   int32_t get;
//   int32_t err;
//   int32_t last_err;

//   int32_t pout;
//   int32_t iout;
//   int32_t dout;
//   int32_t out;
// }pid_q15_t;

void pid_struct_init(
    struct pid *pid,
    float maxout,
    float intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calculate(struct pid *pid, float fdb, float ref);


void pid_struct_init_q15(
    pid_q15_t *pid,
    int32_t maxout,
    int32_t integral_limit,

    int32_t kp,
    int32_t ki,
    int32_t kd);

int32_t pid_calculate_q15(pid_q15_t *pid, int32_t fdb, int32_t ref);


extern pid_q15_t pid_id;
extern pid_q15_t pid_iq;
extern pid_q15_t pid_speed;
extern pid_q15_t pid_pos;

#endif // __PID_H__
