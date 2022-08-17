#ifndef __MC_MATH
#define __MC_MATH

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ABS(x) (((x) > 0) ? (x) : -(x))
#define MAX(a, b) ( (a) >= (b) ? (a) : (b) )
#define MIN(a, b) ( (a) <= (b) ? (a) : (b) )
#define MAX_OF_3(a, b, c) MAX(a , MAX(b, c)) 

#define MAP(v,min1,max1,min2,max2) ((v-min1) * (max2-min2) / (max1-min1) + min2)


typedef struct
{
  int16_t cosx;
  int16_t sinx;
} trig_value_t;

typedef struct{
    uint8_t idx;
    int16_t *pfilter_buffer; 
    uint8_t len;
    int16_t output;
}slide_filter_t;

typedef struct {
    float z1;
    float tc;
    float in;
    float out;
} lpf1_t;

void lpf_1rd_init(lpf1_t *lpf1rd, float tc, float z);
float lpf_1rd_calc(lpf1_t *lpf1rd, float new_sample);

// typedef struct 
// {
//   int16_t part1;
//   int16_t part2;
// } curr_t;

// typedef struct 
// {
//   int16_t part1;
//   int16_t part2;
// } volt_t;

trig_value_t calc_sin_cos(int16_t x, int16_t *sinx, int16_t *cosx);

int16_t sigmoid_q15(int32_t); //q15 5~5
float sigmoid_float(float x);
float sigmoid_fake_float(float x);

int16_t sin_q15(uint16_t);
int16_t cos_q15(uint16_t);

void clarke_q15(int16_t ia, 
                int16_t ib,
                int16_t *ialfa,
                int16_t *ibeta);

void park_q15(  int16_t ialfa, 
                int16_t ibeta, 
                uint16_t theta,
                int16_t *id,
                int16_t *iq);

void inverse_park_q15(int16_t vq, 
                    int16_t vd, 
                    uint16_t theta,
                    int16_t *valfa, 
                    int16_t *vbeta);

void svpwm_q15(int16_t valfa, int16_t vbeta, int16_t tabc[]);
void svpwm_float(float valfa, 
                  float vbeta, 
                  float tabc[], 
                  uint16_t pwm_full, 
                  uint16_t ccr[]);

void clarke_float(float ia, 
                  float ib,
                  float *ialfa,
                  float *ibeta);

void park_float(float ialfa, 
                float ibeta, 
                float theta, 
                float *id, 
                float *iq);

void inverse_park_float(float vd, 
                        float vq, 
                        float theta,
                        float *valfa, 
                        float *vbeta);

int16_t slide_filter_int16(int16_t cur, 
                          uint8_t *idx, 
                          int16_t fil_buffer[], 
                          uint8_t len);

int max_of_3(int a, int b, int c);
int min_of_3(int a, int b, int c);
int32_t calc_theta_delta_raw(uint16_t *last, uint16_t cur, const int cnt_full_range);
int16_t Value_Limit(int32_t x, int16_t MIN, int16_t MAX);	
#ifdef __cplusplus
}
#endif

#endif