
//1 myself h
#include "mc_math.h"

//2 system etc... <stdint.h>  <string.h>
#include <math.h>


//3 external module.

//256
const int16_t sin_lut[] = {
     0,    807,   1614,   2419,   3224,   4026,   4826,   5623,
  6417,   7206,   7992,   8772,   9547,  10317,  11080,  11836,
 12585,  13327,  14060,  14785,  15501,  16208,  16905,  17591,
 18267,  18931,  19585,  20226,  20855,  21471,  22074,  22664,
 23241,  23803,  24350,  24883,  25401,  25903,  26389,  26860,
 27314,  27752,  28173,  28576,  28963,  29331,  29682,  30015,
 30330,  30626,  30904,  31163,  31403,  31624,  31825,  32008,
 32171,  32314,  32438,  32542,  32627,  32691,  32736,  32761,
 32766,  32751,  32716,  32661,  32587,  32493,  32379,  32245,
 32092,  31919,  31727,  31516,  31285,  31036,  30767,  30480,
 30175,  29851,  29509,  29149,  28772,  28377,  27964,  27535,
 27089,  26627,  26148,  25654,  25144,  24618,  24078,  23523,
 22954,  22371,  21774,  21165,  20542,  19907,  19259,  18600,
 17930,  17249,  16557,  15856,  15145,  14424,  13695,  12957,
 12212,  11459,  10699,   9933,   9161,   8383,   7600,   6812,
  6020,   5225,   4426,   3625,   2822,   2017,   1210,    403,
  -403,  -1210,  -2017,  -2822,  -3625,  -4426,  -5225,  -6020,
 -6812,  -7600,  -8383,  -9161,  -9933, -10699, -11459, -12212,
-12957, -13695, -14424, -15145, -15856, -16557, -17249, -17930,
-18600, -19259, -19907, -20542, -21165, -21774, -22371, -22954,
-23523, -24078, -24618, -25144, -25654, -26148, -26627, -27089,
-27535, -27964, -28377, -28772, -29149, -29509, -29851, -30175,
-30480, -30767, -31036, -31285, -31516, -31727, -31919, -32092,
-32245, -32379, -32493, -32587, -32661, -32716, -32751, -32766,
-32761, -32736, -32691, -32627, -32542, -32438, -32314, -32171,
-32008, -31825, -31624, -31403, -31163, -30904, -30626, -30330,
-30015, -29682, -29331, -28963, -28576, -28173, -27752, -27314,
-26860, -26389, -25903, -25401, -24883, -24350, -23803, -23241,
-22664, -22074, -21471, -20855, -20226, -19585, -18931, -18267,
-17591, -16905, -16208, -15501, -14785, -14060, -13327, -12585,
-11836, -11080, -10317,  -9547,  -8772,  -7992,  -7206,  -6417,
 -5623,  -4826,  -4026,  -3224,  -2419,  -1614,   -807,      0,
};

//256. use result = cos_lut[(uint8_t) u1 >> 8],  u1 is q15
const int16_t cos_lut[] = {
 32767,  32757,  32727,  32677,  32607,  32518,  32409,  32280,
 32132,  31964,  31777,  31570,  31345,  31100,  30836,  30554,
 30253,  29934,  29596,  29241,  28868,  28477,  28069,  27644,
 27202,  26744,  26269,  25779,  25273,  24751,  24215,  23663,
 23098,  22518,  21925,  21318,  20699,  20067,  19422,  18766,
 18099,  17420,  16731,  16032,  15323,  14605,  13878,  13142,
 12399,  11648,  10890,  10125,   9354,   8578,   7796,   7009,
  6219,   5424,   4626,   3826,   3023,   2218,   1412,    605,
  -201,  -1009,  -1815,  -2621,  -3425,  -4226,  -5026,  -5822,
 -6615,  -7403,  -8187,  -8967,  -9740, -10508, -11270, -12024,
-12772, -13511, -14243, -14965, -15679, -16383, -17077, -17761,
-18434, -19096, -19746, -20384, -21010, -21623, -22223, -22810,
-23382, -23941, -24485, -25014, -25528, -26026, -26509, -26975,
-27425, -27859, -28275, -28674, -29056, -29421, -29767, -30096,
-30406, -30697, -30970, -31225, -31460, -31676, -31873, -32050,
-32209, -32347, -32466, -32565, -32645, -32704, -32744, -32764,
-32764, -32744, -32704, -32645, -32565, -32466, -32347, -32209,
-32050, -31873, -31676, -31460, -31225, -30970, -30697, -30406,
-30096, -29767, -29421, -29056, -28674, -28275, -27859, -27425,
-26975, -26509, -26026, -25528, -25014, -24485, -23941, -23382,
-22810, -22223, -21623, -21010, -20384, -19746, -19096, -18434,
-17761, -17077, -16383, -15679, -14965, -14243, -13511, -12772,
-12024, -11270, -10508,  -9740,  -8967,  -8187,  -7403,  -6615,
 -5822,  -5026,  -4226,  -3425,  -2621,  -1815,  -1009,   -201,
   605,   1412,   2218,   3023,   3826,   4626,   5424,   6219,
  7009,   7796,   8578,   9354,  10125,  10890,  11648,  12399,
 13142,  13878,  14605,  15323,  16032,  16731,  17420,  18099,
 18766,  19422,  20067,  20699,  21318,  21925,  22518,  23098,
 23663,  24215,  24751,  25273,  25779,  26269,  26744,  27202,
 27644,  28069,  28477,  28868,  29241,  29596,  29934,  30253,
 30554,  30836,  31100,  31345,  31570,  31777,  31964,  32132,
 32280,  32409,  32518,  32607,  32677,  32727,  32757,  32767,
};

//int32_t q15  -5 ~ 5. result /= 32768 
const int32_t exp_lut[] = {
   220,    229,    238,    248,    258,    268,    279,    290, 
   302,    314,    326,    339,    353,    367,    382,    397,
   413,    430,    447,    465,    483,    503,    523,    544,
   565,    588,    612,    636,    661,    688,    715,    744,
   774,    805,    837,    871,    905,    942,    979,   1019,
  1059,   1102,   1146,   1192,   1239,   1289,   1340,   1394,
  1450,   1508,   1568,   1631,   1696,   1764,   1835,   1908,
  1984,   2064,   2146,   2232,   2321,   2414,   2511,   2611,
  2716,   2824,   2937,   3055,   3177,   3304,   3436,   3574,
  3717,   3865,   4020,   4181,   4348,   4522,   4703,   4891,
  5086,   5290,   5501,   5722,   5950,   6188,   6436,   6693,
  6961,   7239,   7529,   7830,   8143,   8469,   8808,   9160,
  9526,   9908,  10304,  10716,  11144,  11590,  12054,  12536,
 13037,  13559,  14101,  14665,  15252,  15862,  16496,  17156,
 17842,  18556,  19298,  20069,  20872,  21707,  22575,  23478,
 24417,  25394,  26409,  27466,  28564,  29706,  30895,  32130,
 33415,  34752,  36142,  37587,  39091,  40654,  42280,  43971,
 45730,  47558,  49461,  51439,  53496,  55636,  57861,  60175,
 62582,  65085,  67688,  70395,  73210,  76138,  79184,  82350,
 85644,  89069,  92632,  96337, 100190, 104197, 108364, 112698,
117205, 121893, 126768, 131838, 137111, 142595, 148298, 154229,
160397, 166812, 173484, 180423, 187639, 195143, 202948, 211065,
219506, 228285, 237416, 246911, 256786, 267056, 277737, 288845,
300397, 312412, 324907, 337901, 351416, 365470, 380087, 395289,
411098, 427540, 444640, 462423, 480917, 500152, 520155, 540958,
562594, 585095, 608496, 632832, 658142, 684465, 711840, 740309,
769918, 800711, 832735, 866040, 900677, 936700, 974163, 1013124,
1053644, 1095784, 1139610, 1185188, 1232589, 1281887, 1333155, 1386475,
1441926, 1499596, 1559572, 1621947, 1686816, 1754280, 1824442, 1897410,
1973297, 2052218, 2134296, 2219657, 2308432, 2400757, 2496775, 2596633,
2700484, 2808490, 2920815, 3037632, 3159121, 3285470, 3416871, 3553528,
3695651, 3843458, 3997176, 4157042, 4323302, 4496211, 4676036, 4863053,
};

//u1 range [-5 ~ 5]. u2 range[-1, 1],  k=32768
const int16_t sigmoid_lut[] = {
-32328, -32310, -32292, -32274, -32254, -32234, -32213, -32191,
-32168, -32144, -32119, -32094, -32067, -32039, -32011, -31981,
-31950, -31918, -31884, -31849, -31813, -31776, -31737, -31696,
-31654, -31610, -31565, -31518, -31469, -31418, -31365, -31310,
-31253, -31194, -31133, -31069, -31003, -30935, -30864, -30790,
-30713, -30634, -30552, -30466, -30377, -30285, -30190, -30091,
-29989, -29883, -29773, -29658, -29540, -29418, -29291, -29160,
-29024, -28883, -28737, -28586, -28430, -28269, -28101, -27929,
-27750, -27565, -27374, -27177, -26973, -26763, -26546, -26321,
-26090, -25851, -25604, -25350, -25089, -24819, -24541, -24255,
-23960, -23657, -23345, -23024, -22694, -22355, -22007, -21650,
-21283, -20907, -20521, -20126, -19721, -19306, -18882, -18448,
-18005, -17551, -17088, -16616, -16134, -15642, -15142, -14632,
-14113, -13585, -13049, -12504, -11951, -11390, -10822, -10246,
 -9662,  -9072,  -8476,  -7874,  -7265,  -6652,  -6034,  -5411,
 -4784,  -4153,  -3520,  -2883,  -2245,  -1604,   -963,   -321,
   321,    963,   1604,   2245,   2883,   3520,   4153,   4784,
  5411,   6034,   6652,   7265,   7874,   8476,   9072,   9662,
 10246,  10822,  11390,  11951,  12504,  13049,  13585,  14113,
 14632,  15142,  15642,  16134,  16616,  17088,  17551,  18005,
 18448,  18882,  19306,  19721,  20126,  20521,  20907,  21283,
 21650,  22007,  22355,  22694,  23024,  23345,  23657,  23960,
 24255,  24541,  24819,  25089,  25350,  25604,  25851,  26090,
 26321,  26546,  26763,  26973,  27177,  27374,  27565,  27750,
 27929,  28101,  28269,  28430,  28586,  28737,  28883,  29024, 
 29160,  29291,  29418,  29540,  29658,  29773,  29883,  29989,
 30091,  30190,  30285,  30377,  30466,  30552,  30634,  30713,
 30790,  30864,  30935,  31003,  31069,  31133,  31194,  31253,
 31310,  31365,  31418,  31469,  31518,  31565,  31610,  31654,
 31696,  31737,  31776,  31813,  31849,  31884,  31918,  31950,
 31981,  32011,  32039,  32067,  32094,  32119,  32144,  32168,
 32191,  32213,  32234,  32254,  32274,  32292,  32310,  32328,
};

int16_t inline sigmoid_q15(int32_t u1) {
  if (u1 > 32767 * 5) {
    return 1 * 32767;
  }
  else if (u1 < -32767 * 5) {
    return -1 * 32767;
  }

  u1 /= 5; //scale to [-32767 32767]
  uint8_t idx = u1 / 256 + 127;
  return sigmoid_lut[idx];
} 

float inline sigmoid_float(float u1) {
  float expx = expf(-u1);

  return 2.0f / ( 1 + expx) - 1;
} 

float inline sigmoid_fake_float(float u1) {
#if 1
  const float a = 0.2f;
  const float bound = (1/a);
  if (u1 > bound ) {
    return 1;
  } else if (u1 < -bound ) {
    return -1;
  }

  return a * u1;
#else
  if (u1 > 1) {
    return 1;
  } else if (u1 < -1) {
    return -1;
  }

  return u1;
#endif
} 

int16_t inline sin_q15(uint16_t u1) {
  return sin_lut[u1 >> 8];
}

int16_t inline cos_q15(uint16_t u1) {
  return cos_lut[u1 >> 8];
}


const int16_t sin_cos_table[] = {
0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE,
};


trig_value_t calc_sin_cos(int16_t u1, int16_t *sinx, int16_t *cosx)
{
  uint16_t index;
  trig_value_t local_trig;
  /* 10 bit index computation  */  
  index = (uint16_t)u1 >> 6;  // /= 64
  
  switch (index & 0x300) 
  {
  case 0x000: //0-90
    local_trig.sinx = sin_cos_table[(uint8_t)index];
    local_trig.cosx = sin_cos_table[0xFF-(uint8_t)index];
    break;
  
  case 0x100:  
     local_trig.sinx = sin_cos_table[0xFF-(uint8_t)index];
     local_trig.cosx = -sin_cos_table[(uint8_t)index];
    break;
  
  case 0x200:
     local_trig.sinx = -sin_cos_table[(uint8_t)index];
     local_trig.cosx = -sin_cos_table[0xFF - (uint8_t)index];
    break;
  
  case 0x300:
     local_trig.sinx =  -sin_cos_table[0xFF - (uint8_t)index];
     local_trig.cosx =  sin_cos_table[(uint8_t)index ]; 
    break;
  default:
    break;
  }

  *sinx = local_trig.sinx;
  *cosx = local_trig.cosx;
  return (local_trig);
}

/*******************************************************************************
* Function Name  : Clarke Transformation
* Description    : This function transforms stator currents qIas and  
*                  qIbs (which are directed along axes each displaced by  
*                  120 degrees) into currents qIalpha and qIbeta in a 
*                  stationary qd reference frame.
*                  qIalpha = qIas
*                  qIbeta = (2*Ib+Ia)/sqrt(3)
*   ialfa �� ia ��ˮƽ����??->  ibeta ��ֱ���ϡ�??
*   \ib              �� ibeta
*    \____ia   ==>   |_______ ialfa.
*    /
*   /ic
* fv.Ialfa = fv.ia;
* fv.Ibeta = (fv.ia + 2*fv.ib) * 0.57735026918963f;
* Input          : ia, ib
* Output         : ialfa, ibeta.
* Return         : none.
*******************************************************************************/

void clarke_q15(int16_t ia, 
                int16_t ib,
                int16_t *ialfa,
                int16_t *ibeta)
{
    /* 1/sqrt(3) in q1.15 format=0.5773315*/
    const int16_t divSQRT_3	= 0x49E6;
    //sqrt3/2 in 1.15
    const int16_t qSqrt3div2 = 0x6ED9;
    int32_t qIa_divSQRT3_tmp;
    int32_t qIb_divSQRT3_tmp;
    int32_t sum_tmp;

    // qIalpha = qIas
    *ialfa = ia;

    qIa_divSQRT3_tmp = divSQRT_3 * ia; 
    qIa_divSQRT3_tmp /=32768;

    qIb_divSQRT3_tmp = divSQRT_3 * ib;
    qIb_divSQRT3_tmp /=32768;

    //so far everything is good.
    //qIbeta = (2*qIbs+qIas)/sqrt(3)  //stԭ����-
    //3��q15��ӣ� �п�������ģ���sum_tmp
    sum_tmp = qIa_divSQRT3_tmp + qIb_divSQRT3_tmp + qIb_divSQRT3_tmp;
    if (sum_tmp > 32767) {
        sum_tmp = 32767;
    }
    else if (sum_tmp < -32767) {
        sum_tmp = -32767;
    }

    *ibeta = sum_tmp;
}

/*******************************************************************************
* Function Name  : Park Transformation
* Description    : This function transforms stator currents qIalpha and qIbeta,
*                  which belong to a stationary qd reference frame, to a rotor 
*                  flux synchronous reference frame (properly oriented), so as 
*                  to obtain qIq and qIds.
* modified langgo 	
* fv.id = fv.cos_theta * fv.Ialfa + fv.sin_theta * fv.Ibeta;
* fv.iq = fv.cos_theta * fv.Ibeta - fv.sin_theta * fv.Ialfa;     
* Input          : alfa, beta, theta.
* Output         : id, iq.
* Return         : none.
*******************************************************************************/
static int16_t park_sinx;   //used for rev park fast calc.
static int16_t park_cosx;   //used for rev park fast calc.

void park_q15(  int16_t ialfa, 
                int16_t ibeta, 
                uint16_t theta,
                int16_t *id,
                int16_t *iq)
{
  int32_t id_tmp1, id_tmp2;
  int32_t iq_tmp1, iq_tmp2; 
  int16_t sinx = sin_q15(theta);
  int16_t cosx = cos_q15(theta);

  //No overflow guaranteed
  id_tmp1 = ialfa * cosx;
  id_tmp1 /= 32768;
  
  //No overflow guaranteed
  id_tmp2 = ibeta * sinx;
  id_tmp2 /= 32768;
  
   //Id component in Q1.15 Format   
  *id = (int16_t)id_tmp1 + (int16_t)id_tmp2;
  
  //No overflow guaranteed
  iq_tmp1 = ibeta * cosx;  	
  iq_tmp1 /= 32768;
  
  //No overflow guaranteed
  iq_tmp2 = ialfa * sinx;
  iq_tmp2 /= 32768;
 
  //Iq component in Q1.15 Format 
  *iq = (int16_t)iq_tmp1 - (int16_t)iq_tmp2;
}

/*******************************************************************************
* Function Name  : Rev_Park Transformation
* Description    : This function transforms stator voltage qVq and qVd, that 
*                  belong to a rotor flux synchronous rotating frame, to a 
*                 stationary reference frame, so as to obtain qValpha and qVbeta                 
* Input          : vq, vd, theta.
* Output         : valfa, vbeta
* modified langgo:  
    fv.Ualfa = -fv.sin_theta * fv.vq + fv.cos_theta * fv.vd ;
    fv.Ubeta = fv.cos_theta * fv.vq + fv.sin_theta * fv.vd ;
* Return         : none.
*******************************************************************************/

void inverse_park_q15(  int16_t vq, 
                        int16_t vd, 
                        uint16_t theta,
                        int16_t *valfa, 
                        int16_t *vbeta)
{ 	
  int32_t valfa1,valfa2,vbeta1,vbeta2;

  int16_t sinx = sin_q15(theta);
  int16_t cosx = cos_q15(theta);

  //No overflow guaranteed  input q d
  valfa1 = -vq * sinx;
  valfa1 /= 32768;
  
  valfa2 = vd * cosx;
  valfa2 /= 32768;

  *valfa = (int16_t)valfa1 + (int16_t)(valfa2);
  
  vbeta1 = vq * cosx;
  vbeta1 /= 32768;
  
  vbeta2 = vd * sinx;
  vbeta2 /= 32768;
		
  *vbeta = (int16_t)vbeta1 + (int16_t)vbeta2;
}

//ta tb tc range[-1, 1] => alfa*[-1, 1]
void svpwm_q15(int16_t valfa, int16_t vbeta, int16_t tabc[])
{
    int tmp1,tmp2,tmp3;
    int sector;
    //disk judgement
    //sqrt(3)/2 = 0.866
    //0<arctan(U��/ U��) <60 in sector1
    const int16_t qSqrt3div2 = 0x6ED9;
    tmp1 = vbeta;
    //tmp2 = (qSqrt3div2 * valfa + vbeta) / 2 / 32768;
    tmp2 = (qSqrt3div2 * valfa / 32768) + vbeta / 2;
    //t1 = vbeta  ###  t2= 0.866f*fv.Ualfa+0.5f*fv.Ubeta;
    tmp3 = tmp2 - tmp1;
    sector = 3;
    sector = (tmp2 > 0) ? (sector-1) : sector;
    sector = (tmp3 > 0) ? (sector-1) : sector;
    sector = (tmp1 < 0) ? (7-sector) : sector;

    switch (sector)
    {
    case 1:
    case 4:
        tabc[0] = tmp2;
        tabc[1] = tmp1 - tmp3;
        tabc[2] = -tmp2;
        break;
    case 2:
    case 5:
        tabc[0] = tmp3 + tmp2;
        tabc[1] = tmp1;
        tabc[2] = -tmp1;
        break;
    case 3:
    case 6:
        tabc[0] = tmp3;
        tabc[1] = -tmp3;
        tabc[2] = -(tmp1 + tmp2);
        break;
    default:
        tabc[0] = 0;
        tabc[1] = 0;
        tabc[2] = 0;
        break;
    }
}

void svpwm_float(float valfa, float vbeta, float tabc[], uint16_t pwm_full, uint16_t ccr[])
{
  const float sqrt3div2 = 0.866f;
  int n, a, b, c, sector;
  float u1 = vbeta;
  float u2 = sqrt3div2 * valfa - 0.5f * vbeta;
  float u3 = -sqrt3div2 * valfa - 0.5f * vbeta;
  float t0,t7;
  float t1,t2,t3,t4,t5,t6;  //sector1, use t4, t6.
  float tx, ty; //1st vector time. 2rd vector time.
  float ta, tb, tc;
  // const float Ts = 1; //10^-4s ������svpwmͬ���� ���Դ˴�=1
  // const float Udc = 100;
  // float m = sqrt3div2 * Ts / Udc; //10^-6

  a = u1 > 0 ? 1 : 0;
  b = u2 > 0 ? 1 : 0;
  c = u3 > 0 ? 1 : 0;
  n = 4 * c + 2 * b + a;

  switch (n) {
    case 3: sector = 1; break;
    case 1: sector = 2; break;
    case 5: sector = 3; break;
    case 4: sector = 4; break;
    case 6: sector = 5; break;
    case 2: sector = 6; break;
    default: sector = 1; break;
  }

  switch (sector) {
    case 1: //0467
    {
      tx = t4 = u2;
      ty = t6 = u1;
      t0 = 0.5f*(1-tx-ty);
      tabc[0] = t0;
      tabc[1] = t0 + tx;
      tabc[2] = t0 + tx + ty;
    }
    break;

    case 2: //0267
    {
      ty = t6 = -u3;
      tx = t2 = -u2;
      t0 = 0.5f*(1-tx-ty);
      tabc[1] = t0;
      tabc[0] = t0 + tx;
      tabc[2] = t0 + tx + ty;
    }
    break;

    case 3: //0237
    {
      tx = t2 = u1;
      ty = t3 = u3;
      t0 = 0.5f*(1-tx-ty);
      tabc[1] = t0;
      tabc[2] = t0 + tx;
      tabc[0] = t0 + tx + ty;
    }
    break;

    case 4: //0137
    {
      ty = t3 = -u2;
      tx = t1 = -u1;
      t0 = 0.5f*(1-tx-ty);
      tabc[2] = t0;
      tabc[1] = t0 + tx;
      tabc[0] = t0 + tx + ty;
    }
    break;

    case 5: //0157
    {
      tx = t1 = u3;
      ty = t5 = u2;
      t0 = 0.5f*(1-tx-ty);
      tabc[2] = t0;
      tabc[0] = t0 + tx;
      tabc[1] = t0 + tx + ty;
    }
    break;

    case 6: //0457
    {
      ty = t5 = -u1;
      tx = t4 = -u3;
      t0 = 0.5f*(1-tx-ty);
      tabc[0] = t0;
      tabc[2] = t0 + tx;
      tabc[1] = t0 + tx + ty;
    }
    break;
  }

  // t0 = 0.5f*(1-tx-ty);
  // tabc[0] = t0;
  // tabc[1] = t0 + tx;
  // tabc[2] = t0 + tx + ty;
  ccr[0] = pwm_full * tabc[0];
  ccr[1] = pwm_full * tabc[1];
  ccr[2] = pwm_full * tabc[2];
}

int16_t slide_filter_int16(int16_t cur, 
                          uint8_t *idx, 
                          int16_t fil_buffer[], 
                          uint8_t len)
{
    int sum = 0;

    fil_buffer[*idx % len] = cur;
    (*idx) ++;

    for (int i = 0; i < len; ++i)
    {
        sum += fil_buffer[i];
    }
    return sum / len;
}

int max_of_3(int a, int b, int c)
{
  int tmp = MAX(a, b);
  return MAX(tmp, c);
  // int max3 = MAX(tmp, c);
  // return max3;
}

int min_of_3(int a, int b, int c)
{
  int tmp = MIN(a, b);
  return MIN(tmp, c);
  // int max3 = MAX(tmp, c);
  // return max3;
}

void clarke_float(float ia, 
                  float ib,
                  float *ialfa,
                  float *ibeta)
{
  
  *ialfa = ia;
  *ibeta = (2*ib + ia) / sqrtf(3);
}

// * fv.id = fv.cos_theta * fv.Ialfa + fv.sin_theta * fv.Ibeta;
// * fv.iq = fv.cos_theta * fv.Ibeta - fv.sin_theta * fv.Ialfa;     
void park_float(float ialfa, float ibeta, float theta, float *id, float *iq)
{
  *id = ialfa * cosf(theta) + ibeta * sinf(theta);
  *iq = -ialfa * sinf(theta) + ibeta * cosf(theta); 
}


// fv.Ualfa = -fv.sin_theta * fv.vq + fv.cos_theta * fv.vd ;
// fv.Ubeta = fv.cos_theta * fv.vq + fv.sin_theta * fv.vd ;
void inverse_park_float(float vd, 
                        float vq, 
                        float theta,
                        float *valfa, 
                        float *vbeta)
{
  *valfa = vd*cosf(theta) - vq * sinf(theta);
  *vbeta = vd*sinf(theta) + vq * cosf(theta);
}

#include "log.h"
static volatile fuck_bug;
int32_t calc_theta_delta_raw(uint16_t *last, uint16_t cur, const int cnt_full_range)
{
    static int delta, res1, res2;
    
    // #define ABS(x) (((x) > 0) ? (x) : -(x))
    
    //����������� �������߷�תԽ��
    if (cur <= *last)
    {
        res1 = cur + cnt_full_range - *last; //+
        res2 = cur - *last;                  //-
    }
    else
    {                                        //cur > last
        res1 = cur - *last;                  //+
        res2 = cur - cnt_full_range - *last; //-
    }

    //��������ת�� ת���ĽǶ�С���Ǹ������
    if (ABS(res1) < ABS(res2))
    {
        delta = res1;
    }
    else
    {
        delta = res2;
    }
    
    //update last.
    // *last = cur;

    return delta;
}

//call example : ref = speed_ramp(cur, set); called freq = 1000 hz.  //1s done.
int16_t foc_speed_ref_ramp(int16_t cur, int16_t target)
{
    if (cur == target)
    {
        return cur;
    }
    int delta = target - cur;
    if (delta > 0)
    {
        return cur + 1;
    }
    else
    {
        return cur - 1;
    }
}

#define RAMP_DELTA  (0.2f)
float foc_speed_ref_ramp_float(float cur, float target)
{
    if ( ABS( cur - target ) < 1.1f )
    {
        return cur;
    }
    float delta = target - cur;
    if (delta > 0)
    {
        return cur + RAMP_DELTA;
    }
    else
    {
        return cur - RAMP_DELTA;
    }
}


void lpf_1rd_init(lpf1_t *lpf1rd, float tc, float z)
{
    // lpf_1rd_reset(lpf1rd);
    lpf1rd->z1 = 0;
    lpf1rd->tc = 0;
    lpf1rd->in = 0;
    lpf1rd->out = 0;
    lpf1rd->tc = tc;
}

float lpf_1rd_calc(lpf1_t *lpf1rd, float new_sample)
{
    lpf1rd->in =  new_sample;
    lpf1rd->out = lpf1rd->z1 + lpf1rd->tc * (lpf1rd->in - lpf1rd->z1);
    lpf1rd->z1 =  lpf1rd->out;

    return lpf1rd->out;
}
/*******************************************
* Function Name : Value_Limit
* Description   : ���������Сֵ
* Input         : None
* Output        : return
* Author        : Ming
* Date          : 2021��9��3��11:18:49
* Notes         : None
*******************************************/
int16_t Value_Limit(int32_t x, int16_t MIN, int16_t MAX)
{
  if (x>MAX) return MAX;
  if (x<MIN) return MIN;
  return x;
}