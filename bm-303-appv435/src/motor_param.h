

#define POLE_PAIR   (15)
#define VBUS        (24)
#define R_SHUNT     (0.01)  //10mR
#define CURRENT_GAIN    (4.7)
// #define PHASE_RS    (0.0746f) //Ohm 
// #define PHASE_LS    (23e-6)  //Hen
#define PHASE_RS    (3.2f) //Ohm 
#define PHASE_LS    (15e-3)  //Hen
#define MIN_SPEED_RPM   500

//max torque = (VREF / 2) / (Rshunt * gain)
//           = (vdd / 2 ) / 
#define INIT_TORQUE (1) //Amp


/*
台表测试
15.6mH 6.5ohm
19.0mH 8.0ohm = 
11.1mH 4.4ohm
*/