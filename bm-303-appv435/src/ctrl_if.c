#include "foc.h"

void foc_align_rotor(char argc, char* argv)
{
    // fv.fsm_state = FSM_STATE_ROTOR_ALIGN;
    // align_start_timestamp = HAL_GetTick();
    // LOGW("rotor align start... continue %dms", align_continues_time);
}

void foc_start_sensor_foc(char argc, char* argv)
{

}



uint8_t button_state;
void foc_button_ctrl()
{
    static uint8_t enable;
    enable = !enable;
    button_state ++;
    button_state %= 3;
    switch (button_state)
    {
        case 0:
            foc_stop(0,0);
        break;

        case 1:
        {
            foc_start_hall_linear(0,0);//hallxy sensor.
        }
        break;

        case 2:
        {
            fv.fsm_state = FSM_STATE_SENSORLESS_CLOSELOOP;
        }
        break;

        default:
        break;
    }
}