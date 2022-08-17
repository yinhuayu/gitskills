#include "fdcan.h"

void langgo_can_init(uint32_t can_clk);
int can_send(uint16_t stdid, uint8_t data[], uint8_t len);
void set_moto_speed(FDCAN_HandleTypeDef* hcan, uint16_t stdid, int16_t v[]);
