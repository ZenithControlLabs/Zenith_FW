#ifndef __INTF_H
#define __INTF_H 

#include <stdbool.h>
#include <stdint.h>

#include "stm32l4xx_hal.h"

void intf_init(DAC_HandleTypeDef *hdac);
void intf_out(int16_t x, int16_t y);
bool intf_is_mode_analog();

#endif // __INTF_H