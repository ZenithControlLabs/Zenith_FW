#ifndef __INTF_H
#define __INTF_H 

#include <stdbool.h>
#include <stdint.h>

#include "stick_types.h"
#include "stm32l4xx_hal.h"

enum {
    CHAN_X = 0,
    CHAN_Y = 1,
    CHAN_END
};

void adc_read(ADC_HandleTypeDef *hadc, uint16_t *adc_res, bool apply_pol);
void intf_init(DAC_HandleTypeDef *hdac);
void intf_out(int16_t x, int16_t y);
void intf_out_cal(ax_t x, ax_t y);
bool intf_is_mode_analog();

#endif // __INTF_H