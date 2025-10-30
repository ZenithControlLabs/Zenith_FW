#ifndef __MENU_H
#define __MENU_H

#include "stick_types.h"
#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef enum {
    DIR_NEUTRAL = 0b000,
    DIR_UNKNOWN = 0b001,
    DIR_UP      = 0b100,
    DIR_DOWN    = 0b101,
    DIR_LEFT    = 0b110,
    DIR_RIGHT   = 0b111
} stick_menu_direction_t;

bool get_menu_btn_press();
stick_menu_direction_t stick_val_to_menu_dir(analog_data_t *in);
void factory_init(ADC_HandleTypeDef *hadc);
bool menu_process(analog_data_t *raw, analog_data_t *cal);

#endif // __MENU_H