#ifndef __MENU_H
#define __MENU_H

#include "stick_types.h"
#include <stdbool.h>

void menu_process(analog_data_t *in, analog_data_t *out, bool btn_press);

#endif // __MENU_H