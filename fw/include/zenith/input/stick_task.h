#ifndef ZENITH_STICK_TASK_H
#define ZENITH_STICK_TASK_H

#include "zenith/includes.h"

extern volatile _Atomic cal_msg_t _cal_msg;

void stick_task(uint32_t timestamp, analog_data_t *in, analog_data_t *out);


#endif // ZENITH_STICK_TASK_H