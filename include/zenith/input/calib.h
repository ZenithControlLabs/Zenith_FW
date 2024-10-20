#ifndef ZENITH_CALIB_H
#define ZENITH_CALIB_H

#include "stick.h"

#define CALIBRATION_NUM_STEPS NUM_NOTCHES * 2

extern volatile int _cal_step;

typedef enum { CALIB_NONE, CALIB_ADVANCE, CALIB_UNDO } cal_msg_t;
extern volatile _Atomic cal_msg_t _cal_msg;

void calibration_advance(analog_data_t *in);

void calibration_undo(void);

void calibration_finish(void);

#endif // ZENITH_CALIB_H