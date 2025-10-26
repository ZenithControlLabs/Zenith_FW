#ifndef STICK_H
#define STICK_H

#ifdef DEBUG
#define debug_print(fmt, args...) printf(fmt, ##args)
#else
#define debug_print(fmt, args...)
#endif

#include "stick_types.h"

#include <stdbool.h>
#include <stdint.h>

#define NUM_NOTCHES 8
#define FIT_ORDER 3

typedef struct {
    bool calibrated;
    float fit_coeffs_x[FIT_ORDER + 1];
    float fit_coeffs_y[FIT_ORDER + 1];

    float affine_coeffs[NUM_NOTCHES][4];
    float boundary_angles[NUM_NOTCHES];

    ax_t notch_points_x_in[NUM_NOTCHES];
    ax_t notch_points_y_in[NUM_NOTCHES];
} calib_results_t;

typedef struct {
    ax_t notch_points_x[NUM_NOTCHES];
    ax_t notch_points_y[NUM_NOTCHES];
    float angle_deadzones[NUM_NOTCHES];
    float mag_threshold;
    float cutoff_hz;
} stick_config_t;

#define CALIBRATION_NUM_STEPS NUM_NOTCHES * 2

extern volatile int _cal_step;

typedef enum { CALIB_NONE, CALIB_ADVANCE, CALIB_UNDO } cal_msg_t;

extern float al_g_dt;

void analoglib_init(calib_results_t *settings_calib_results, stick_config_t *settings_stick_config, const float dt);

void analoglib_cal_advance(analog_data_t *in);

void analoglib_cal_undo(void);

void analoglib_process(analog_data_t *in, analog_data_t *out,
                   const bool gate_limiter_enable);

#endif // STICK_H