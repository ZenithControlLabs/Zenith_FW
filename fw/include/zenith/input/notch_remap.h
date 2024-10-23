#ifndef ZENITH_NOTCH_REMAP_H
#define ZENITH_NOTCH_REMAP_H

#include "zenith/input/stick.h"

void notch_remap(const ax_t x_in, const ax_t y_in, ax_t *x_out, ax_t *y_out,
                 const calib_results_t *calib_results,
                 const stick_config_t *stick_config);

void notch_calibrate(const ax_t in_points_x[], const ax_t in_points_y[],
                     const ax_t notch_points_x[], const ax_t notch_points_y[],
                     calib_results_t *calib_results);

#endif // ZENITH_NOTCH_REMAP_H