#ifndef ZENITH_LINEARIZE_H
#define ZENITH_LINEARIZE_H

#include "stick.h"

static const double linearize_reference[] = {
    -100 / 127.0, -75 / 127.0, 0 / 127.0, 75 / 127.0, 100 / 127.0};

ax_t linearize(const ax_t point, const float coefficients[]);

void linearize_cal(const ax_t cleaned_points_x[], const ax_t cleaned_points_y[],
                   ax_t linearized_points_x[], ax_t linearized_points_y[],
                   calib_results_t *calib_results);

// See note in linearize.c for attribution
//--------------------------------------------------------
// polyfit()
// Computes polynomial coefficients that best fit a set
// of input points.
//
// Returns 0 if success.
//--------------------------------------------------------
int polyfit(int pointCount, const double *xValues, const double *yValues,
            int coefficientCount, double *coefficientResults);

#endif // ZENITH_LINEARIZE_H