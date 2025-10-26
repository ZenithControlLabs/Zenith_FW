#ifndef ZENITH_SNAPBACK_FILTER_H
#define ZENITH_SNAPBACK_FILTER_H

#include "stick.h"

void snapback_filter(const ax_t x_in, const ax_t y_in, ax_t *x_out, ax_t *y_out, const stick_config_t *stick_config);

#endif // ZENITH_SNAPBACK_FILTER_H