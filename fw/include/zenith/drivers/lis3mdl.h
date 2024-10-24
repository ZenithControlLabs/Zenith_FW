#ifndef ZENITH_LIS3MDL_H
#define ZENITH_LIS3MDL_H
#include <stdint.h>

typedef struct {
    int16_t h_x;
    int16_t h_y;
    int16_t h_z;
    int16_t h_temp;
} lis3mdl_reading_t;

void lis3mdl_setup(i2c_inst_t *i2c, uint addr, bool temp_en);

void __time_critical_func(lis3mdl_read)(i2c_inst_t *i2c, uint addr, lis3mdl_reading_t *dest, bool read_temp);

#endif //ZENITH_LIS3MDL_H
