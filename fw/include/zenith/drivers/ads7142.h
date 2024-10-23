#ifndef ZENITH_ADS7142_H
#define ZENITH_ADS7142_H

typedef struct {
    uint16_t a_x;
    uint16_t a_y;
} ads7142_reading_t;

void ads7142_setup(i2c_inst_t *i2c, uint addr);

void __time_critical_func(ads7142_read)(i2c_inst_t *i2c, uint addr, ads7142_reading_t *dest);

#endif //ZENITH_ADS7142_H