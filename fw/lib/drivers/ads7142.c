#include <hardware/i2c.h>
#include <stdint.h>

#include "zenith/drivers/ads7142.h"

static inline void ads7142_write_reg(i2c_inst_t *i2c, uint addr, uint8_t regaddr, uint8_t regval) {
    uint8_t data[] = {0x8, regaddr, regval};

    i2c_write_blocking(i2c, addr, data, 3, false);
}

void ads7142_setup(i2c_inst_t *i2c, uint addr) {
    ads7142_write_reg(i2c, addr, 0x1F, 0x01);
    ads7142_write_reg(i2c, addr, 0x15, 0x01);
    ads7142_write_reg(i2c, addr, 0x24, 0x03);
    ads7142_write_reg(i2c, addr, 0x1C, 0x07);
    ads7142_write_reg(i2c, addr, 0x20, 0x03);
    ads7142_write_reg(i2c, addr, 0x18, 0x00);
    ads7142_write_reg(i2c, addr, 0x19, 0x15);
    ads7142_write_reg(i2c, addr, 0x30, 0x0F);
    ads7142_write_reg(i2c, addr, 0x1E, 0x01);
}

void __time_critical_func(ads7142_read)(i2c_inst_t *i2c, uint addr, ads7142_reading_t *dest) {
    uint8_t data_buf[4] = {
        0x30,
        0x08,
        0,
        0,
    };

    i2c_write_blocking(i2c, addr, data_buf, 2, false);
    i2c_read_blocking(i2c, addr, data_buf, 4, false);
    dest->a_x = ((data_buf[3] << 8) + data_buf[2]);
    dest->a_y = ((data_buf[1] << 8) + data_buf[0]);
    ads7142_write_reg(i2c, addr, 0x1E, 0x01);
}