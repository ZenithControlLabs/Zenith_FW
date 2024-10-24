#include <hardware/i2c.h>
#include <stdint.h>

#include "zenith/drivers/lis3mdl.h"

void lis3mdl_setup(i2c_inst_t *i2c, uint addr) {
    // Temp disabled, LP mode w/ FAST_ODR @ 1kHZ
    uint8_t ctrl_reg1_cfg[] = {0x20, 0b00000010};
    // Full scale +/- 4G
    // Not needed for now because matches default
    uint8_t ctrl_reg2_cfg[] = {0x21, 0b01100000};
    // Continuous measurement
    uint8_t ctrl_reg3_cfg[] = {0x22, 0b00000000};
    // LP mode on Z-axis as well
    // Not needed for now because matches default
    // uint8_t ctrl_reg4_cfg[] = {0x23, 0b00000000};

    i2c_write_blocking(i2c, addr, ctrl_reg1_cfg, 2, false);
    i2c_write_blocking(i2c, addr, ctrl_reg2_cfg, 2, false);
    i2c_write_blocking(i2c, addr, ctrl_reg3_cfg, 2, false);
}

void __time_critical_func(lis3mdl_read)(i2c_inst_t *i2c, uint addr, lis3mdl_reading_t *dest) {

    uint8_t regl;
    uint8_t regh;
    uint8_t read_buf[2];

    regl = 0x28;
    regh = 0x29;
    i2c_write_blocking(i2c, addr, &regl, 1, true);
    i2c_read_blocking(i2c, addr, read_buf, 1, false);
    i2c_write_blocking(i2c, addr, &regh, 1, true);
    i2c_read_blocking(i2c, addr, (read_buf + 1), 1, false);
    dest->h_x = (read_buf[1] << 8) + read_buf[0];

    regl = 0x2a;
    regh = 0x2b;
    i2c_write_blocking(i2c, addr, &regl, 1, true);
    i2c_read_blocking(i2c, addr, read_buf, 1, false);
    i2c_write_blocking(i2c, addr, &regh, 1, true);
    i2c_read_blocking(i2c, addr, (read_buf + 1), 1, false);
    dest->h_y = (read_buf[1] << 8) + read_buf[0];

    regl = 0x2c;
    regh = 0x2d;
    i2c_write_blocking(i2c, addr, &regl, 1, true);
    i2c_read_blocking(i2c, addr, read_buf, 1, false);
    i2c_write_blocking(i2c, addr, &regh, 1, true);
    i2c_read_blocking(i2c, addr, (read_buf + 1), 1, false);
    dest->h_z = (read_buf[1] << 8) + read_buf[0];
}