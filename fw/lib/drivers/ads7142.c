#include <hardware/i2c.h>
#include <stdint.h>

#include "zenith/drivers/ads7142.h"

void ads7142_setup(i2c_inst_t *i2c, uint addr) {
    // Manual mode with Auto-Sequence
    uint8_t opmode_sel_data[] = {
        0x8,
        0x1C,
        0b00000101,
    };

    // Start the sequence
    uint8_t start_sequence_data[] = {
        0x8,
        0x1E,
        0b00000001,
    };

    i2c_write_blocking(i2c, addr, opmode_sel_data, 3, false);
    i2c_write_blocking(i2c, addr, start_sequence_data, 3, false);
}

void __time_critical_func(ads7142_read)(i2c_inst_t *i2c, uint addr, ads7142_reading_t *dest) {
    uint8_t data_buf[4];

    i2c_read_blocking(i2c, addr, data_buf, 4, false);
    dest->a_x = ((data_buf[2] << 8) + data_buf[3]) >> 4;
    dest->a_y = ((data_buf[0] << 8) + data_buf[1]) >> 4;
}