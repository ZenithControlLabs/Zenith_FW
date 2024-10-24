#include "hw/phobri_v1_X.h"
#include "zenith/types.h"
#include "zenith/drivers/lis3mdl.h"
#include "zenith_cfg.h"
#include <hardware/gpio.h>

#include <stdint.h>

v1_1_datacollect_t dc;

void phobri_v1_1_datacollect_core1_init(void) {
    gpio_init(STICK_HALL_DRDY);
    gpio_init(STICK_ADC_DRDY_N);
    gpio_set_dir(STICK_HALL_DRDY, GPIO_IN);
    gpio_set_dir(STICK_ADC_DRDY_N, GPIO_IN);

    i2c_init(STICK_I2C_INTF, 400 * 1000);
    gpio_set_function(STICK_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(STICK_I2C_SDA, GPIO_FUNC_I2C);

    lis3mdl_setup(STICK_I2C_INTF, I2C_HALL_ADDR, true);
    dc.ax = 0;
    dc.ay = 0;
}

void phobri_v1_1_datacollect_read_analog(analog_data_t *analog_data) {
    lis3mdl_read(STICK_I2C_INTF, I2C_HALL_ADDR, (lis3mdl_reading_t*)&dc, true);
     
    analog_data->ax1 = INT_N_TO_AX(dc.h_x, 16);
    analog_data->ax2 = INT_N_TO_AX(dc.h_y, 16);
}
