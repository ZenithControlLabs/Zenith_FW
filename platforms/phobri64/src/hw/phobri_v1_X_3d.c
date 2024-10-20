#include "hw/phobri_v1_X.h"
#include "zenith/types.h"
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <stdint.h>

// #define SIGNAL_CIRC_BUF_LEN_BITS 5
// #define SIGNAL_CIRC_BUF_LEN (1 << SIGNAL_CIRC_BUF_LEN_BITS)

// TODO: expand this with a circular buffer,
// storing older data and perhaps other axes
ax_t _hx_val = 0;
ax_t _hy_val = 0;

void lis3mdl_setup(uint i2c_addr) {
    // Temp disabled, LP mode w/ FAST_ODR @ 1kHZ
    uint8_t ctrl_reg1_cfg[] = {0x20, 0b00000010};
    // Full scale +/- 4G
    // Not needed for now because matches default
    // uint8_t ctrl_reg2_cfg[] = {0x21, 0b00000000};
    // Continuous measurement
    uint8_t ctrl_reg3_cfg[] = {0x22, 0b00000000};
    // LP mode on Z-axis as well
    // Not needed for now because matches default
    // uint8_t ctrl_reg4_cfg[] = {0x23, 0b00000000};

    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, ctrl_reg1_cfg, 2, false);
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, ctrl_reg3_cfg, 2, false);
}

void __time_critical_func(lis3mdl_read)(float *dest, uint i2c_addr) {

    uint8_t regl;
    uint8_t regh;
    uint8_t read_buf[2];
    uint32_t xval;
    uint32_t yval;
    uint32_t zval;

    regl = 0x28;
    regh = 0x29;
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, &regl, 1, true);
    i2c_read_blocking(STICK_I2C_INTF, i2c_addr, read_buf, 1, false);
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, &regh, 1, true);
    i2c_read_blocking(STICK_I2C_INTF, i2c_addr, (read_buf + 1), 1, false);
    xval = (read_buf[1] << 8) + read_buf[0];

    // not using these for now, only reading the X-axis

    regl = 0x2a;
    regh = 0x2b;
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, &regl, 1, true);
    i2c_read_blocking(STICK_I2C_INTF, i2c_addr, read_buf, 1, false);
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, &regh, 1, true);
    i2c_read_blocking(STICK_I2C_INTF, i2c_addr, (read_buf + 1), 1, false);
    yval = (read_buf[1] << 8) + read_buf[0];

    regl = 0x2c;
    regh = 0x2d;
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, &regl, 1, true);
    i2c_read_blocking(STICK_I2C_INTF, i2c_addr, read_buf, 1, false);
    i2c_write_blocking(STICK_I2C_INTF, i2c_addr, &regh, 1, true);
    i2c_read_blocking(STICK_I2C_INTF, i2c_addr, (read_buf + 1), 1, false);
    zval = (read_buf[1] << 8) + read_buf[0];

    *dest = sqrtf((float)((xval * xval) + (yval * yval) + (zval * zval)));
}

void __time_critical_func(hx_drdy_isr)(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    lis3mdl_read(&_hx_val, I2C_HX_ADDR);
}

void __time_critical_func(hy_drdy_isr)(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    lis3mdl_read(&_hy_val, I2C_HY_ADDR);
}

void phobri_v1_x_3d_core1_init(void) {
    gpio_init(STICK_HX_DRDY);
    gpio_init(STICK_HY_DRDY);
    gpio_init(STICK_ADC_DRDY_N);
    gpio_set_dir(STICK_HX_DRDY, GPIO_IN);
    gpio_set_dir(STICK_HY_DRDY, GPIO_IN);
    gpio_set_dir(STICK_ADC_DRDY_N, GPIO_IN);

    i2c_init(STICK_I2C_INTF, 400 * 1000);
    gpio_set_function(STICK_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(STICK_I2C_SDA, GPIO_FUNC_I2C);

    lis3mdl_setup(I2C_HY_ADDR);
    lis3mdl_setup(I2C_HX_ADDR);

    gpio_set_irq_enabled_with_callback(STICK_HX_DRDY, GPIO_IRQ_EDGE_RISE, true,
                                       &hx_drdy_isr);
    gpio_set_irq_enabled_with_callback(STICK_HY_DRDY, GPIO_IRQ_EDGE_RISE, true,
                                       &hy_drdy_isr);
}

void phobri_v1_x_3d_read_analog(analog_data_t *analog_data) {
    analog_data->ax1 = _hx_val;
    analog_data->ax2 = _hy_val;
}