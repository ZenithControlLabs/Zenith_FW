#include "hw/phobri_v1_X.h"
#include "zenith/types.h"
#include "zenith/drivers/lis3mdl.h"
#include <hardware/gpio.h>

#include <stdint.h>

lis3mdl_reading_t reading_hx;
lis3mdl_reading_t reading_hy;

ax_t hx_val; 
ax_t hy_val;

void __time_critical_func(hx_drdy_isr)(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    lis3mdl_read(STICK_I2C_INTF, I2C_HX_ADDR, &reading_hx, false);
}

void __time_critical_func(hy_drdy_isr)(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    lis3mdl_read(STICK_I2C_INTF, I2C_HY_ADDR, &reading_hy, false);
}

void phobri_v1_0_core1_init(void) {
    gpio_init(STICK_HX_DRDY);
    gpio_init(STICK_HY_DRDY);
    gpio_init(STICK_ADC_DRDY_N);
    gpio_set_dir(STICK_HX_DRDY, GPIO_IN);
    gpio_set_dir(STICK_HY_DRDY, GPIO_IN);
    gpio_set_dir(STICK_ADC_DRDY_N, GPIO_IN);

    i2c_init(STICK_I2C_INTF, 400 * 1000);
    gpio_set_function(STICK_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(STICK_I2C_SDA, GPIO_FUNC_I2C);

    lis3mdl_setup(STICK_I2C_INTF, I2C_HX_ADDR, false);
    lis3mdl_setup(STICK_I2C_INTF, I2C_HY_ADDR, false);

    //gpio_set_irq_enabled_with_callback(STICK_HX_DRDY, GPIO_IRQ_EDGE_RISE, true,
    //                                   &hx_drdy_isr);
    //gpio_set_irq_enabled_with_callback(STICK_HY_DRDY, GPIO_IRQ_EDGE_RISE, true,
    //                                   &hy_drdy_isr);
}

void phobri_v1_0_read_analog(analog_data_t *analog_data) {
    hx_drdy_isr(0,0);
    hy_drdy_isr(0,0);
     
    analog_data->ax1 = INT_N_TO_AX(reading_hx.h_z, 16);
    analog_data->ax2 = INT_N_TO_AX(reading_hy.h_z, 16);
}
