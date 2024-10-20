#include "hw/phobri_v1_X.h"
#include "zenith/types.h"
#include "zenith/utilities/running_avg.h"
#include <hardware/gpio.h>

#define CALIB_AVG_LEN_BITS 6
#define NORMAL_AVG_LEN_BITS 5

running_avg_t x_avg;
running_avg_t y_avg;

void ads7142_setup(void) {
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

    i2c_write_blocking(STICK_I2C_INTF, I2C_ADC_ADDR, opmode_sel_data, 3, false);
    i2c_write_blocking(STICK_I2C_INTF, I2C_ADC_ADDR, start_sequence_data, 3,
                       false);
}

void __time_critical_func(ads7142_isr)(uint gpio, uint32_t events) {
    uint8_t data_buf[4];

    i2c_read_blocking(STICK_I2C_INTF, I2C_ADC_ADDR, data_buf, 4, false);
    uint16_t _adc_x_val = ((data_buf[2] << 8) + data_buf[3]) >> 4;
    uint16_t _adc_y_val = ((data_buf[0] << 8) + data_buf[1]) >> 4;
    update_running_avg(&x_avg, _adc_x_val);
    update_running_avg(&y_avg, _adc_y_val);
    // printf("%d %d\n", _adc_x_val, _adc_y_val);
}

void phobri_v1_x_analog_core1_init(void) {
    gpio_init(STICK_HX_DRDY);
    gpio_init(STICK_HY_DRDY);
    gpio_init(STICK_ADC_DRDY_N);
    gpio_set_dir(STICK_HX_DRDY, GPIO_IN);
    gpio_set_dir(STICK_HY_DRDY, GPIO_IN);
    gpio_set_dir(STICK_ADC_DRDY_N, GPIO_IN);

    i2c_init(STICK_I2C_INTF, 400 * 1000);
    gpio_set_function(STICK_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(STICK_I2C_SDA, GPIO_FUNC_I2C);

    ads7142_setup();

    init_running_avg(&x_avg, CALIB_AVG_LEN_BITS, NORMAL_AVG_LEN_BITS);
    init_running_avg(&y_avg, CALIB_AVG_LEN_BITS, NORMAL_AVG_LEN_BITS);

    // gpio_set_irq_enabled_with_callback(STICK_ADC_DRDY_N, GPIO_IRQ_EDGE_FALL,
    //                                   true, &ads7142_isr);
}

void phobri_v1_x_analog_read_analog(analog_data_t *analog_data) {
    ads7142_isr(0, 0);
    analog_data->ax1 = UINT_N_TO_AX(
        (uint16_t)(x_avg.running_sum_small >> NORMAL_AVG_LEN_BITS), 12);
    analog_data->ax2 = UINT_N_TO_AX(
        (uint16_t)(y_avg.running_sum_small >> NORMAL_AVG_LEN_BITS), 12);
}

void phobri_v1_x_analog_read_cal(analog_data_t *analog) {
    analog->ax1 = UINT_N_TO_AX(
        (uint16_t)(x_avg.running_sum_large >> CALIB_AVG_LEN_BITS), 12);
    analog->ax2 = UINT_N_TO_AX(
        (uint16_t)(y_avg.running_sum_large >> CALIB_AVG_LEN_BITS), 12);
}