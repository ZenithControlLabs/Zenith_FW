#include "hw/phobri_v1_X.h"
#include "zenith/types.h"
#include "zenith/utilities/running_avg.h"
#include "zenith/drivers/ads7142.h"
#include <hardware/gpio.h>

#define CALIB_AVG_LEN_BITS 6
#define NORMAL_AVG_LEN_BITS 5

running_avg_t adc_x_avg;
running_avg_t adc_y_avg;

ads7142_reading_t reading;

void phobri_v1_1_analog_core1_init(void) {
    gpio_init(STICK_HALL_DRDY);
    gpio_init(STICK_ADC_DRDY_N);
    gpio_set_dir(STICK_HALL_DRDY, GPIO_IN);
    gpio_set_dir(STICK_ADC_DRDY_N, GPIO_IN);

    i2c_init(STICK_I2C_INTF, 400 * 1000);
    gpio_set_function(STICK_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(STICK_I2C_SDA, GPIO_FUNC_I2C);

    ads7142_setup(STICK_I2C_INTF, I2C_ADC_ADDR);

    init_running_avg(&adc_x_avg, CALIB_AVG_LEN_BITS, NORMAL_AVG_LEN_BITS);
    init_running_avg(&adc_y_avg, CALIB_AVG_LEN_BITS, NORMAL_AVG_LEN_BITS);

    // gpio_set_irq_enabled_with_callback(STICK_ADC_DRDY_N, GPIO_IRQ_EDGE_FALL,
    //                                   true, &ads7142_isr);
}

void phobri_v1_1_analog_read_analog(analog_data_t *analog_data) {
    ads7142_read(STICK_I2C_INTF, I2C_ADC_ADDR, &reading);
    update_running_avg(&adc_x_avg, reading.a_x);
    update_running_avg(&adc_y_avg, reading.a_y);

    analog_data->ax1 = UINT_N_TO_AX(
        (uint16_t)(adc_x_avg.running_sum_small >> NORMAL_AVG_LEN_BITS), 16);
    analog_data->ax2 = UINT_N_TO_AX(
        (uint16_t)(adc_y_avg.running_sum_small >> NORMAL_AVG_LEN_BITS), 16);
}

/*void phobri_v1_1_analog_read_cal(analog_data_t *analog) {
    analog->ax1 = UINT_N_TO_AX(
        (uint16_t)(adc_x_avg.running_sum_large >> CALIB_AVG_LEN_BITS), 12);
    analog->ax2 = UINT_N_TO_AX(
        (uint16_t)(adc_y_avg.running_sum_large >> CALIB_AVG_LEN_BITS), 12);
}*/
