#include "hw/phobri_proto.h"
#include "zenith/utilities/running_avg.h"
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <stdint.h>

#define CALIB_AVG_LEN_BITS 6
#define NORMAL_AVG_LEN_BITS 4

running_avg_t x_avg;
running_avg_t y_avg;

uint16_t __time_critical_func(read_ext_adc)(bool is_x_axis) {
    const uint8_t config_val = (!is_x_axis) ? 0xD0 : 0xF0;
    uint8_t data_buf[3];
    gpio_put(STICK_SPI_CS, 0);

    spi_read_blocking(STICK_SPI_INTF, config_val, data_buf, 3);
    uint16_t tempValue =
        ((data_buf[0] & 0x07) << 9) | data_buf[1] << 1 | data_buf[2] >> 7;

    gpio_put(STICK_SPI_CS, 1);
    return tempValue;
}

inline void phobri_proto_core1_init(void) {
    spi_init(STICK_SPI_INTF, 3000 * 1000);
    gpio_set_function(STICK_SPI_CLK, GPIO_FUNC_SPI);
    gpio_set_function(STICK_SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(STICK_SPI_RX, GPIO_FUNC_SPI);

    gpio_init(STICK_SPI_CS);
    gpio_set_dir(STICK_SPI_CS, GPIO_OUT);
    gpio_put(STICK_SPI_CS, true); // active low

    init_running_avg(&x_avg, CALIB_AVG_LEN_BITS, NORMAL_AVG_LEN_BITS);
    init_running_avg(&y_avg, CALIB_AVG_LEN_BITS, NORMAL_AVG_LEN_BITS);
}

inline void phobri_proto_read_analog(analog_data_t *analog_data) {
    analog_data->ax1 = UINT_N_TO_AX(
        (uint16_t)(x_avg.running_sum_small >> NORMAL_AVG_LEN_BITS), 12);
    analog_data->ax2 = UINT_N_TO_AX(
        (uint16_t)(y_avg.running_sum_small >> NORMAL_AVG_LEN_BITS), 12);
}

// This actually updates the analog signal
// read_analog is simply used to read from our circular buffer
inline void phobri_proto_core1_inject(analog_data_t *analog_data) {
    uint16_t raw_x = read_ext_adc(true);
    uint16_t raw_y = read_ext_adc(false);
    update_running_avg(&x_avg, raw_x);
    update_running_avg(&y_avg, raw_y);
}