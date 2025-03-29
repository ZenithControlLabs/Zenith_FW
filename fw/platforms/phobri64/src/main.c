#include "main.h"

void setup_gpio_button(uint8_t gpio) {
    gpio_init(gpio);
    gpio_pull_up(gpio);
    gpio_set_dir(gpio, GPIO_IN);
}

void cb_zenith_init_hardware(void) {
    setup_gpio_button(BTN_A_PIN);
    setup_gpio_button(BTN_B_PIN);
    setup_gpio_button(BTN_START_PIN);
    setup_gpio_button(BTN_ZR_PIN);
    setup_gpio_button(BTN_ZL_PIN);
    setup_gpio_button(BTN_R_PIN);
    setup_gpio_button(BTN_L_PIN);
    setup_gpio_button(BTN_CU_PIN);
    setup_gpio_button(BTN_CD_PIN);
    setup_gpio_button(BTN_CL_PIN);
    setup_gpio_button(BTN_CR_PIN);
    setup_gpio_button(BTN_DU_PIN);
    setup_gpio_button(BTN_DD_PIN);
    setup_gpio_button(BTN_DL_PIN);
    setup_gpio_button(BTN_DR_PIN);
}
void cb_zenith_read_buttons(btn_data_t *buttons) {
    buttons->s.b1 = !gpio_get(BTN_A_PIN);
    buttons->s.b2 = !gpio_get(BTN_B_PIN);
    buttons->s.b3 = !gpio_get(BTN_CU_PIN);
    buttons->s.b4 = !gpio_get(BTN_CD_PIN);
    buttons->s.b5 = !gpio_get(BTN_CL_PIN);
    buttons->s.b6 = !gpio_get(BTN_CR_PIN);
    buttons->s.b7 = !gpio_get(BTN_START_PIN);
    buttons->s.b8 = !gpio_get(BTN_L_PIN);
    buttons->s.b9 = !gpio_get(BTN_R_PIN);
    buttons->s.b10 = !gpio_get(BTN_ZL_PIN);
    buttons->s.b11 = !gpio_get(BTN_DD_PIN);
    buttons->s.b12 = !gpio_get(BTN_DL_PIN);
    buttons->s.b13 = !gpio_get(BTN_DR_PIN);
    buttons->s.b14 = !gpio_get(BTN_DU_PIN);
    buttons->s.b15 = !gpio_get(BTN_ZR_PIN);
}

void cb_zenith_read_analog(analog_data_t *analog_data) {
    HW_READ_ANALOG(analog_data);
}

void cb_zenith_core1_init(void) { HW_CORE1_INIT(); }
void cb_zenith_core0_inject(void) {
#ifdef DEBUG
    if (!gpio_get(BTN_START_PIN) && !gpio_get(BTN_ZL_PIN) &&
        !gpio_get(BTN_ZR_PIN)) {
        reset_usb_boot(0, 0);
    }

    if (!gpio_get(BTN_START_PIN) && !gpio_get(BTN_L_PIN) &&
        !gpio_get(BTN_R_PIN)) {
        watchdog_reboot(0, 0, 0);
    }
#endif
}
void cb_zenith_core1_inject(void) {
#ifdef HW_PHOBRI_PROTO
    phobri_proto_core1_inject();
#endif
}

bool cb_zenith_user_webusb_cmd(uint8_t *in, uint8_t *out) { return false; }

void cb_zenith_user_settings_reset(uint8_t *data) {}

int main() {
    adc_run(0);
    set_sys_clock_khz(130000, true);

    stdio_uart_init_full(DEBUG_UART, 115200, DEBUG_TX_PIN, -1);

    printf("Phobri64 Started.\n");

    setup_gpio_button(BTN_START_PIN);
    gpio_set_function(BTN_START_PIN, GPIO_FUNC_SIO);
    // Handle bootloader stuff
    if (!gpio_get(BTN_START_PIN)) {
        reset_usb_boot(0, 0);
    }

    zenith_start();
}
