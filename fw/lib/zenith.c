#include "zenith/includes.h"

btn_data_t _buttons = {0};
btn_data_t _buttons_processed = {0};

analog_data_t _analog_data = {0};
analog_data_t _analog_data_processed = {0};

volatile _Atomic uint32_t _timestamp = 0;

void zenith_loop_core0(void) {
    for (;;) {
        atomic_store(&_timestamp, time_us_32());

        cb_zenith_read_buttons(&_buttons);
        btn_remap_task(&_buttons, &_buttons_processed);

        usb_task(atomic_load(&_timestamp), &_buttons_processed,
                 &_analog_data_processed, &_analog_data);

        comms_task(atomic_load(&_timestamp), &_buttons_processed,
                   &_analog_data_processed);

        cb_zenith_core0_inject();
    }
}

void zenith_loop_core1(void) {
    cb_zenith_core1_init();

    for (;;) {
        settings_core1_handle_commit();

        stick_task(atomic_load(&_timestamp), &_analog_data,
                   &_analog_data_processed);

        cb_zenith_core1_inject();
    }
}

void zenith_start() {
    settings_load();

    cb_zenith_init_hardware();

    usb_init();

    comms_init();

    multicore_lockout_victim_init();

    multicore_launch_core1(zenith_loop_core1);

    zenith_loop_core0();
}