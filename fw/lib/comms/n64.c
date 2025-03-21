#include "zenith/includes.h"

#include "joybus.pio.h"

joybus_port_t port;

// These are shared between RX and TX ISRs.
volatile absolute_time_t _receive_end;
volatile uint8_t _bytes[8];
volatile uint8_t _len;

static n64_input_t _out_buffer = {.stick_x = 0, .stick_y = 0};

static void __not_in_flash_func(rx_fifo_not_empty_handler)(void) {
    uint8_t *resp;
    uint8_t len;
    uint8_t packet = pio_sm_get_blocking((&port)->pio, (&port)->sm);

    if (packet == PROBE || packet == RESET) {
        resp = (uint8_t *)&default_n64_status;
        len = sizeof(n64_status_t);
    } else if (packet == POLL) {
        resp = (uint8_t *)&_out_buffer;
        len = sizeof(n64_input_t);
    } else {
        // don't care. it's not a valid packet, we're not going to treat it as
        // such
        return;
    }

    _receive_end = make_timeout_time_us(4);
    irq_set_enabled(PIO0_IRQ_0,
                    false); // disable RX interrupts. we do not care what
                            // happens on the bus now since we have control.

    // TODO: put a mutex here?
    // since accessing the report struct is not atomic
    for (int i = 0; i < len; i++) {
        _bytes[i] = resp[i];
    }

    _len = len;
    irq_set_enabled(
        PIO0_IRQ_1,
        true); // ok now enable TX interrupts, we need to fill the TX queue
    joybus_program_send_init((&port)->pio, (&port)->sm, (&port)->offset,
                             (&port)->pin, &(&port)->config);
}

static void __not_in_flash_func(tx_fifo_empty_handler)(void) {
    // Can't do anything until our waiting period is up.
    // Yes this blocks the system.
    static int _ind = 0;

    while (!time_reached(_receive_end)) {
        tight_loop_contents();
    }

    uint8_t byte;
    bool stop;
    for (; _ind < _len; _ind++) {

        byte = _bytes[_ind];
        stop = _ind == _len - 1;
        uint32_t data_shifted = (byte << 24) | (stop << 23);
        pio_sm_put_blocking((&port)->pio, (&port)->sm, data_shifted);

        if (pio_sm_is_tx_fifo_full((&port)->pio, (&port)->sm)) {
            // our job here is done
            break;
        }
    }

    if (_ind == _len) {
        // we are officially done transferring, since there is nothing left in
        // our queue (software side)
        _ind = 0;
        irq_set_enabled(PIO0_IRQ_1, false); // disable TX
        irq_set_enabled(PIO0_IRQ_0, true);  // re-enable RX interrupts
    }
}

uint joybus_port_init(joybus_port_t *port, uint pin, PIO pio, int sm,
                      int offset) {
    if (sm < 0) {
        sm = pio_claim_unused_sm(pio, true);
    } else {
        pio_sm_claim(pio, sm);
    }

    if (offset < 0) {
        offset = pio_add_program(pio, &joybus_program);
    }

    port->pin = pin;
    port->pio = pio;
    port->sm = sm;
    port->offset = offset;
    port->config = joybus_program_get_config(pio, sm, offset, pin);

    joybus_program_receive_init(port->pio, port->sm, port->offset, port->pin,
                                &port->config);

    return offset;
}

void joybus_init_comms(void) {
    // We are just going to claim PIO0
    joybus_port_init(&port, ZENITH_SERIAL_PIN, pio0, -1, -1);
    irq_set_enabled(
        PIO0_IRQ_0,
        true); // we want to be interrupted as soon as new byte is ready
    irq_set_enabled(PIO0_IRQ_1, false); // not ready for that yet

    irq_set_exclusive_handler(PIO0_IRQ_0, rx_fifo_not_empty_handler);
    irq_set_exclusive_handler(PIO0_IRQ_1, tx_fifo_empty_handler);
}

void n64_init() { joybus_init_comms(); }

void n64_comms_task(uint32_t timestamp, btn_data_t *buttons,
                    analog_data_t *analog) {
    _out_buffer.button_a = buttons->s.b1;
    _out_buffer.button_b = buttons->s.b2;

    _out_buffer.cpad_up = buttons->s.b3;
    _out_buffer.cpad_down = buttons->s.b4;

    _out_buffer.cpad_left = buttons->s.b5;
    _out_buffer.cpad_right = buttons->s.b6;

    _out_buffer.button_start = buttons->s.b7;

    _out_buffer.button_l = buttons->s.b8;

    _out_buffer.button_r = buttons->s.b9;
    _out_buffer.button_z = buttons->s.b10;

    _out_buffer.stick_x = (int8_t)(analog->ax1 * 127.0);
    _out_buffer.stick_y = (int8_t)(analog->ax2 * 127.0);

    _out_buffer.dpad_down = buttons->s.b11;
    _out_buffer.dpad_left = buttons->s.b12;
    _out_buffer.dpad_right = buttons->s.b13;
    _out_buffer.dpad_up = buttons->s.b14;
}
