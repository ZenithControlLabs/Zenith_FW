#include "joybus_cons.h"
#include "joybus.pio.h"
#include <pico/time.h>
#include <stdio.h>

#include "zenith/includes.h"

joybus_port_t cntlr_port;

uint joybus_cons_port_init(joybus_port_t *port, uint pin, PIO pio, int sm,
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

    joybus_program_send_init(port->pio, port->sm, port->offset, port->pin,
                             &port->config);

    return offset;
}

uint joybus_cons_port_change_pin(joybus_port_t *port, uint pin) {
    port->pin = pin;
    port->config =
        joybus_program_get_config(port->pio, port->sm, port->offset, pin);

    joybus_program_send_init(port->pio, port->sm, port->offset, port->pin,
                             &port->config);

    return port->offset;
}

void init_joybus() {
    // We are just going to claim PIO1
    joybus_cons_port_init(&cntlr_port, JOYBUS_CTLR0, pio1, -1, -1);
    gpio_pull_up(JOYBUS_CTLR0);

#ifdef SEND_PROBE
    // Send status byte
    uint32_t data_shifted = (PROBE << 24) | (1 << 23);
    // printf("putting %d\n", data_shifted);

    bool got_response = false;
    while (!got_response) {
        pio_sm_put_blocking((&cntlr_port)->pio, (&cntlr_port)->sm,
                            data_shifted);
        sleep_us(50);
        if (pio_sm_is_rx_fifo_empty((&cntlr_port)->pio, (&cntlr_port)->sm)) {
            sleep_ms(1);
            joybus_program_send_init((&cntlr_port)->pio, (&cntlr_port)->sm,
                                     (&cntlr_port)->offset, (&cntlr_port)->pin,
                                     &(&cntlr_port)->config);
            continue;
        }
        int byte_cnt = 0;
        while (byte_cnt < 3) {
            uint32_t d =
                pio_sm_get_blocking((&cntlr_port)->pio, (&cntlr_port)->sm);
            byte_cnt++;
        }
        got_response = true;
    }
#endif

    joybus_program_send_init((&cntlr_port)->pio, (&cntlr_port)->sm,
                             (&cntlr_port)->offset, (&cntlr_port)->pin,
                             &(&cntlr_port)->config);

    return;
}

void change_joybus_index(int index) {
    uint pin = JOYBUS_CTLR0;
    if (index == 1)
        pin = JOYBUS_CTLR1;
    joybus_cons_port_change_pin(&cntlr_port, pin);
    gpio_pull_up(pin);

    joybus_program_send_init((&cntlr_port)->pio, (&cntlr_port)->sm,
                             (&cntlr_port)->offset, (&cntlr_port)->pin,
                             &(&cntlr_port)->config);
}

uint32_t read_joybus_ctlr() {
    uint32_t data_shifted = (POLL << 24) | (1 << 23);
    pio_sm_put_blocking((&cntlr_port)->pio, (&cntlr_port)->sm, data_shifted);

#ifdef JOYBUS_TIMEOUT
    sleep_us(JOYBUS_TIMEOUT);
#endif

    int byte_cnt = 0;
    uint32_t ctlr_word = 0;
    while (byte_cnt < 4) {
#ifdef JOYBUS_TIMEOUT
        uint8_t temp_data =
            pio_sm_get((&cntlr_port)->pio, (&cntlr_port)->sm);
#else
        uint8_t temp_data =
            pio_sm_get_blocking((&cntlr_port)->pio, (&cntlr_port)->sm);
#endif
        ctlr_word >>= 8;
        ctlr_word |= (temp_data << 24);
        byte_cnt++;
        // printf("%d ", temp_data);
    }
    // printf("%d\n", ctlr_word);
    // printf("\n");

    joybus_program_send_init((&cntlr_port)->pio, (&cntlr_port)->sm,
                             (&cntlr_port)->offset, (&cntlr_port)->pin,
                             &(&cntlr_port)->config);

    return ctlr_word;
}