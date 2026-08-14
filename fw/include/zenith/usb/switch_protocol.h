#ifndef ZENITH_SWITCH_PROTOCOL_H
#define ZENITH_SWITCH_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

void switch_protocol_init(void);
void switch_protocol_queue_output(uint8_t report_id, const uint8_t *data,
                                  uint16_t len);
bool switch_protocol_make_report(uint8_t report[64], btn_data_t *buttons,
                                 analog_data_t *analog);

#endif
