#ifndef ZENITH_USB_H
#define ZENITH_USB_H

int usb_init(void);

void usb_task(uint32_t timestamp, btn_data_t *buttons, analog_data_t *analog,
              analog_data_t *analog_raw);

#endif // ZENITH_USB_H
