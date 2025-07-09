#ifndef HW_INTF_H
#define HW_INTF_H

#include "main.h"

#if defined(HW_PHOBRI_PROTO)
#define ZTH_LINEARIZATION_EN 1
#include "hw/phobri_proto.h"
#define HW_CORE1_INIT phobri_proto_core1_init
#define HW_READ_ANALOG phobri_proto_read_analog
#elif defined(HW_PHOBRI_V1_0)
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_0_core1_init
#define HW_READ_ANALOG phobri_v1_0_read_analog
void phobri_v1_0_core1_init(void);
void phobri_v1_0_read_analog(analog_data_t *analog);
#elif defined(HW_PHOBRI_V1_1_ANALOG)
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_1_analog_core1_init
#define HW_READ_ANALOG phobri_v1_1_analog_read_analog
void phobri_v1_1_analog_core1_init(void);
void phobri_v1_1_analog_read_analog(analog_data_t *analog);
#elif defined(HW_PHOBRI_V1_1_DATACOLLECT)
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_1_datacollect_core1_init
#define HW_READ_ANALOG phobri_v1_1_datacollect_read_analog
void phobri_v1_1_datacollect_core1_init(void);
void phobri_v1_1_datacollect_read_analog(analog_data_t *analog);
#else
#error "No valid HW target defined!"
#endif

#endif // HW_INTF_H