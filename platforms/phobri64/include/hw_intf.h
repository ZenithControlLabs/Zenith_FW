#ifndef HW_INTF_H
#define HW_INTF_H

#include "main.h"

#if defined(HW_PHOBRI_PROTO)
#include "hw/phobri_proto.h"
#define HW_CORE1_INIT phobri_proto_core1_init
#define HW_READ_ANALOG phobri_proto_read_analog
#elif defined(HW_PHOBRI_V1_0)
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_0_core1_init
#define HW_READ_ANALOG phobri_v1_0_read_analog
#elif defined(HW_PHOBRI_V1_1_ANALOG)
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_1_analog_core1_init
#define HW_READ_ANALOG phobri_v1_1_analog_read_analog
#elif defined(HW_PHOBRI_v1_1_DATACOLLECT)
#error "TODO"
#else
#error "No valid HW target defined!"
#endif

#endif // HW_INTF_H