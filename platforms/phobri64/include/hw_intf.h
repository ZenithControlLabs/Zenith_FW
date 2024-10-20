#ifndef HW_INTF_H
#define HW_INTF_H

#include "main.h"

#if defined(HW_PHOBRI_PROTO)
#include "hw/phobri_proto.h"
#define HW_CORE1_INIT phobri_proto_core1_init
#define HW_READ_ANALOG phobri_proto_read_analog
#elif defined(HW_PHOBRI_V1_X_ANALOG)
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_x_analog_core1_init
#define HW_READ_ANALOG phobri_v1_x_analog_read_analog
#else // HW_PHOBRI_V1_X_3D
#include "hw/phobri_v1_X.h"
#define HW_CORE1_INIT phobri_v1_x_3d_core1_init
#define HW_READ_ANALOG phobri_v1_x_3d_read_analog
#endif

#endif // HW_INTF_H