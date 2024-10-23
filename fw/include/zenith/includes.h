#ifndef ZENITH_INCLUDES_H
#define ZENITH_INCLUDES_H

// clang-format off

// system includes
#include <math.h>
#ifndef __STDC_NO_ATOMICS__
#include <stdatomic.h>
#endif

#include <pico/multicore.h>
#include <pico/bootrom.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <tusb.h>

// local includes
#include "zenith_cfg.h" // Compile-time config file, provided by the userspace library.
#include "zenith_cfg_def.h" // The defaults.

#include "zenith.h" //  The public facing API
#include "zenith/types.h"   // Commonly used structs.
#include "zenith/settings.h" // Where the settings struct is held. Routines to write it to flash.

#include "zenith/utilities/interval.h"
#include "zenith/utilities/misc.h"

#include "zenith/usb/tusb_config.h"
#include "zenith/usb/descriptors.h"
#include "zenith/usb/desc_bos.h"
#include "zenith/usb/webusb.h"
#include "zenith/usb/usb.h" // The USB interface.

#include "joybus.pio.h"
#include "zenith/comms/gamecube.h"
#include "zenith/comms/n64.h"
#include "zenith/comms/n64_crc.h"
#include "zenith/comms/comms.h" // For talking over the controller's main protocol (N64, GCC, etc.)

#include "zenith/input/btn_remap.h"
#include "zenith/input/stick.h"
#include "zenith/input/notch_remap.h"
#include "zenith/input/linearize.h"
#include "zenith/input/calib.h"

// clang-format on

#endif // ZENITH_INCLUDES_H