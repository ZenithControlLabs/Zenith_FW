# Phobri64 Firmware

This repository contains the firmware for the Phobri64 controller, based off the Zenith controller library.

## Building

1. Install raspberry pi pico sdk: https://github.com/raspberrypi/pico-sdk
   1. Instructions are available in their readme, install scripts are also available.
2. Set PICO_SDK_PATH environment variable to where the sdk is installed
   1. `export PICO_SDK_PATH=</path/to>/pico-sdk`
3. Set PCB environment variable with your board version
   1. versions are listed in "phobri64/include/hw_intf.h"
       1. HW_PHOBRI_PROTO
       2. HW_PHOBRI_V1_0
       3. HW_PHOBRI_V1_1_ANALOG
       4. HW_PHOBRI_V1_1_DATACOLLECT
    2. `export PCB=HW_PHOBRI_V1_0`
4. Configure project with cmake
   1. `cd </path/to>/phobri64`
   2. `mkdir build`
   3. `cd build`
   4. `cmake ..`
5. Build the code
   1. `cmake --build ./`
