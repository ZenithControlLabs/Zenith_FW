add_library(zenith_lib INTERFACE)

target_sources(zenith_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/zenith.c
  ${CMAKE_CURRENT_LIST_DIR}/src/settings.c 

  ${CMAKE_CURRENT_LIST_DIR}/src/utilities/running_avg.c

  ${CMAKE_CURRENT_LIST_DIR}/src/usb/desc_bos.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/usb/usb.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/usb/descriptors.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/usb/webusb.c 

  ${CMAKE_CURRENT_LIST_DIR}/src/input/btn_remap.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/input/calib.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/input/linearize.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/input/notch_remap.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/input/stick.c 

  ${CMAKE_CURRENT_LIST_DIR}/src/comms/comms.c 
#  ${CMAKE_CURRENT_LIST_DIR}/src/comms/gamecube.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/comms/n64.c 
)

target_include_directories(zenith_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/include/zenith/usb # to make TinyUSB happy
)

pico_generate_pio_header(zenith_lib ${CMAKE_CURRENT_LIST_DIR}/src/comms/joybus.pio)

# Pull in pico libraries that we need
target_link_libraries(zenith_lib INTERFACE
  pico_stdlib
  pico_bootrom
  pico_multicore
  pico_platform
  pico_rand

  hardware_gpio
  hardware_adc
  hardware_flash
  hardware_pio
  hardware_dma
  hardware_pwm
  hardware_spi
  hardware_sync
  hardware_timer
  hardware_irq

  tinyusb_device
  tinyusb_board
)

