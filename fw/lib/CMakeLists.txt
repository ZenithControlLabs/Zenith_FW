add_library(zenith_lib INTERFACE)

target_sources(zenith_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/zenith.c
  ${CMAKE_CURRENT_LIST_DIR}/settings.c 

  ${CMAKE_CURRENT_LIST_DIR}/utilities/running_avg.c

  ${CMAKE_CURRENT_LIST_DIR}/usb/desc_bos.c 
  ${CMAKE_CURRENT_LIST_DIR}/usb/usb.c 
  ${CMAKE_CURRENT_LIST_DIR}/usb/descriptors.c 
  ${CMAKE_CURRENT_LIST_DIR}/usb/webusb.c 

  ${CMAKE_CURRENT_LIST_DIR}/drivers/ads7142.c
  ${CMAKE_CURRENT_LIST_DIR}/drivers/lis3mdl.c

  ${CMAKE_CURRENT_LIST_DIR}/input/btn_remap.c 
  ${CMAKE_CURRENT_LIST_DIR}/input/stick_task.c 

  ${CMAKE_CURRENT_LIST_DIR}/comms/comms.c 
#  ${CMAKE_CURRENT_LIST_DIR}/comms/gamecube.c 
  ${CMAKE_CURRENT_LIST_DIR}/comms/n64.c 
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../analog_lib/ analog_lib)
target_include_directories(zenith_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/../include
  ${CMAKE_CURRENT_LIST_DIR}/../analog_lib/include
  ${CMAKE_CURRENT_LIST_DIR}/../include/zenith/usb # to make TinyUSB happy
)

pico_generate_pio_header(zenith_lib ${CMAKE_CURRENT_LIST_DIR}/comms/joybus.pio)

# Pull in pico libraries that we need
target_link_libraries(zenith_lib INTERFACE
  analog_lib
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

