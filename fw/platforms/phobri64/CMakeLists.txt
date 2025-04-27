cmake_minimum_required(VERSION 3.13)

# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# check pre-requisites
if(NOT DEFINED PCB)
	message(FATAL_ERROR "PCB variable is not defined, cannot continue")
else()
	message("PCB=${PCB}")
endif()

if(NOT DEFINED ENV{PICO_SDK_PATH})
	message(FATAL_ERROR "PICO_SDK_PATH variable is not defined, cannot continue")
endif()

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

# configure project
project(Phobri64 C CXX ASM)

pico_sdk_init()

add_subdirectory(${CMAKE_SOURCE_DIR}/../../lib/ lib)

include_directories(
	include
)

add_executable(${PROJECT_NAME}
	src/hw/phobri_proto.c
	src/hw/phobri_v1_0.c
	src/hw/phobri_v1_1_analog.c
	src/hw/phobri_v1_1_datacollect.c
	src/main.c
)

include(../../lib/git_version.cmake)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0")

target_compile_definitions(${PROJECT_NAME}
	PRIVATE
	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	#PICO_DEFAULT_BOOT_STAGE2=boot2_generic_03h
	#PICO_FLASH_SPI_CLKDIV=8
	${PCB}
)

# Enable debugging features when running debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG)
endif()


target_link_libraries(${PROJECT_NAME}
PRIVATE
zenith_lib
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
hardware_i2c
hardware_sync
hardware_timer
hardware_watchdog

tinyusb_device
tinyusb_board
)

pico_add_extra_outputs(${PROJECT_NAME})
