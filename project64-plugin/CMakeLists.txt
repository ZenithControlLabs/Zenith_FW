cmake_minimum_required(VERSION 3.20)
project(ZenithRawInput LANGUAGES CXX)

add_library(ZenithRawInput SHARED
    src/plugin.cpp
    src/usb_device.cpp
)
target_compile_features(ZenithRawInput PRIVATE cxx_std_17)
target_compile_definitions(ZenithRawInput PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_include_directories(ZenithRawInput PRIVATE include src)
target_link_libraries(ZenithRawInput PRIVATE setupapi winusb comctl32)
set_target_properties(ZenithRawInput PROPERTIES
    OUTPUT_NAME "Zenith_Raw_Input"
    PREFIX ""
)

if(MSVC)
    target_compile_options(ZenithRawInput PRIVATE /W4 /permissive- /EHsc /utf-8)
endif()
