add_library(analog_lib INTERFACE)

target_sources(analog_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/linearize.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/notch_remap.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/snapback_filter.c 
  ${CMAKE_CURRENT_LIST_DIR}/src/stick.c 
)

target_include_directories(analog_lib INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)