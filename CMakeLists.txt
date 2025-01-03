cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(hello-pico-i2s C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(main
  main.cpp
  i2s.pio
  si5351.c
  radlib/util/dsp_util.cpp
  radlib/util/f32_fft.cpp
)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
target_link_libraries(main pico_stdlib hardware_i2c hardware_dma 
  hardware_pio hardware_clocks)
pico_add_extra_outputs(main)
