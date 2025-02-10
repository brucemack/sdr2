cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(sdr2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

pico_sdk_init()

add_executable(main
  main.cpp
  sweeper.cpp
  i2s.pio
  si5351.c
  radlib/util/dsp_util.cpp
  radlib/util/f32_fft.cpp
  kc1fsz-tools-cpp/src/rp2040/PicoPollTimer.cpp
  kc1fsz-tools-cpp/src/rp2040/PicoPerfTimer.cpp
)
#pico_add_extra_outputs(main)
target_compile_definitions(main PRIVATE PICO_BUILD=1)
pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)
target_include_directories(main PRIVATE 
  kc1fsz-tools-cpp/include
  $ENV{HOME}/pico/CMSISDSP/CMSIS-DSP/Include
  $ENV{HOME}/pico/CMSISDSP/CMSIS_6/CMSIS/Core/Include
)
target_link_libraries(main 
  pico_stdlib 
  hardware_i2c 
  hardware_dma 
  hardware_pio 
  hardware_clocks
  $ENV{HOME}/pico/CMSISDSP/build/bin_dsp/libCMSISDSP.a
)
