Build/Debug Commands
====================

One-time setup of make process:

        git submodule update --init
        mkdir build
        cd build
        cmake -DPICO_BOARD=pico2 ..

Command used to flash code to board:        

        ~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "rp2350.dap.core1 cortex_m reset_config sysresetreq" -c "program main.elf verify reset exit"

Pico Pinout Notes
=================

GP0  - (Reserved for UART0 TX)
GP1  - (Reserved for UART0 RX)
GP2  - I2C1 SDA for MCP4725 DAC (Addr 0x60)
GP3  - I2C1 SCL for MCP4725 DAC (Addr 0x60)
GP4  - SCK out to PCM1804 ADC and PCM5100 DAC
GP5  - RST out to PCM1804 ADC

GP6  - DIN in from PC1804 ADC
GP7  - BCK out to PC1804 ADC
GP8  - LRCK out to PC1804 ADC
GP9  - DOUT out to PCM5100 DAC

GP10 - BCK out to PCM5100 DAC
GP11 - LRCK out to PCM5100 DAC
GP12
GP13
GP14
GP15
GP16 - I2C0 SDA for Si5351 (Addr 0x60)
GP17 - I2C0 SCL for Si5351 (Addr 0x60)

Hardware Notes
==============

### Instrumentation Amplifiers on Receive Board

See Horowitz and Hill (3rd ed, section 5.16, figure 5.88) for a description of the 
instrumentation amplifier congiruation used on the receive board.  Gain for this amplifier is 
controlled determined by 1 + 2 R<sub>f</sub> / R<sub>g</sub>.  Notice that the feedback 
path is tapped after a 47 ohm resistor to set the output impedance of the stage.


Research
========

Consider the PCM5100 DAC and the PCM1863 ADC? SparkFun is using the DAC on their boards.

References
==========

[PCM1804 Datasheet](https://www.ti.com/lit/ds/symlink/pcm1804.pdf)


Steps to Build CMSIS-DSP
========================

The library gets built once in a parallel project:

        cd ~/pico
        mkdir CMSISDSP
        cd CMSISDSP
        git clone https://github.com/ARM-software/CMSIS-DSP.git        
        git clone https://github.com/ARM-software/CMSIS_6.git
        cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .

Then setup the CMakeLists.txt shown below and build:

        mkdir build
        cd build
        # IMPORTANT! pico2 build enables the generation of FPU instructions
        cmake -DPICO_BOARD=pico2 ..
        make -j4

What the CMakeLists.txt looks like:

```
cmake_minimum_required (VERSION 3.6)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake) 

set(HOME $ENV{HOME})
set(CMSISDSP ${HOME}/pico/CMSISDSP/CMSIS-DSP)
set(CMSISCORE ${HOME}/pico/CMSISDSP/CMSIS_6/CMSIS/Core)

# Define the project
project (cmsis-dsp VERSION 0.1)

# Initialise the Pico SDK
pico_sdk_init()

add_subdirectory(${CMSISDSP}/Source bin_dsp)

target_compile_options(CMSISDSP PUBLIC 
    -Wsign-compare
    -Wdouble-promotion
    -Ofast -ffast-math
    -DNDEBUG
    -Wall -Wextra  -Werror
    -fshort-enums 
    #-fshort-wchar
)
```

Example of referencing the built CMSIS-DSP library in a client project:

```
set(HOME $ENV{HOME})
target_include_directories(main PRIVATE
  ${HOME}/pico/CMSISDSP/CMSIS-DSP/Include
  ${HOME}/pico/CMSISDSP/CMSIS_6/CMSIS/Core/Include
)
target_link_libraries(main
  pico_stdlib
  ${HOME}/pico/CMSISDSP/build/bin_dsp/libCMSISDSP.a
)
```
