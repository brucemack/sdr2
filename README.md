Build/Debug Commands
====================

One-time setup of make process:

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
GP4    
GP5  - RST out to PCM1804 ADC
GP6  - DIN in from PC1804 ADC
GP7  - BCK in from PC1804 ADC
GP8  - Debug out
GP9  - LRCK in from PC1804 ADC
GP10 - SCK out to PCM1804 ADC
GP11
GP12
GP13
GP14
GP15
GP16 - I2C0 SDA for Si5351 (Addr 0x60)
GP17 - I2C0 SCL for Si5351 (Addr 0x60)

Research
========

Consider the PCM5100 DAC and the PCM1863 ADC? SparkFun is using the DAC on their boards.
