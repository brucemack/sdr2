Build/Debug Commands
====================

One-time setup of make process:

        mkdir build
        cd build
        cmake -DPICO_BOARD=pico2 ..

Command used to flash code to board:        

        ~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "rp2350.dap.core1 cortex_m reset_config sysresetreq" -c "program main.elf verify reset exit"
