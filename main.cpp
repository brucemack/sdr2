/**
 * Pico I2S Demonstration
 * Copyright (C) 2025, Bruce MacKinnon
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * NOT FOR COMMERCIAL USE WITHOUT PERMISSION.
 */
#include <stdio.h>
#include <math.h>
#include <cstring>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "radlib/util/dsp_util.h"
#include "radlib/util/f32_fft.h"

#include "i2s.pio.h"
#include "si5351.h"

#define I2C0_SDA_PIN (16)  // Physical pin 21
#define I2C0_SCL_PIN (17)  // Physical pin 22

using namespace radlib;

#define LED_PIN (PICO_DEFAULT_LED_PIN)

// DMA stuff
// The *2 accounts for left/right
#define AUDIO_BUFFER_SIZE (32 * 2)
// Here is where the actual audio data gets written
// The *2 accounts for the fact that we are double-buffering.
static __attribute__((aligned(8))) uint32_t audio_buffer[AUDIO_BUFFER_SIZE * 2];
// Here is where the buffer addresses are stored to control DMA
// The *2 accounts for the fact that we are double-buffering.
static __attribute__((aligned(8))) uint32_t* addr_buffer[2];

static uint dma_ch_in_ctrl = 0;
static uint dma_ch_in_data = 0;

static volatile uint32_t dma_counter_0 = 0;

#define AN_BUFFER_SIZE (256)

// Circular buffers were the I/Q signals are accumulated
static float an_buffer_l[AN_BUFFER_SIZE];
static float an_buffer_r[AN_BUFFER_SIZE];
static uint an_buffer_ptr = 0;

// Inside of this handler we will already see the result of the 
// control channel's update of .write_addr.
static void dma_in_handler() {   

    // Figure out which part of the double-buffer we just finished
    // loading into.  Notice: the pointer is signed.
    int32_t* audio_data;
    if (dma_counter_0 % 2 == 0) {
        audio_data = (int32_t*)audio_buffer;
    } else {
        audio_data = (int32_t*)&(audio_buffer[AUDIO_BUFFER_SIZE]);
    }

    // Move into analysis buffer
    for (int i = 0; i < AUDIO_BUFFER_SIZE; i += 2) {
        // The 24-bit signed value is left-justified in the 32-bit word, 
        // so we need to shift right 8. Sign extension is automatic.
        an_buffer_r[an_buffer_ptr] = audio_data[i] >> 8;
        an_buffer_l[an_buffer_ptr] = audio_data[i + 1] >> 8;
        // Increment and wrap
        an_buffer_ptr++;
        if (an_buffer_ptr == AN_BUFFER_SIZE) 
            an_buffer_ptr = 0;
    }

    // Clear the IRQ status
    dma_hw->ints0 = 1u << dma_ch_in_data;

    dma_counter_0++;
}

int main(int argc, const char** argv) {

    unsigned long fs = 48000;
    unsigned long sck_mult = 384;
    unsigned long sck_freq = sck_mult * fs;
    // Pin to be allocated to I2S SCK (output to CODEC)
    // GP10 is physical pin 14
    uint sck_pin = 10;
    // Pin to be allocated to ~RST
    uint rst_pin = 5;
    // Pin to be allocated to I2S DIN (input from)
    uint din_pin = 6;
    unsigned long system_clock_khz = 125000;

    // Adjust system clock to more evenly divide the 
    // audio sampling frequency.
    set_sys_clock_khz(system_clock_khz, true);

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(rst_pin);
    gpio_set_dir(rst_pin, GPIO_OUT);
    gpio_put(rst_pin, 1);

    // This example will use I2C0!
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SDA_PIN);

    // Startup ID
    sleep_ms(500);

    // ----- Si5351 Initialization --------------------------------------------
    // We are using I2C0 here!
    si_init(i2c0);
    si_enable(0, true);

    int32_t freq = 7255000;
    int32_t cal = 490;

    // Change freq
    si_evaluate(0, freq + cal);

    printf("Si5351 freq %d\n", freq);

    // ----- PCM1804 A/D Converter Setup -------------------------------------

    // Assuming a system freq of 125 MHz, an SCK PIO divisor of 4 
    // and an SCK = fs * 384.
    float sample_freq = 40690;
    const uint work_size = 256;
    float trigSpace[work_size];
    F32FFT fft(work_size, trigSpace);

    // Reset the CODEC
    gpio_put(rst_pin, 0);
    sleep_ms(100);
    gpio_put(rst_pin, 1);
    sleep_ms(100);

    // ----- SCK Setup ------------------------------------------------------

    // Allocate state machine
    uint sck_sm = pio_claim_unused_sm(pio0, true);
    uint sck_sm_mask = 1 << sck_sm;

    // Load PIO program into the PIO
    uint sck_program_offset = pio_add_program(pio0, &i2s_sck_program);
  
    // Setup the function select for a GPIO to use output from the given PIO 
    // instance.
    // 
    // PIO appears as an alternate function in the GPIO muxing, just like an 
    // SPI or UART. This function configures that multiplexing to connect a 
    // given PIO instance to a GPIO. Note that this is not necessary for a 
    // state machine to be able to read the input value from a GPIO, but only 
    // for it to set the output value or output enable.
    pio_gpio_init(pio0, sck_pin);

    // NOTE: The xxx_get_default_config() function is generated by the PIO
    // assembler and defined inside of the generated .h file.
    pio_sm_config sck_sm_config = 
        i2s_sck_program_get_default_config(sck_program_offset);
    // Associate pin with state machine. 
    // Because we are using the "SET" command in the PIO program
    // (and not OUT or side-set) we use the set_set function here.
    sm_config_set_set_pins(&sck_sm_config, sck_pin, 1);
    // Initialize setting and direction of the pin before SM is enabled
    uint sck_pin_mask = 1 << sck_pin;
    pio_sm_set_pins_with_mask(pio0, sck_sm, 0, sck_pin_mask);
    pio_sm_set_pindirs_with_mask(pio0, sck_sm, sck_pin_mask, sck_pin_mask);
    // Hook it all together.  (But this does not enable the SM!)
    pio_sm_init(pio0, sck_sm, sck_program_offset, &sck_sm_config);

    // Adjust state-machine clock divisor.  Remember that we need
    // the state machine to run at 2x SCK speed since it takes two 
    // instructions to acheive one clock transition on the pin.
    //
    // NOTE: The clock divisor is in 16:8 format
    //
    // d d d d d d d d d d d d d d d d . f f f f f f f f
    //            Integer Part         |  Fraction Part
    unsigned int sck_sm_clock_d = 4;
    //unsigned int sck_sm_clock_f = 132;
    // Avoid jitter by avoiding fractional clock division.
    unsigned int sck_sm_clock_f = 0;
    // Sanity check:
    // 2 * (3 + (132/256)) = 7.03125
    // 7.03125 * 48,000 * 384 = 129,600,000
    pio_sm_set_clkdiv_int_frac(pio0, sck_sm, sck_sm_clock_d, sck_sm_clock_f);

    // Final enable of the SCK state machine
    pio_enable_sm_mask_in_sync(pio0, sck_sm_mask);

    // Now issue a reset of the CODEC
    // Per datasheet page 18: "Because the system clock is used as a clock signal
    // for the reset circuit, the system clock must be supplied as soon as the 
    // power is supplied ..."
    //
    sleep_ms(100);
    gpio_put(rst_pin, 0);
    sleep_ms(5);
    gpio_put(rst_pin, 1);

    // Per PCM1804 datasheet page 18: 
    //
    // "The digital output is valid after the reset state is released and the 
    // time of 1116/fs has passed."
    // 
    // Assuming fs = 40,690 hz, then we must wait at least 27ms after reset!

    sleep_ms(50);

    // ----- DIN Setup ------------------------------------------------------

    // Allocate state machine
    uint din_sm = pio_claim_unused_sm(pio0, true);
    uint din_sm_mask = 1 << din_sm;

    // Load program into the PIO
    uint din_program_offset = pio_add_program(pio0, &i2s_din_program);
  
    // Setup the function select for a GPIO to use from the given PIO 
    // instance.
    // DIN
    pio_gpio_init(pio0, din_pin);
    gpio_set_pulls(din_pin, false, false);
    gpio_set_dir(din_pin, GPIO_IN);
    // BCK
    pio_gpio_init(pio0, din_pin + 1);
    gpio_set_pulls(din_pin + 1, false, false);
    gpio_set_dir(din_pin + 1, GPIO_IN);
    // LRCK
    pio_gpio_init(pio0, din_pin + 2);
    gpio_set_pulls(din_pin + 2, false, false);
    gpio_set_dir(din_pin + 2, GPIO_IN);
    // DIAG (OUT)
    pio_gpio_init(pio0, din_pin + 3);
    //gpio_set_pulls(din_pin + 3, false, false);
    gpio_set_dir(din_pin + 3, GPIO_OUT);

    // NOTE: The xxx_get_default_config() function is generated by the PIO
    // assembler and defined inside of the generated .h file.
    pio_sm_config din_sm_config = 
        i2s_din_program_get_default_config(din_program_offset);
    // Associate the input pins with state machine.  This will be 
    // relevant to the DIN pin for IN instructions and the BCK, LRCK
    // pins for WAIT instructions.
    sm_config_set_in_pins(&din_sm_config, din_pin);
    // Set the "jump pin" for the state machine. This will be the 
    // LRCK pin in this usage.
    sm_config_set_jmp_pin(&din_sm_config, din_pin + 2);
    // Output (debug pin)
    sm_config_set_set_pins(&din_sm_config, din_pin + 3, 1);
    // Configure the IN shift behavior.
    // Parameter 0: "false" means shift ISR to left on input.
    // Parameter 1: "false" means autopush is not enabled.
    // Parameter 2: "0" means threshold (in bits) before auto/conditional 
    //              push to the ISR.
    sm_config_set_in_shift(&din_sm_config, false, false, 0);
    // Merge the FIFOs since we are only doing RX.  This gives us 
    // 8 words of buffer instead of the usual 4.
    sm_config_set_fifo_join(&din_sm_config, PIO_FIFO_JOIN_RX);

    // Initialize the direction of the pins before SM is enabled
    // There are four pins in the mask here. 
    uint din_pins_mask = 0b1111 << din_pin;
    uint din_pindirs   = 0b1000 << din_pin;
    // The "0" means input, "1" means output
    pio_sm_set_pindirs_with_mask(pio0, din_sm, din_pindirs, din_pins_mask);

    // Hook it all together.  (But this does not enable the SM!)
    pio_sm_init(pio0, din_sm, din_program_offset, &din_sm_config);
          
    // Adjust state-machine clock divisor.  The speed is somewhat
    // arbitrary here, so long as it is fast enough to see the 
    // transitions on BCK and LRCK.  We run it at the same speed as the 
    //
    // NOTE: The clock divisor is in 16:8 format
    //
    // d d d d d d d d d d d d d d d d . f f f f f f f f
    //            Integer Part         |  Fraction Part
    pio_sm_set_clkdiv_int_frac(pio0, din_sm, 
        sck_sm_clock_d, sck_sm_clock_f);

    // ----- DMA setup -------------------------------------------

    // The control channel will read between these two addresses,
    // telling the data channel to write to them alternately (i.e.
    // double-buffer).
    addr_buffer[0] = audio_buffer;
    addr_buffer[1] = &(audio_buffer[AUDIO_BUFFER_SIZE]);
    
    dma_ch_in_ctrl = dma_claim_unused_channel(true);
    dma_ch_in_data = dma_claim_unused_channel(true);

    // Setup the control channel. This channel is only needed to 
    // support the double-buffering behavior. A write by the control
    // channel will trigger the data channel to wake up and 
    // start to move data out of the PIO RX FIFO.
    dma_channel_config cfg = dma_channel_get_default_config(dma_ch_in_ctrl);
    // The control channel needs to step across the addresses of 
    // the various buffers.
    channel_config_set_read_increment(&cfg, true);
    // But always writing into the same location
    channel_config_set_write_increment(&cfg, false);
    // Sanity check before we start making assumptions about the
    // transfer size.
    assert(sizeof(uint32_t*) == 4);
    // Configure how many bits are involved in the address rotation.
    // 3 bits are used because we are wrapping through a total of 8 
    // bytes (two 4-byte addresses).  
    // The "false" means the read side.
    channel_config_set_ring(&cfg, false, 3);
    // Each address is 32-bits, so that's what we need to transfer 
    // each time.
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    // Program the DMA channel
    dma_channel_configure(dma_ch_in_ctrl, &cfg, 
        // Here is where we write to (the data channel)
        // NOTE: dma_hw is a global variable from the PICO SDK
        // Since we are writing to write_addr_trig, the result of 
        // the control channel write will be to start the data
        // channel.
        &dma_hw->ch[dma_ch_in_data].al2_write_addr_trig,
        // Here is where we start to read from (the address 
        // buffer area).
        addr_buffer, 
        // TRANS_COUNT: Number of transfers to perform before stopping.
        // This count will be reset to the original value (1) every 
        // time the channel is started.
        1, 
        // false means don't start yet
        false);

    // Setup the data channel.
    cfg = dma_channel_get_default_config(dma_ch_in_data);
    // No increment required because we are always reading from the 
    // PIO RX FIFO every time.
    channel_config_set_read_increment(&cfg, false);
    // We need to increment the write to move across the buffer
    channel_config_set_write_increment(&cfg, true);
    // Set size of each transfer (one audio word)
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    // We trigger the control channel once the data transfer is done
    channel_config_set_chain_to(&cfg, dma_ch_in_ctrl);
    // Attach the DMA channel to the RX DREQ of the PIO state machine. 
    // The "false" below indicates RX.
    // This is the "magic" that connects the PIO SM to the DMA.
    channel_config_set_dreq(&cfg, pio_get_dreq(pio0, din_sm, false));
    // Program the DMA channel
    dma_channel_configure(dma_ch_in_data, &cfg,
        // Initial write address
        // 0 means that the target will be set by the control channel
        0, 
        // Initial Read address
        // The memory-mapped location of the RX FIFO of the PIO state
        // machine used for receiving data
        // This is the "magic" that connects the PIO SM to the DMA.
        &(pio0->rxf[din_sm]),
        // Number of transfers (each is 32 bits)
        AUDIO_BUFFER_SIZE,
        // Don't start yet
        false);

    // Enable interrupt when DMA data transfer completes
    dma_channel_set_irq0_enabled(dma_ch_in_data, true);
    // Bind to the interrupt handler and enable
    irq_set_exclusive_handler(DMA_IRQ_0, dma_in_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Start DMA action on the control side.  This will trigger
    // the data DMA channel in turn.
    dma_channel_start(dma_ch_in_ctrl);

    // Final enable of the two SMs to keep them in sync.
    pio_enable_sm_mask_in_sync(pio0, sck_sm_mask | din_sm_mask);

    // Endless loop
    while (true) {

        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);

        // Grab a copy of the structure that may be changing 
        // inside of an ISR.
        // IMPORTANT: Interrupts are disabled during the copy!
        float an_buffer_l_copy[AN_BUFFER_SIZE];
        uint32_t in = save_and_disable_interrupts();
        std::memcpy(an_buffer_l_copy, an_buffer_l, AN_BUFFER_SIZE * sizeof(float));
        restore_interrupts(in);

        // Analyze the buffers
        cf32 work[work_size];
        float max_mag = 0;
        for (int i = 0; i < work_size; i++) {
            work[i].r = an_buffer_l_copy[i];
            work[i].i = 0;
            if (fabs(an_buffer_l_copy[i]) > max_mag)
                max_mag = fabs(an_buffer_l_copy[i]);
        }

        //printf("dma_counter_0/_01=%d/%d\n", dma_counter_0, dma_counter_1);

        // FFT stuff
        fft.transform(work);
        uint maxIdx = maxMagIdx(work, 0, work_size / 2);
        printf("Max mag %d, FFT max bin %d, mag %d\n", 
            (int)max_mag, maxIdx, (int)work[maxIdx].mag());
   }

    return 0;
}