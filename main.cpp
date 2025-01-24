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
 *  
 */

/*
Command used to load code onto the board: 

~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "rp2350.dap.core1 cortex_m reset_config sysresetreq" -c "program main.elf verify reset exit"

*/
#include <stdio.h>
#include <math.h>
#include <cstring>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"

#include "kc1fsz-tools/rp2040/PicoPollTimer.h"
#include "kc1fsz-tools/rp2040/PicoPerfTimer.h"
#include "radlib/util/dsp_util.h"
#include "radlib/util/f32_fft.h"

#include "i2s.pio.h"
#include "si5351.h"

#define I2C0_SDA_PIN (16)  // Physical pin 21
#define I2C0_SCL_PIN (17)  // Physical pin 22

#define I2C1_SDA_PIN (2)  
#define I2C1_SCL_PIN (3)  

using namespace radlib;
using namespace kc1fsz;

#define LED_PIN (PICO_DEFAULT_LED_PIN)

// Diagnostic counters
static volatile uint32_t txFifoFull = 0;
static volatile uint32_t dacBufferEmpty = 0;
static volatile uint32_t dac_tick_count = 0;
static volatile uint32_t diag_count_0 = 0;
static volatile uint32_t diag_count_1 = 0;
static volatile uint32_t diag_count_2 = 0;
static volatile uint32_t dac_buffer_write_count = 0;
static volatile uint32_t dac_buffer_nonempty_count = 0;
static volatile uint32_t max_proc_0 = 0;

// DMA stuff for ADC
// The *2 accounts for left/right channels
#define ADC_BUFFER_SIZE (384 * 2)
// Here is where the actual audio data gets written
// The *2 accounts for the fact that we are double-buffering.
static __attribute__((aligned(8))) uint32_t adc_buffer[ADC_BUFFER_SIZE * 2];
// Here is where the buffer addresses are stored to control DMA
// The *2 accounts for the fact that we are double-buffering.
static __attribute__((aligned(8))) uint32_t* adc_addr_buffer[2];

static uint dma_ch_in_ctrl = 0;
static uint dma_ch_in_data = 0;

// Circular analysis buffers were the raw I/Q signals are accumulated for further
// analysis. Fs=48k
#define AN_BUFFER_SIZE (1024)
static float an_buffer_i[AN_BUFFER_SIZE];
static float an_buffer_q[AN_BUFFER_SIZE];
// Location of *next* read/write into the buffer
static uint an_buffer_wr_ptr = 0;
static uint an_buffer_rd_ptr = 0;

// LSB/USB selector
static bool modeLSB = true;
//static bool modeLSB = false;

#define HILBERT_SIZE (31)

static float hilbert_impulse[HILBERT_SIZE] = {
    0.004195635890348866, 
    -1.2790256324988477e-15, 
    0.009282101548804558, 
    -3.220409857465908e-16, 
    0.01883580699770617, 
    -8.18901417658659e-16, 
    0.03440100801932521, 
    -6.356643085811313e-16, 
    0.059551575569702433, 
    -8.708587876669048e-16, 
    0.10303763641989427, 
    -6.507176308640055e-16, 
    0.19683153562363995, 
    -1.8755360872545065e-16, 
    0.6313536408821954, 
    0, 
    -0.6313536408821954, 
    1.8755360872545065e-16, 
    -0.19683153562363995, 
    6.507176308640055e-16, 
    -0.10303763641989427, 
    8.708587876669048e-16, 
    -0.059551575569702433, 
    6.356643085811313e-16, 
    -0.03440100801932521, 
    8.18901417658659e-16, 
    -0.01883580699770617, 
    3.220409857465908e-16, 
    -0.009282101548804558, 
    1.2790256324988477e-15, 
    -0.004195635890348866 
};

// Build the group-delay in samples.
// This is assuming an odd Hilbert size.
// NOTE: The +1 seems to give slightly better rejection
#define HILBERT_GROUP_DELAY ((HILBERT_SIZE + 1) / 2) + 1

// Circular buffer - result of Hilbert transform (i.e. one sideband 
// eliminated). Fs=48k
static float ssb_buffer[AN_BUFFER_SIZE];

#define LPF_IMPULSE_SIZE (51)

static float LPFImpulse[LPF_IMPULSE_SIZE] = {
    -0.007988518488193047, -0.0322923540338797, -0.0025580477346377516, -0.007508482037031126, -0.0015547763074605164, 0.003280642290253403, 0.008795348444624476, 0.013112016273176279, 0.015229588003704697, 0.014260356689224188, 0.009841832366608681, 0.002295390261866158, -0.007296945242233224, -0.01717660135976687, -0.025298778719890665, -0.029398991822789773, -0.027623603206855935, -0.018731596787559205, -0.0025587380614820365, 0.020105815714798622, 0.04727569765636943, 0.07608694879779716, 0.10336445155058628, 0.12570901987001032, 0.14037248973886046, 0.14550041816800013, 0.14037248973886046, 0.12570901987001032, 0.10336445155058628, 0.07608694879779716, 0.04727569765636943, 0.020105815714798622, -0.0025587380614820365, -0.018731596787559205, -0.027623603206855935, -0.029398991822789773, -0.025298778719890665, -0.01717660135976687, -0.007296945242233224, 0.002295390261866158, 0.009841832366608681, 0.014260356689224188, 0.015229588003704697, 0.013112016273176279, 0.008795348444624476, 0.003280642290253403, -0.0015547763074605164, -0.007508482037031126, -0.0025580477346377516, -0.0322923540338797, -0.007988518488193047
};

// Circular buffer - result of the LPF. Fs=48k
static float lpf_buffer[AN_BUFFER_SIZE];

static uint decimation_counter = 0;

// Buffer used to drive the DAC.  This is treated like a 12-bit 
// integer which is written directly to the DAC (i.e. all pre-scaling
// is done in advance).
#define DAC_BUFFER_SIZE (256)
static uint16_t dac_buffer[DAC_BUFFER_SIZE];
static volatile uint32_t dac_buffer_wr_ptr = 0;
static volatile uint32_t dac_buffer_rd_ptr = 0;

/**
 * @param x Circular signal buffer.  
 * @param h Non-circular impulse response. Size (hN) is arbitrary, but 
 * must be smaller than xN.
 * @param xNext Most recent insert location in circular buffer x.
 * @param xN Size of circular buffer x. 
 * @param hN Size of impulse response buffer h.
 */
static float convolve_circular_f32(const float* x, unsigned int xNext, unsigned int xN, 
    const float* h, unsigned int hN) {
    unsigned int n = xNext;
    float result = 0;
    for (unsigned int k = 0; k < hN; k++) {
        // Multiply-accumulate
        result += x[n] * h[k];
        // Move backwards through the signal buffer x, per the definintion 
        // of convolution.  Implement the wrap when needed.
        if (n == 0) {
            n = xN - 1;
        } else {
            n = n - 1;
        }
    }
    return result;
}

// This will be called once every AUDIO_BUFFER_SIZE/2 samples.
//
// If AUDIO_BUFFER_SIZE is 384 * 2, then every time this is called 
// we'll have 384 I/Q sample pairs to work with.  
static void dma_in_handler() {   

    // Counter used to alternate between double-buffer sides
    static uint32_t dma_count_0 = 0;

    diag_count_2++;

    // Figure out which part of the double-buffer we just finished
    // loading into.  Notice: the pointer is signed.
    int32_t* adc_data;
    if (dma_count_0 % 2 == 0) {
        adc_data = (int32_t*)adc_buffer;
    } else {
        adc_data = (int32_t*)&(adc_buffer[ADC_BUFFER_SIZE]);
    }

    // Move from the DMA buffer and into analysis buffer.  This also 
    // separates the I/Q streams, corrects the scaling, and produces
    // the real LSB/USB.
    for (int i = 0; i < ADC_BUFFER_SIZE; i += 2) {

        // The 24-bit signed value is left-justified in the 32-bit word, 
        // so we need to shift right 8. Sign extension is automatic.
        an_buffer_i[an_buffer_wr_ptr] = adc_data[i] >> 8;
        an_buffer_q[an_buffer_wr_ptr] = adc_data[i + 1] >> 8;

        // Increment and wrap
        an_buffer_wr_ptr++;
        if (an_buffer_wr_ptr == AN_BUFFER_SIZE) 
            an_buffer_wr_ptr = 0;
        // Look for overflows (i.e. background process not keeping up)
        if (an_buffer_rd_ptr == an_buffer_wr_ptr)
            diag_count_1++;
    }
   
    // Clear the IRQ status
    dma_hw->ints0 = 1u << dma_ch_in_data;

    dma_count_0++;
}

static repeating_timer_t dac_timer; 

static bool dac_tick(repeating_timer_t* t) {

    dac_tick_count++;

    i2c_hw_t *hw = i2c_get_hw(i2c1);

    // Tx FIFO must not be full
    if (!(hw->status & I2C_IC_STATUS_TFNF_BITS)) {
        txFifoFull++;
        return true;
    }

    // Make sure we have something in the DAC buffer
    if (dac_buffer_rd_ptr == dac_buffer_wr_ptr) {
        dacBufferEmpty++;
        return true;
    }
    
    uint16_t rawSample = dac_buffer[dac_buffer_rd_ptr];

    // All scaling and offsets are done in advance to make this 
    // ISR as fast as possible.
    // To create an output sample we need to write three words.  The STOP flag
    // is set on the last one.
    //
    // 0 0 0 | 0   1   0   x   x   0   0   x 
    // 0 0 0 | d11 d10 d09 d08 d07 d06 d05 d04
    // 0 1 0 | d03 d02 d01 d00 x   x   x   x
    //   ^
    //   |
    //   +------ STOP BIT!
    //
    hw->data_cmd = 0b000'0100'0000;
    hw->data_cmd = 0b000'0000'0000 | ((rawSample >> 4) & 0xff); // High 8 bits
    // STOP requested.  Data is low 4 bits of sample, padded on right with zeros
    hw->data_cmd = 0b010'0000'0000 | ((rawSample << 4) & 0xff);       
    
    dac_buffer_rd_ptr++;
    if (dac_buffer_rd_ptr == DAC_BUFFER_SIZE) 
        dac_buffer_rd_ptr = 0;

    return true;
}

// Converts from scale if input to scale of output (0-4095)
//static float dacScale = 0.05;
static float dacScale = 0.01;

// This should be called when a complete frame of I/Q data has been 
// received.
//
// Assuming the inbound rate is 48,000 and the outbound rate is 8,000, 
// we'll decimate by a factor of 6 to create 384/6 = 64 outbound samples
// in each call.
//
// Benchmark: measured 2.6ms to process 8ms (384 samples) of I/Q input data.
//
static void process_in_frame() {

    // Safely capture the start and end of the analysis buffer
    // IMPORTANT: Interrupts are disabled
    uint32_t in_state = save_and_disable_interrupts();
    // Catch up to inbound DMA by reading up to BUT NOT INCLUDING here
    unsigned int an_end = an_buffer_wr_ptr;
    // Here is where the reading starts (where we left off before)
    unsigned int new_an_buffer_rd_ptr = an_buffer_rd_ptr;
    // Here is where we should start writing oubound DAC data
    unsigned int new_dac_buffer_wr_ptr = dac_buffer_wr_ptr;
    restore_interrupts(in_state);

    // This processing loop happens while interrupts are enabled
    // so it's lower priority than the DMA and DAC timer.

    while (new_an_buffer_rd_ptr != an_end) {

        // Apply the delay to the I stream, taking into account a 
        // possible wrap around the start of the circular buffer.
        unsigned int delayed_n;
        if (new_an_buffer_rd_ptr >= HILBERT_GROUP_DELAY) {
            delayed_n = new_an_buffer_rd_ptr - HILBERT_GROUP_DELAY;
        } else {
            delayed_n = AN_BUFFER_SIZE + new_an_buffer_rd_ptr - HILBERT_GROUP_DELAY;
        }
        float sampleI = an_buffer_i[delayed_n];

        // Apply the Hilbert transform to the Q stream.
        float sampleQ = convolve_circular_f32(an_buffer_q, new_an_buffer_rd_ptr, AN_BUFFER_SIZE,
            hilbert_impulse, HILBERT_SIZE);
        //float sampleQ = 0;

        float sampleSSB;
        if (modeLSB) {
            sampleSSB = sampleI + sampleQ;
        } else {
            sampleSSB = sampleI - sampleQ;
        }

        // Save in output circular buffer
        ssb_buffer[new_an_buffer_rd_ptr] = sampleSSB;

        // Apply a LFP to get rid of any high-frequency content in the baseband 
        // audio signal.
        float audio48 = convolve_circular_f32(ssb_buffer, new_an_buffer_rd_ptr, AN_BUFFER_SIZE,
            LPFImpulse, LPF_IMPULSE_SIZE);
        //float audio48 = 0;

        // Save in output circular buffer
        lpf_buffer[new_an_buffer_rd_ptr] = audio48;
        
        // Decimation down to audio sample rate of 8k (/6)       
        if (++decimation_counter == 6) {
            
            // Here we need to scale to a -2,0248->+2,2048 range
            float audioScaled = audio48 * dacScale;
            // Here we need to adjust to get a 0->4095 range
            float audioCentered = audioScaled + 2048.0;
            // Clip to 12 bits.
            if (audioCentered < 0) 
                audioCentered = 0;
            else if (audioCentered > 4095) 
                audioCentered = 4095;

            dac_buffer[new_dac_buffer_wr_ptr] = (uint16_t)audioCentered;

            // Move the write pointer forward 
            new_dac_buffer_wr_ptr++;
            if (new_dac_buffer_wr_ptr == DAC_BUFFER_SIZE)
                new_dac_buffer_wr_ptr = 0;

            decimation_counter = 0;
            dac_buffer_write_count++;
        }
        
        // Increment and wrap
        new_an_buffer_rd_ptr++;
        if (new_an_buffer_rd_ptr == AN_BUFFER_SIZE) 
           new_an_buffer_rd_ptr = 0;
    }

    // Adjust the pointers on the buffers
    in_state = save_and_disable_interrupts();
    an_buffer_rd_ptr = new_an_buffer_rd_ptr;
    dac_buffer_wr_ptr = new_dac_buffer_wr_ptr;
    restore_interrupts(in_state);
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
    unsigned long system_clock_khz = 129600;
    PicoPerfTimer timer_0;

    // Adjust system clock to more evenly divide the 
    // audio sampling frequency.
    set_sys_clock_khz(system_clock_khz, true);

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(rst_pin);
    gpio_set_dir(rst_pin, GPIO_OUT);
    gpio_put(rst_pin, 1);

    // I2C bus setup
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    // NOTE: The Sparkfun MCP4725 breakout board has 4.7k pullups on the 
    // I2C lines.  Therefore we are not enabling them here.
    //gpio_pull_up(I2C1_SDA_PIN);
    //gpio_pull_up(I2C1_SCL_PIN);
    i2c_set_baudrate(i2c1, 800000);

    // Startup ID
    sleep_ms(500);
    sleep_ms(500);

    int32_t freq = 7255000;
    int32_t cal = 490;

    printf("Minimal SDR2\nCopyright (C) 2025 Bruce MacKinnon KC1FSZ\n");
    printf("Freq %d\n", freq);

    // ===== Si5351 Initialization =============================================

    // We are using I2C0 here!
    si_init(i2c0);
    si_enable(0, true);
    // Change freq
    si_evaluate(0, freq + cal);
    printf("Si5351 initialized 1\n");

    // ===== MCP4725 DAC Setup ================================================

    // One-time initialization of the I2C channel
    i2c_hw_t *hw = i2c_get_hw(i2c1);
    hw->enable = 0;
    hw->tar = 0x60;
    hw->enable = 1;

    // 8kHz timer for DAC output.  
    // The negative time is used to indicate the time between callback starts.
    // This uses the default alarm pool.
    add_repeating_timer_us(-125, dac_tick, 0, &dac_timer);

    // ===== PCM1804 A/D Converter Setup ======================================

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
    unsigned int sck_sm_clock_d = 3;
    unsigned int sck_sm_clock_f = 132;
    // Sanity check:
    // 2 * (3 + (132/256)) = 7.03125
    // 7.03125 * 48,000 * 384 = 129,600,000

    // Avoid jitter by avoiding fractional clock division.
    //unsigned int sck_sm_clock_f = 0;
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
    adc_addr_buffer[0] = adc_buffer;
    adc_addr_buffer[1] = &(adc_buffer[ADC_BUFFER_SIZE]);
    
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
        adc_addr_buffer, 
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
        ADC_BUFFER_SIZE,
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

    int strobe = 0;
    
    // Audio processing should happen once every DMA frame (8ms)
    // in order to avoid falling behind.
    PicoPollTimer processTimer;
    processTimer.setIntervalUs(8 * 1000);
    // Display/diagnostic should happen once per second
    PicoPollTimer flashTimer;
    flashTimer.setIntervalUs(1000 * 1000);

    // ===== Main Event Loop =================================================

    while (true) {

        // Here is where we process inbound I/Q data and produce DAC data
        if (processTimer.poll()) {
            timer_0.reset();
            process_in_frame();           
            if (timer_0.elapsedUs() > max_proc_0) 
                max_proc_0 = timer_0.elapsedUs();
        }

        // Do periodic display/diagnostic stuff
        if (flashTimer.poll()) {

            ++strobe;
            if (strobe & 1 == 1) {
                gpio_put(LED_PIN, 1);
            } else {
                gpio_put(LED_PIN, 0);
            }

            cf32 work[work_size];
            float max_mag = 0, min_mag = 0;

            // Analyze the SSB buffer
            for (int i = 0; i < work_size; i++) {               
                work[i].r = lpf_buffer[i];
                work[i].i = 0;
                if (lpf_buffer[i] > max_mag)
                    max_mag = lpf_buffer[i];
                if (lpf_buffer[i] < min_mag)
                    min_mag = lpf_buffer[i];
            }
            fft.transform(work);
            uint maxIdx = maxMagIdx(work, 0, work_size / 2);

            uint32_t dac_buffer_depth;
            if (dac_buffer_wr_ptr > dac_buffer_rd_ptr)
                dac_buffer_depth = dac_buffer_wr_ptr - dac_buffer_rd_ptr;
            else 
                dac_buffer_depth = (DAC_BUFFER_SIZE - dac_buffer_rd_ptr) + dac_buffer_wr_ptr;

            //printf("Max/min mag %d/%d, FFT max bin %d, FFT mag %d\n", 
            //    (int)max_mag, (int)min_mag, maxIdx, (int)work[maxIdx].mag());
            printf("DACTick=%d, DACBWR=%d, DACBEM=%d, DACBD=%d\n", 
                dac_tick_count, dac_buffer_write_count, dacBufferEmpty, dac_buffer_depth); 
            //printf("DACTick=%d, DACBufferWrite=%d, DAC_empty=%d, %d, %d, %d, %d\n", 
            //    dac_tick_count, dac_buffer_write_count, dacBufferEmpty, 
            //    diag_count_0, diag_count_1, diag_count_2, max_proc_0);

            /*
            // Spectrum
            printf("<PLOT00>:");
            for (unsigned int i = 0; i < work_size / 2; i++) {
                if (i > 0)
                    printf(",");
                printf("%d", (int)work[i].mag());
            }
            printf("\n");
            */
        }
   }

    return 0;
}