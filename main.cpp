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

#include <arm_math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"

#include "kc1fsz-tools/rp2040/PicoPollTimer.h"
#include "kc1fsz-tools/rp2040/PicoPerfTimer.h"

#include "i2s.pio.h"
#include "si5351.h"
#include "sweeper.h"

#define I2C0_SDA_PIN (16)  // Physical pin 21
#define I2C0_SCL_PIN (17)  // Physical pin 22
#define I2C1_SDA_PIN (2)  
#define I2C1_SCL_PIN (3)  
#define LED_PIN (PICO_DEFAULT_LED_PIN)

using namespace kc1fsz;

//#define ADC_SAMPLE_COUNT (512)
//#define ADC_SAMPLE_BYTES_LOG2 (12)
//#define DAC_SAMPLE_BYTES_LOG2 (12)

#define ADC_SAMPLE_COUNT (256)
#define ADC_SAMPLE_BYTES_LOG2 (11)
#define DAC_SAMPLE_BYTES_LOG2 (11)

// Buffer used to drive the DAC via DMA.
// 2* for L and R
#define DAC_BUFFER_SIZE (ADC_SAMPLE_COUNT * 2)
// 4* for 32-bit integers
#define DAC_BUFFER_ALIGN (DAC_BUFFER_SIZE * 4)
// 4096-byte alignment is needed because we are using a DMA channel in ring mode
// and all buffers must be aligned to a power of two boundary.
// 256 samples * 2  words per sample * 4 bytes per word = 2048 bytes
static __attribute__((aligned(DAC_BUFFER_ALIGN))) uint32_t dac_buffer_ping[DAC_BUFFER_SIZE];
static __attribute__((aligned(DAC_BUFFER_ALIGN))) uint32_t dac_buffer_pong[DAC_BUFFER_SIZE];

// Buffer used to drive the ADC via DMA.
// When running at 48kHz, each buffer of 384 samples represents 8ms of activity
// When running at 48kHz, each buffer of 512 samples represents 10ms of activity
// The *2 accounts for left + right channels
#define ADC_BUFFER_SIZE (ADC_SAMPLE_COUNT * 2)
// Here is where the actual audio data gets written
// The *2 accounts for the fact that we are double-buffering.
static __attribute__((aligned(8))) uint32_t adc_buffer[ADC_BUFFER_SIZE * 2];
// Here is where the buffer addresses are stored to control ADC DMA
// The *2 accounts for the fact that we are double-buffering.
static __attribute__((aligned(8))) uint32_t* adc_addr_buffer[2];

// Diagnostic counters
static volatile uint32_t dma_in_count = 0;
static volatile uint32_t dma_out_count = 0;
static volatile uint32_t proc_count = 0;
static volatile uint32_t max_proc_0 = 0;

static uint dma_ch_in_ctrl = 0;
static uint dma_ch_in_data = 0;
static uint dma_ch_out_data0 = 0;
static uint dma_ch_out_data1 = 0;

// Enabled inside ADC DMA IRQ to indicate that a new frame is available
static volatile bool adc_frame_ready = false;
// This flag is used to manage the alternating buffers. "ping open"
// indicates that the ping buffer was just written and should be sent
// out on the next opportunity. Otherwise, it's the pong buffer that
// was just written and is waiting to be sent.
static volatile bool dac_buffer_ping_open = false;

static PicoPerfTimer timer_0;

#define HILBERT_IMPULSE_SIZE (51)
static const float hilbert_impulse[HILBERT_IMPULSE_SIZE] = {
0.012442742396853695, -9.247429900666847e-16, 0.009007745694645828, 3.886440143014065e-16, 0.012248270003437301, 2.0291951727332382e-16, 0.016267702606548327, 1.073881019267362e-15, 0.021262591317853616, -1.5445700787658912e-15, 0.027540135461651433, 9.040409971208902e-16, 0.03555144114893796, -9.736771998276128e-17, 0.04616069183085064, 2.5884635757740463e-16, 0.060879120316970736, -1.9452769758181735e-16, 0.08310832605502039, 6.688463706336815e-16, 0.12163479949392442, -4.100082211906412e-17, 0.208751841831881, 3.007875123018464e-16, 0.635463817659196, 0, -0.635463817659196, -3.007875123018464e-16, -0.208751841831881, 4.100082211906412e-17, -0.12163479949392442, -6.688463706336815e-16, -0.08310832605502039, 1.9452769758181735e-16, -0.060879120316970736, -2.5884635757740463e-16, -0.04616069183085064, 9.736771998276128e-17, -0.03555144114893796, -9.040409971208902e-16, -0.027540135461651433, 1.5445700787658912e-15, -0.021262591317853616, -1.073881019267362e-15, -0.016267702606548327, -2.0291951727332382e-16, -0.012248270003437301, -3.886440143014065e-16, -0.009007745694645828, 9.247429900666847e-16, -0.012442742396853695
};

/*
#define HILBERT_SIZE (71)
static float hilbert_impulse[HILBERT_SIZE] = {
0.003280158972196459, -2.6913320235605067e-07, 0.00281406062078617, -9.161363096266983e-07, 0.003985021414434486, -1.618587666932063e-06, 0.005450999350074075, -1.9189040647516074e-06, 0.007262664727861606, -2.360314424730244e-06, 0.00948053959593801, -2.1703343573990264e-06, 0.012181883403014827, -2.5628817403290866e-06, 0.015461200723989862, -7.066359931599645e-07, 0.019459515188764917, -3.2918847746577675e-06, 0.024362576948291248, -3.386452949263455e-06, 0.030473764476294157, -3.5184352949003947e-06, 0.0382687121226955, -4.391933037083758e-06, 0.04857005331189018, -3.953559009023066e-06, 0.06294804140744603, -2.314572775720779e-06, 0.08477848616922334, -1.6697579050910363e-06, 0.12285835532250555, -1.5223564667124609e-06, 0.2095022309647281, -1.9124822250834506e-06, 0.6357140091664184, 0, -0.6357140091664184, 1.9124822250834506e-06, -0.2095022309647281, 1.5223564667124609e-06, -0.12285835532250555, 1.6697579050910363e-06, -0.08477848616922334, 2.314572775720779e-06, -0.06294804140744603, 3.953559009023066e-06, -0.04857005331189018, 4.391933037083758e-06, -0.0382687121226955, 3.5184352949003947e-06, -0.030473764476294157, 3.386452949263455e-06, -0.024362576948291248, 3.2918847746577675e-06, -0.019459515188764917, 7.066359931599645e-07, -0.015461200723989862, 2.5628817403290866e-06, -0.012181883403014827, 2.1703343573990264e-06, -0.00948053959593801, 2.360314424730244e-06, -0.007262664727861606, 1.9189040647516074e-06, -0.005450999350074075, 1.618587666932063e-06, -0.003985021414434486, 9.161363096266983e-07, -0.00281406062078617, 2.6913320235605067e-07, -0.003280158972196459
};
*/

// Build the group-delay in samples.
// This is assuming an odd Hilbert size.
// NOTE: The +1 seems to give slightly better rejection
#define HILBERT_GROUP_DELAY ((HILBERT_IMPULSE_SIZE + 1) / 2) + 2
//#define HILBERT_GROUP_DELAY ((HILBERT_IMPULSE_SIZE + 1) / 2) - 1

/*
#define LPF_IMPULSE_SIZE (51)
static float lpf_impulse[LPF_IMPULSE_SIZE] = {
-0.007988518488193047, -0.0322923540338797, -0.0025580477346377516, -0.007508482037031126, -0.0015547763074605164, 0.003280642290253403, 0.008795348444624476, 0.013112016273176279, 0.015229588003704697, 0.014260356689224188, 0.009841832366608681, 0.002295390261866158, -0.007296945242233224, -0.01717660135976687, -0.025298778719890665, -0.029398991822789773, -0.027623603206855935, -0.018731596787559205, -0.0025587380614820365, 0.020105815714798622, 0.04727569765636943, 0.07608694879779716, 0.10336445155058628, 0.12570901987001032, 0.14037248973886046, 0.14550041816800013, 0.14037248973886046, 0.12570901987001032, 0.10336445155058628, 0.07608694879779716, 0.04727569765636943, 0.020105815714798622, -0.0025587380614820365, -0.018731596787559205, -0.027623603206855935, -0.029398991822789773, -0.025298778719890665, -0.01717660135976687, -0.007296945242233224, 0.002295390261866158, 0.009841832366608681, 0.014260356689224188, 0.015229588003704697, 0.013112016273176279, 0.008795348444624476, 0.003280642290253403, -0.0015547763074605164, -0.007508482037031126, -0.0025580477346377516, -0.0322923540338797, -0.007988518488193047
};
*/

#define LPF_IMPULSE_SIZE (91)
static float lpf_impulse[LPF_IMPULSE_SIZE] = {
0.026023731209180712, -0.014944709238861435, -0.01282048293469228, -0.011606039275329378, -0.010593560098382651, -0.00924125050458392, -0.007286060750618341, -0.004648438806810258, -0.0015355909555464862, 0.00171013338400933, 0.0045964325270102255, 0.006672390629168848, 0.007523840203798958, 0.006943597991965513, 0.004907442913820445, 0.0017317488367759044, -0.002112573413878447, -0.005956312807227829, -0.00905892246527812, -0.010785308726788935, -0.010650672953206002, -0.008495597575241154, -0.004513733345988606, 0.0007693820564916878, 0.006489959083732819, 0.011662090756684888, 0.01522931867508315, 0.01634570240113426, 0.014472103355133298, 0.009568210608822089, 0.0020876523550769467, -0.006951746454653754, -0.01615006622499047, -0.023815670331730376, -0.028272268352863006, -0.02806697519845502, -0.02215747488201716, -0.010297917934256877, 0.007255778779401038, 0.029207770582472974, 0.05373554159034625, 0.0786499018999863, 0.10140628513007802, 0.1196538139172792, 0.13142332055479838, 0.13550049513969512, 0.13142332055479838, 0.1196538139172792, 0.10140628513007802, 0.0786499018999863, 0.05373554159034625, 0.029207770582472974, 0.007255778779401038, -0.010297917934256877, -0.02215747488201716, -0.02806697519845502, -0.028272268352863006, -0.023815670331730376, -0.01615006622499047, -0.006951746454653754, 0.0020876523550769467, 0.009568210608822089, 0.014472103355133298, 0.01634570240113426, 0.01522931867508315, 0.011662090756684888, 0.006489959083732819, 0.0007693820564916878, -0.004513733345988606, -0.008495597575241154, -0.010650672953206002, -0.010785308726788935, -0.00905892246527812, -0.005956312807227829, -0.002112573413878447, 0.0017317488367759044, 0.004907442913820445, 0.006943597991965513, 0.007523840203798958, 0.006672390629168848, 0.0045964325270102255, 0.00171013338400933, -0.0015355909555464862, -0.004648438806810258, -0.007286060750618341, -0.00924125050458392, -0.010593560098382651, -0.011606039275329378, -0.01282048293469228, -0.014944709238861435, 0.026023731209180712
};

// State buffers/etc for FIR filters
static float hilbert_delay_state[HILBERT_IMPULSE_SIZE + ADC_SAMPLE_COUNT - 1];
static float hilbert_fir_state[HILBERT_IMPULSE_SIZE + ADC_SAMPLE_COUNT - 1];
static float lpf_fir_state[LPF_IMPULSE_SIZE + ADC_SAMPLE_COUNT - 1];
static arm_fir_instance_f32 hilbert_fir;
static arm_fir_instance_f32 lpf_fir;
static arm_cfft_instance_f32 audio_fft;

static float tx_lpf_fir_state[LPF_IMPULSE_SIZE + ADC_SAMPLE_COUNT - 1];
static float tx_hilbert_delay_state[HILBERT_IMPULSE_SIZE + ADC_SAMPLE_COUNT - 1];
static float tx_hilbert_fir_state[HILBERT_IMPULSE_SIZE + ADC_SAMPLE_COUNT - 1];
static arm_fir_instance_f32 tx_lpf_fir;
static arm_fir_instance_f32 tx_hilbert_fir;

//static int32_t freq = 7200000;
static int32_t freq = 7255000;
// Calibration
static int32_t cal = 490;
// LSB/USB selector
static bool modeLSB = true;
//static bool modeLSB = false;
static bool modeTX = true;

static float dacScale = 100.0;
static float txDacScale = 1.0;
// Used to trim I/Q imbalance on input
static float imbalanceScale = 0.97;

// ----- Sweeper Integration ------------------------------------------

/*
class SweeperContextImpl : public SweeperContext {
public:

    void setFreq(unsigned int freqHz) {
        si_evaluate(0, freqHz + cal);
    }

    unsigned int getRMS() const {
        float total = 0;
        for (unsigned int i = 0; i < AN_BUFFER_SIZE; i++) {
            total += pow(lpf_buffer[i], 2.0);
        }
        return sqrt(total);
    }
};
*/

static void process_in_frame_rx();
static void process_in_frame_tx();

// This will be called once every AUDIO_BUFFER_SIZE/2 samples.
// VERY IMPORTANT: This interrupt handler needs to be fast enough 
// to run inside of one sample block.
static void dma_adc_handler() {   

    dma_in_count++;
    adc_frame_ready = true;

    timer_0.reset();
    if (!modeTX)
        process_in_frame_rx();
    else
        process_in_frame_tx();
    if (timer_0.elapsedUs() > max_proc_0) 
        max_proc_0 = timer_0.elapsedUs();

    // Clear the IRQ status
    dma_hw->ints0 = 1u << dma_ch_in_data;
}

static void dma_dac0_handler() {   

    dma_out_count++;
    dac_buffer_ping_open = true;

    // Clear the IRQ status
    dma_hw->ints0 = 1u << dma_ch_out_data0;
}

static void dma_dac1_handler() {   

    dma_out_count++;
    dac_buffer_ping_open = false;

    // Clear the IRQ status
    dma_hw->ints0 = 1u << dma_ch_out_data1;
}

static void dma_irq_handler() {   
    // Figure out which interrupt fired
    if (dma_hw->ints0 & (1u << dma_ch_in_data)) {
        dma_adc_handler();
    }
    if (dma_hw->ints0 & (1u << dma_ch_out_data0)) {
        dma_dac0_handler();
    }
    if (dma_hw->ints0 & (1u << dma_ch_out_data1)) {
        dma_dac1_handler();
    }
}

// RX mode processing buffers
// (Pulled outside to enable introspection)
static float an1_i[ADC_SAMPLE_COUNT];
static float an1_q[ADC_SAMPLE_COUNT];
static float an2_i[ADC_SAMPLE_COUNT];
static float an2_q[ADC_SAMPLE_COUNT];
static float an3_ssb[ADC_SAMPLE_COUNT];
static float an4_audio[ADC_SAMPLE_COUNT];
static bool overflow = false;

// -----------------------------------------------------------------------------
// IMPORTANT FUNCTION: 
//
// This should be called in receive mode when a complete frame of I/Q 
// data has been converted.
//
// Benchmark: Approximately 1ms per call, using a 51 tap Hilbert transform
// and a 91 tap low pass filter.
//
static void process_in_frame_rx() {

    proc_count++;

    // Counter used to alternate between double-buffer sides
    static uint32_t dma_count_0 = 0;

    // Figure out which part of the double-buffer we just finished
    // loading into.  Notice: the pointer is signed.
    int32_t* adc_data;
    if (dma_count_0 % 2 == 0) {
        adc_data = (int32_t*)adc_buffer;
    } else {
        adc_data = (int32_t*)&(adc_buffer[ADC_BUFFER_SIZE]);
    }
    dma_count_0++;

    // Move from the DMA buffer and into analysis buffer.  This also 
    // separates the I/Q streams, and corrects the scaling.
    unsigned int j = 0;
    for (unsigned int i = 0; i < ADC_BUFFER_SIZE; i += 2) {
        // The 24-bit signed value is left-justified in the 32-bit word, 
        // so we need to shift right 8. Sign extension is automatic.
        // Range of 24 bits is -8,388,608 to 8,388,607.
        an1_i[j] = adc_data[i] >> 8;
        an1_q[j] = adc_data[i + 1] >> 8;
        j++;
    }

    // Create a delayed version of the I stream
    // TODO: PACKAGE INTO A FUNCTION WITH SAME SIGNATURE AS arm_fir_32()
    // TODO: MAKE A DIAGRAM
    memmove((void*)hilbert_delay_state, 
        (const void*)&(hilbert_delay_state[ADC_SAMPLE_COUNT]), 
        (HILBERT_IMPULSE_SIZE - 1) * sizeof(float));
    memmove((void*)&(hilbert_delay_state[HILBERT_IMPULSE_SIZE - 1]), 
        (const void*)an1_i,
        ADC_SAMPLE_COUNT * sizeof(float));
    for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) 
        an2_i[i] = hilbert_delay_state[(HILBERT_IMPULSE_SIZE - 1) - HILBERT_GROUP_DELAY + i];

    // Apply the Hilbert transform of the Q stream
    arm_fir_f32(&hilbert_fir, an1_q, an2_q, ADC_SAMPLE_COUNT); 

    // Make the SSB by combining the I and Q streams
    for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) 
        if (modeLSB) {
            an3_ssb[i] = an2_i[i] - an2_q[i];
        } else {
            an3_ssb[i] = an2_i[i] + an2_q[i];
        }

    // Apply the LPF to the selected sideband
    arm_fir_f32(&lpf_fir, an3_ssb, an4_audio, ADC_SAMPLE_COUNT);

    // Write to the DAC buffer based on our current tracking of which 
    // is available for use.
    int32_t* dac_buffer;
    if (dac_buffer_ping_open) 
        dac_buffer = (int32_t*)dac_buffer_ping;
    else
        dac_buffer = (int32_t*)dac_buffer_pong;

    // Note that we are only writing to the left DAC channel.
    j = 0;
    for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        float fScaled = (an4_audio[i] * dacScale);
        // 24 bit signed
        if (fabs(fScaled) > 8388607.0) {
            overflow = true;
            fScaled = 0;
        }
        int32_t aScaled = fScaled;
        aScaled = aScaled << 8;
        // Right 
        dac_buffer[j++] = 0;
        // Left
        dac_buffer[j++] = aScaled;
    }
}

// TX mode processing buffers
// (Pulled outside to enable introspection)
static float tx_an1[ADC_SAMPLE_COUNT];
static float tx_an2[ADC_SAMPLE_COUNT];
static float tx_an3_i[ADC_SAMPLE_COUNT];
static float tx_an3_q[ADC_SAMPLE_COUNT];
static bool txOverflow = false;

// -----------------------------------------------------------------------------
// IMPORTANT FUNCTION: 
//
// This is called in transmit mode each time a complete frame of audio data has
// been received from the audio source (e.g. microphone). This performs the DSP
// math and creates I/Q streams that are sent to the DAC for modulation/transmission.
//
// EXTREMELY TIME-SENSITIVE, so be careful about adding too much stuff here.
// This needs to execute inside of one audio block's time.
//
static void process_in_frame_tx() {

    proc_count++;

    // Counter used to alternate between double-buffer sides
    static uint32_t dma_count_0 = 0;

    // Figure out which part of the double-buffer we just finished
    // loading into.  Notice: the pointer is signed.
    int32_t* adc_data;
    if (dma_count_0 % 2 == 0) {
        adc_data = (int32_t*)adc_buffer;
    } else {
        adc_data = (int32_t*)&(adc_buffer[ADC_BUFFER_SIZE]);
    }
    dma_count_0++;

    // Move from the DMA buffer and into analysis buffer. 
    unsigned int j = 0;
    for (unsigned int i = 0; i < ADC_BUFFER_SIZE; i += 2) {
        // The 24-bit signed value is left-justified in the 32-bit word, 
        // so we need to shift right 8. Sign extension is automatic.
        // Range of 24 bits is -8,388,608 to 8,388,607.
        //
        // Pulling from the left channel
        tx_an1[j] = adc_data[i + 1] >> 8;
        j++;
    }

    // TODO: Review filter shape - need something different on TX?
    // Low-pass filter the audio input
    arm_fir_f32(&tx_lpf_fir, tx_an1, tx_an2, ADC_SAMPLE_COUNT);

    // Create a delayed version of the audio to get the I stream
    // TODO: PACKAGE INTO A FUNCTION WITH SAME SIGNATURE AS arm_fir_32()
    // TODO: MAKE A DIAGRAM
    memmove((void*)tx_hilbert_delay_state, 
        (const void*)&(tx_hilbert_delay_state[ADC_SAMPLE_COUNT]), 
        (HILBERT_IMPULSE_SIZE - 1) * sizeof(float));
    memmove((void*)&(tx_hilbert_delay_state[HILBERT_IMPULSE_SIZE - 1]), 
        (const void*)tx_an2,
        ADC_SAMPLE_COUNT * sizeof(float));
    for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) 
        tx_an3_i[i] = tx_hilbert_delay_state[(HILBERT_IMPULSE_SIZE - 1) - HILBERT_GROUP_DELAY + i];

    // Apply the Hilbert transform of the audio to get the Q stream
    arm_fir_f32(&tx_hilbert_fir, tx_an2, tx_an3_q, ADC_SAMPLE_COUNT); 

    // Write to the DAC buffer based on our current tracking of which 
    // is available for use.
    int32_t* dac_buffer;
    if (dac_buffer_ping_open) 
        dac_buffer = (int32_t*)dac_buffer_ping;
    else
        dac_buffer = (int32_t*)dac_buffer_pong;

    j = 0;
    for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) {

        float scaledI = (tx_an3_i[i] * txDacScale);
        // 24 bit signed
        if (fabs(scaledI) > 8388607.0) {
            txOverflow = true;
        }
        int32_t aScaledI = scaledI;
        aScaledI = aScaledI << 8;

        float scaledQ = (tx_an3_q[i] * txDacScale);
        // 24 bit signed
        if (fabs(scaledQ) > 8388607.0) {
            txOverflow = true;
        }
        int32_t aScaledQ = scaledQ;
        aScaledQ = aScaledQ << 8;

        // Right 
        dac_buffer[j++] = aScaledI;
        // Left
        dac_buffer[j++] = aScaledQ;
    }
}

// TEMP
float phaseAdjust = 0.24;
//float PIPI = 3.14159265359;

static void generateTestTone() {


    float fs = 48000;
    float omega = 2.0 * PI / fs;
    float fIncrement = fs / (ADC_SAMPLE_COUNT);
    float ft = fIncrement * 7;
    omega *= ft;
    float phi = 0;
    float a = 4000000.0;
    float phase;
    
    if (!modeLSB)
        phase = PI / 2.0 + phaseAdjust;
    else 
        phase = -(PI / 2.0 - phaseAdjust);

    for (unsigned int i = 0; i < DAC_BUFFER_SIZE; i += 2) {
            
        float c0 = imbalanceScale * a * cos(phi);
        int32_t c1 = c0;
        // (Left)
        dac_buffer_ping[i + 1] = c1 << 8;
        dac_buffer_pong[i + 1] = c1 << 8;

        float d0 = a * cos(phi - phase);
        int32_t d1 = d0;

        // (Right)
        dac_buffer_ping[i] = d1 << 8;
        dac_buffer_pong[i] = d1 << 8;
        phi += omega;
    }
}

void init_si5351() {
    // We are using I2C0 here!
    printf("Si5351 initializing 0\n");
    si_init(i2c0); 
    printf("Si5351 initializing 1\n");
    si_enable(0, true);
}

int main(int argc, const char** argv) {

    // Get all of the ARM CMSIS-DSP structures setup
    arm_fir_init_f32(&hilbert_fir, 
        HILBERT_IMPULSE_SIZE, hilbert_impulse, hilbert_fir_state, ADC_SAMPLE_COUNT);
    arm_fir_init_f32(&lpf_fir, 
        LPF_IMPULSE_SIZE, lpf_impulse, lpf_fir_state, ADC_SAMPLE_COUNT);
    //arm_cfft_init_512_f32(&audio_fft);
    arm_cfft_init_256_f32(&audio_fft);

    arm_fir_init_f32(&tx_hilbert_fir, 
        HILBERT_IMPULSE_SIZE, hilbert_impulse, tx_hilbert_fir_state, ADC_SAMPLE_COUNT);
    arm_fir_init_f32(&tx_lpf_fir, 
        LPF_IMPULSE_SIZE, lpf_impulse, tx_lpf_fir_state, ADC_SAMPLE_COUNT);

    unsigned long fs = 48000;

    // Pin to be allocated to I2S SCK (output to CODEC)
    // GP10 is physical pin 14
    uint sck_pin = 4;
    // ADC pins
    // Pin to be allocated to ~RST
    uint rst_pin = 5;
    // Pin to be allocated to I2S DIN (input from)
    uint din_pin = 6;
    // DAC pins
    // Pin to be allocated to I2S DOUT 
    uint dout_pin = 9;

    unsigned long system_clock_khz = 129600;

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
    i2c_set_baudrate(i2c0, 100000);

    //i2c_init(i2c1, 100 * 1000);
    //gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    //gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    // NOTE: The Sparkfun MCP4725 breakout board has 4.7k pullups on the 
    // I2C lines.  Therefore we are not enabling them here.
    //gpio_pull_up(I2C1_SDA_PIN);
    //gpio_pull_up(I2C1_SCL_PIN);
    //i2c_set_baudrate(i2c1, 800000);

    // Startup ID
    sleep_ms(500);
    sleep_ms(500);

    printf("Minimal SDR2\nCopyright (C) Bruce MacKinnon KC1FSZ, 2025\n");
    printf("Freq %d\n", freq);

    generateTestTone();

    // ===== Si5351 Initialization =============================================

    init_si5351();
    // Change freq
    printf("Si5351 initializing 2\n");
    si_evaluate(0, freq + cal);
    printf("Si5351 initialized 3\n");

    //const uint work_size = 256;
    //float trigSpace[work_size];
    //F32FFT fft(work_size, trigSpace);

    // ===== PCM1804 A/D Converter Setup ======================================

    // TODO: REVIEW WHETHER THIS IS NEEDED
    // Reset the CODEC
    gpio_put(rst_pin, 0);
    sleep_ms(100);
    gpio_put(rst_pin, 1);
    sleep_ms(100);

    // ===== PCM5100 DAC Setup ===============================================

    // ===== I2S SCK PIO Setup ===============================================

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

    // ===== I2S LRCK, BCK, DIN Setup (ADC) =====================================

    // Allocate state machine
    uint din_sm = pio_claim_unused_sm(pio0, true);
    uint din_sm_mask = 1 << din_sm;
    
    // Load master ADC program into the PIO
    uint din_program_offset = pio_add_program(pio0, &i2s_din_master_program);
  
    // Setup the function select for a GPIO to use from the given PIO 
    // instance.
    // DIN
    pio_gpio_init(pio0, din_pin);
    gpio_set_pulls(din_pin, false, false);
    gpio_set_dir(din_pin, GPIO_IN);
    // BCK
    pio_gpio_init(pio0, din_pin + 1);
    gpio_set_dir(din_pin + 1, GPIO_OUT);
    // LRCK
    pio_gpio_init(pio0, din_pin + 2);
    gpio_set_dir(din_pin + 2, GPIO_OUT);

    // NOTE: The xxx_get_default_config() function is generated by the PIO
    // assembler and defined inside of the generated .h file.
    pio_sm_config din_sm_config = 
        i2s_din_master_program_get_default_config(din_program_offset);
    // Associate the input pin with state machine.  This will be 
    // relevant to the DIN pin for IN instructions.
    sm_config_set_in_pins(&din_sm_config, din_pin);
    // Set the "side set pins" for the state machine. 
    // These are BCLK and LRCLK
    sm_config_set_sideset_pins(&din_sm_config, din_pin + 1);
    // Configure the IN shift behavior.,,,,,
    // Parameter 0: "false" means shift ISR to left on input.
    // Parameter 1: "true" means autopush is enabled.
    // Parameter 2: "32" means threshold (in bits) before auto/conditional 
    //              push to the ISR.
    sm_config_set_in_shift(&din_sm_config, false, true, 32);
    // Merge the FIFOs since we are only doing RX.  This gives us 
    // 8 words of buffer instead of the usual 4.
    sm_config_set_fifo_join(&din_sm_config, PIO_FIFO_JOIN_RX);

    // Initialize the direction of the pins before SM is enabled
    // There are three pins in the mask here. 
    uint din_pins_mask = 0b111 << din_pin;
    // DIN: The "0" means input, "1" means output
    uint din_pindirs   = 0b110 << din_pin;
    pio_sm_set_pindirs_with_mask(pio0, din_sm, din_pindirs, din_pins_mask);
    // Start with the two clocks in 1 state.
    uint din_pinvals   = 0b110 << din_pin;
    pio_sm_set_pins_with_mask(pio0, din_sm, din_pinvals, din_pins_mask);

    // Hook it all together.  (But this does not enable the SM!)
    pio_sm_init(pio0, din_sm, din_program_offset, &din_sm_config);
          
    // Adjust state-machine clock divisor.  
    // NOTE: The clock divisor is in 16:8 format
    //
    // d d d d d d d d d d d d d d d d . f f f f f f f f
    //            Integer Part         |  Fraction Part
    // 
    pio_sm_set_clkdiv_int_frac(pio0, din_sm, 21, 24);
    
    // ----- ADC DMA setup ---------------------------------------

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

    // Enable interrupt when DMA data transfer completes via
    // the DMA_IRQ0.
    dma_channel_set_irq0_enabled(dma_ch_in_data, true);

    // ===== I2S DOUT/BCK/LRCK PIO Setup (To DAC) ==============================

    // Allocate state machine
    uint dout_sm = pio_claim_unused_sm(pio0, true);
    uint dout_sm_mask = 1 << dout_sm;

    // Load master ADC program into the PIO
    uint dout_program_offset = pio_add_program(pio0, &i2s_dout_master_program);
  
    // Setup the function select for a GPIO to use from the given PIO 
    // instance: DOUT
    pio_gpio_init(pio0, dout_pin);
    gpio_set_dir(dout_pin, GPIO_OUT);
    // BCK
    pio_gpio_init(pio0, dout_pin + 1);
    gpio_set_dir(dout_pin + 1, GPIO_OUT);
    // LRCK
    pio_gpio_init(pio0, dout_pin + 2);
    gpio_set_dir(dout_pin + 2, GPIO_OUT);

    // NOTE: The xxx_get_default_config() function is generated by the PIO
    // assembler and defined inside of the generated .h file.
    pio_sm_config dout_sm_config = 
        i2s_dout_master_program_get_default_config(dout_program_offset);
    // Associate the input pin with state machine.  This will be 
    // relevant to the DOUT pin for OUT instructions.
    sm_config_set_out_pins(&dout_sm_config, dout_pin, 1);
    // Set the "side set pins" for the state machine. 
    // These are BCLK and LRCLK
    sm_config_set_sideset_pins(&dout_sm_config, dout_pin + 1);
    // Configure the OUT shift behavior.
    // Parameter 0: "false" means shift OSR to left on input.
    // Parameter 1: "true" means autopull is enabled.
    // Parameter 2: "32" means threshold (in bits) before auto/conditional 
    //              pull to the OSR.
    sm_config_set_out_shift(&dout_sm_config, false, true, 32);
    // Merge the FIFOs since we are only doing TX.  This gives us 
    // 8 words of buffer instead of the usual 4.
    sm_config_set_fifo_join(&dout_sm_config, PIO_FIFO_JOIN_TX);

    // Initialize the direction of the pins before SM is enabled
    uint dout_pins_mask = 0b111 << dout_pin;
    // DIN: The "0" means input, "1" means output
    uint dout_pindirs   = 0b111 << dout_pin;
    pio_sm_set_pindirs_with_mask(pio0, dout_sm, dout_pindirs, dout_pins_mask);
    // Start with the two clocks in 1 state.
    uint dout_pinvals   = 0b110 << dout_pin;
    pio_sm_set_pins_with_mask(pio0, dout_sm, dout_pinvals, dout_pins_mask);

    // Hook it all together.  (But this does not enable the SM!)
    pio_sm_init(pio0, dout_sm, dout_program_offset, &dout_sm_config);
          
    // Adjust state-machine clock divisor.  
    // NOTE: The clock divisor is in 16:8 format
    //
    // d d d d d d d d d d d d d d d d . f f f f f f f f
    //            Integer Part         |  Fraction Part
    // 
    // TODO: USE VARIABLES SHARED WITH DIN
    pio_sm_set_clkdiv_int_frac(pio0, dout_sm, 21, 24);
    
    dma_ch_out_data0 = dma_claim_unused_channel(true);
    dma_ch_out_data1 = dma_claim_unused_channel(true);

    // ----- DAC DMA channel 0 setup -------------------------------------

    cfg = dma_channel_get_default_config(dma_ch_out_data0);
    // We need to increment the read to move across the buffer
    channel_config_set_read_increment(&cfg, true);
    // Define wrap-around ring (read). Per Pico SDK docs:
    //
    // "For values n > 0, only the lower n bits of the address will change. This 
    // wraps the address on a (1 << n) byte boundary, facilitating access to 
    // naturally-aligned ring buffers. Ring sizes between 2 and 32768 bytes are 
    // possible (size_bits from 1 - 15)."
    //
    // WARNING: Make sure the buffer is sufficiently aligned for this to work!
    channel_config_set_ring(&cfg, false, DAC_SAMPLE_BYTES_LOG2);
    // No increment required because we are always writing to the 
    // PIO TX FIFO every time.
    channel_config_set_write_increment(&cfg, false);
    // Set size of each transfer (one audio word)
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    // Attach the DMA channel to the TX DREQ of the PIO state machine. 
    // The "true" below indicates TX.
    // This is the "magic" that connects the PIO SM to the DMA.
    channel_config_set_dreq(&cfg, pio_get_dreq(pio0, dout_sm, true));
    // We trigger the other data channel once the data transfer is done
    // in order to achieve the ping-pong effect.
    channel_config_set_chain_to(&cfg, dma_ch_out_data1);
    // Program the DMA channel
    dma_channel_configure(dma_ch_out_data0, &cfg,
        // Initial write address
        // The memory-mapped location of the RX FIFO of the PIO state
        // machine used for receiving data
        // This is the "magic" that connects the PIO SM to the DMA.
        &(pio0->txf[dout_sm]),
        // Initial Read address
        dac_buffer_ping, 
        // Number of transfers (each is 32 bits)
        DAC_BUFFER_SIZE,
        // Don't start yet
        false);
    // Enable interrupt when DMA data transfer completes
    dma_channel_set_irq0_enabled(dma_ch_out_data0, true);

    // ----- DAC DMA channel 1 setup -------------------------------------

    cfg = dma_channel_get_default_config(dma_ch_out_data1);
    // We need to increment the read to move across the buffer
    channel_config_set_read_increment(&cfg, true);
    // Define wrap-around ring (read)
    channel_config_set_ring(&cfg, false, DAC_SAMPLE_BYTES_LOG2);
    // No increment required because we are always writing to the 
    // PIO TX FIFO every time.
    channel_config_set_write_increment(&cfg, false);
    // Set size of each transfer (one audio word)
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    // Attach the DMA channel to the TX DREQ of the PIO state machine. 
    // The "true" below indicates TX.
    // This is the "magic" that connects the PIO SM to the DMA.
    channel_config_set_dreq(&cfg, pio_get_dreq(pio0, dout_sm, true));
    // We trigger the other data channel once the data transfer is done
    // in order to achieve the ping-pong effect.
    channel_config_set_chain_to(&cfg, dma_ch_out_data0);
    // Program the DMA channel
    dma_channel_configure(dma_ch_out_data1, &cfg,
        // Initial write address
        // The memory-mapped location of the RX FIFO of the PIO state
        // machine used for receiving data
        // This is the "magic" that connects the PIO SM to the DMA.
        &(pio0->txf[dout_sm]),
        // Initial Read address
        dac_buffer_pong, 
        // Number of transfers (each is 32 bits)
        DAC_BUFFER_SIZE,
        // Don't start yet
        false);
    // Enable interrupt when DMA data transfer completes
    dma_channel_set_irq0_enabled(dma_ch_out_data1, true);

    // ----- Final Enables ----------------------------------------------------

    // Bind to the interrupt handler 
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    // Enable DMA interrupts
    irq_set_enabled(DMA_IRQ_0, true);

    // Start ADC DMA action on the control side.  This will trigger
    // the ADC data DMA channel in turn.
    dma_channel_start(dma_ch_in_ctrl);
    // Start DAC DMA action immediately so the DAC FIFO is full
    // from the beginning.
    dma_channel_start(dma_ch_out_data0);

    // Stuff the TX FIFO to get going.  If the DAC state machine
    // gets started before there is anything in the FIFO we will
    // get stalled forever. 
    // TODO: EXPLAIN WHY
    uint fill_count = 0;
    while (pio0->fstat & 0x00040000 == 0) {
        fill_count++;
    }

    // Final enable of the three SMs to keep them in sync.
    pio_enable_sm_mask_in_sync(pio0, sck_sm_mask | din_sm_mask | dout_sm_mask);

    // Now issue a reset of the ADC
    //
    // Per datasheet page 18: "In slave mode, the system clock rate is automatically
    // detected."
    //
    // Per datasheet page 18: "The PCM1804 needs -RST=low when control pins
    // are changed or in slave mode when SCKI, LRCK, and BCK are changed."
    //
    // I believe these two statements imply that a -RST is needed after *all* of the
    // clocks are being driven at the target frequency.

    sleep_ms(100);
    gpio_put(rst_pin, 0);
    sleep_ms(100);
    gpio_put(rst_pin, 1);

    int strobe = 0;
    
    // Audio processing should happen once every DMA frame (10.6ms)
    // in order to avoid falling behind. We run a bit faster than than.
    PicoPollTimer processTimer;
    processTimer.setIntervalUs(2 * 1000);

    // Display/diagnostic should happen once per second
    PicoPollTimer flashTimer;
    flashTimer.setIntervalUs(1000 * 1000);

    // A timer used for diagnostic features
    PicoPollTimer sweepTimer;
    sweepTimer.setIntervalUs(100 * 1000);

    //SweeperState swState;
    //SweeperContextImpl swContext;

    // ===== Main Event Loop =================================================

    while (true) { 
        
        int c = getchar_timeout_us(0);
        if (c == '-') {
            freq = freq - 1000;
            si_evaluate(0, freq + cal);
            printf("Freq %d\n", freq);
        }
        else if (c == '=') {
            freq = freq + 1000;
            si_evaluate(0, freq + cal);
            printf("Freq %d\n", freq);
        }
        else if (c == '_') {
            freq = freq - 100;
            si_evaluate(0, freq + cal);
            printf("Freq %d\n", freq);
        }
        else if (c == '+') {
            freq = freq + 100;
            si_evaluate(0, freq + cal);
            printf("Freq %d\n", freq);
        }
        else if (c == 'a') {
            dacScale = dacScale + 5.0;
            printf("Scale %f\n", dacScale);
        }
        else if (c == 's') {
            dacScale = dacScale - 5.0;
            printf("Scale %f\n", dacScale);
        }
        else if (c == 'q' || c == 'w') {
            if (c == 'q')
                phaseAdjust -= 0.005;
            else
                phaseAdjust += 0.005;
            printf("Phase Adjust %f\n", phaseAdjust);
            generateTestTone();
        }
        else if (c == 'e' || c == 'r') {
            if (c == 'e')
                imbalanceScale -= 0.01;
            else
                imbalanceScale += 0.01;
            printf("imbalanceBalance %f\n", imbalanceScale);
            generateTestTone();
        }

        /*
        else if (c == 's') {
            swState.startHz = 7200000 - 5000;
            swState.endHz = 7200000 + 5000;
            swState.stepHz = 100;
            swState.state = SweeperState::State::READY;
            printf("Starting sweep ...");
        }
        else if (c == 't') {
            for (unsigned int i = 0; i < swState.sampleCount; i++) 
                printf("%d\n", swState.sample[i]);
        }
        */
        else if (c == 'p') {
            // Make a quick copy since this buffer is changing in real-time
            float copy[ADC_SAMPLE_COUNT / 4];
            for (unsigned int i = 0; i < ADC_SAMPLE_COUNT / 4; i++) {
                if (modeTX)
                    copy[i] = tx_an1[i];
                else
                    copy[i] = an1_i[i];
            }
            printf("\n\n\n======================================\n");
            for (unsigned int i = 0; i < ADC_SAMPLE_COUNT / 4; i++) 
                printf("%d\n", (int)copy[i]);
        }
        else if (c == 'q') {
            float copy[ADC_SAMPLE_COUNT];
            for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) {
                int32_t a = dac_buffer_pong[i * 2];
                a = a >> 8;
                copy[i] = a;
            }
            printf("\n\n\n======================================\n");
            for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) 
                printf("%d\n", (int)copy[i]);
        }
        else if (c == 'u')
            init_si5351();

        //if (sweepTimer.poll()) 
        //    sweeper_tick(&swState, &swContext);

        // Do periodic display/diagnostic stuff
        if (flashTimer.poll()) {

            ++strobe;
            if (strobe & 1 == 1) {
                gpio_put(LED_PIN, 1);
            } else {
                gpio_put(LED_PIN, 0);
            }

            //printf("Max time %d\n", max_proc_0);
            max_proc_0 = 0;
            //printf("IN/OUT/PROC %d/%d/%d\n", dma_in_count, dma_out_count, proc_count);
            //printf("FSTAT %08x, FDEBUG=%08x\n", pio0->fstat, pio0->fdebug);

            // Spectral analysis on final audio
            float work[ADC_SAMPLE_COUNT * 2];
            float maxAudio = 0;
            float power = 0;
            for (unsigned int i = 0; i < ADC_SAMPLE_COUNT; i++) {               
                float s;
                if (!modeTX)
                    s = an4_audio[i];
                else 
                    //s = tx_an1[i];
                    s = tx_an3_q[i];
                power += s * s;
                // Complex real/imaginary pair
                work[i * 2] = s;
                work[i * 2 + 1] = 0;
                if (s > maxAudio)
                    maxAudio = s;
            }
            power = sqrt(power);

            // FFT in place.
            // Argument 3: 0=forward FT
            // Argument 4: 1="Bit reversal", which indicates that the bins
            //   should be organized in the "natural" order, presumably with 
            //   some performance overhead.
            arm_cfft_f32(&audio_fft, work, 0, 1);

            float maxMagSquared = 0;
            unsigned int maxMagIdx = 0;
            // Only look at the positive frequencies, ignore DC
            for (int i = 0; i < ADC_SAMPLE_COUNT / 2; i++) {               
                float magSquared = work[i * 2] * work[i * 2] + work[i * 2 + 1] * work[i * 2 + 1];
                if (magSquared > maxMagSquared) {
                    maxMagSquared = magSquared;
                    maxMagIdx = i;
                }
            }
            float maxMag = sqrt(maxMagSquared);

            printf("Power %d, max %d, FFT max bin %d, FFT max mag %d\n", 
                (int)power, (int)maxAudio, maxMagIdx, (int)maxMag);

            if (overflow) {
                overflow = false;
                printf("Overflow\n");
            }
        }

        //process_in_frame();           
   }

    return 0;
}
