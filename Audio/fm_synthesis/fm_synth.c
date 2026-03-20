/**
 *  V. Hunter Adams (vha3@cornell.edu)
 
    This is an experiment with the multicore capabilities on the
    RP2040. The program instantiates a timer interrupt on each core.
    Each of these timer interrupts writes to a separate channel
    of the SPI DAC and does DDS of two sine waves of two different
    frequencies. These sine waves are amplitude-modulated to "beeps."

    No spinlock is required to mediate the SPI writes because of the
    SPI buffer on the RP2040. Spinlocks are used in the main program
    running on each core to lock the other out from an incrementing
    global variable. These are "under the hood" of the PT_SEM_SAFE_x
    macros. Two threads ping-pong using these semaphores.

    Note that globals are visible from both cores. Note also that GPIO
    pin mappings performed on core 0 can be utilized from core 1.
    Creation of an alarm pool is required to force a timer interrupt to
    take place on core 1 rather than core 0.

 */

// Include necessary libraries
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/adc.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
// #define mul(a,b) ((fix)(((( signed long long )(a))*(( signed long long )(b)))>>12)) //multiply two fixed 16:16
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 40000            // sample rate

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (2300.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sine_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 max_noise_amplitude = 180 ;    // maximum noise amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
volatile fix15 mod_attack_inc ;                      // rate at which sound ramps up
volatile fix15 mod_decay_inc ;                       // rate at which sound ramps down
fix15 noise_attack_inc ;                      // rate at which sound ramps up
fix15 noise_decay_inc ;                       // rate at which sound ramps down
fix15 pitch_attack_inc ;                      // rate at which sound ramps up
fix15 pitch_decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 noise_amplitude = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define NOTE_DELAY             1000

#define NOISE_ATTACK            250
#define PITCH_ATTACK            500
#define MOD_ATTACK_TIME         90
#define ATTACK_TIME             160

#define NOISE_DECAY             240
#define PITCH_DECAY             500
#define MOD_DECAY_TIME          320
#define DECAY_TIME              220

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int STATE_1 = 0 ;
volatile unsigned int count_0 = 0 ;
volatile unsigned int count_1 = 0 ;

// debouncing
absolute_time_t last_press_time;
const int debounce_ms = 60;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define PITCH     26
#define BUTTON   0
#define SPI_PORT spi0

// Two variables to store core number
volatile int corenum_0  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;

// adc pitch value
volatile float pitch = 0.1;

float note = 130.8;
volatile fix15 current_mod_depth ; 
volatile unsigned int mod_inc, main_inc ;
volatile unsigned int mod_accum, main_accum ;
volatile bool button_ready = true;
volatile fix15 max_mod_depth = 200000;
volatile fix15 current_mod_depth = 2000;
volatile fix15 max_pitch_bend = 8000;
volatile fix15 pitch_bend = 0;
fix15 octave_num = 4;
fix15 Fmod = 3.0;

fix15 mod_wave, main_wave ;


// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {

    // DDS phase and sine table lookup
    phase_accum_main_0 += phase_incr_main_0  ;
    // float swoop_freq = -260*sine_table[];

    float current_note = 250.0 + 900.0*pitch*pitch;

    Fmod = 3.0 + 1.0*pitch;
    main_inc = current_note * pow(2,32 )/ Fs ;
    mod_inc = Fmod * current_note * pow(2,32 )/ Fs ;

    // compute modulating wave
    mod_accum += mod_inc ;
    mod_wave = sine_table[mod_accum>>24] ;

    // set dds main freq and FM modulate it
    main_accum += main_inc + pitch_bend + (unsigned int) multfix15(mod_wave, current_mod_depth) ;
    // update main waveform
    main_wave = sine_table[main_accum>>24] + noise_amplitude*(rand() % 100 - 50);

    if (STATE_0 == 0){

        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            main_wave)) + 2048 ; // limit to amp

        if (count_0 < NOTE_DELAY) {
            current_amplitude_0 = 0 ;
        }

        // Ramp up amplitude
        else if (count_0 < ATTACK_TIME + NOTE_DELAY) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }

        if ( count_0 > NOTE_DELAY && count_0 < MOD_ATTACK_TIME + NOTE_DELAY) {
            current_mod_depth = current_mod_depth + mod_attack_inc;
        }

        if ( count_0 > NOTE_DELAY && count_0 < PITCH_ATTACK + NOTE_DELAY) {
            pitch_bend = pitch_bend + pitch_attack_inc;
        }

        else if ( count_0 > PITCH_ATTACK + NOTE_DELAY && count_0 < PITCH_ATTACK + NOTE_DELAY + PITCH_DECAY) {
            pitch_bend = pitch_bend - pitch_decay_inc;
        }

        if (count_0 < NOISE_ATTACK) {
            noise_amplitude = noise_amplitude + noise_attack_inc;
        }

        else if (count_0 < NOISE_ATTACK + NOISE_DECAY) {
            noise_amplitude = noise_amplitude - noise_decay_inc;
        }

        else {
            noise_amplitude = 0;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

    }


    // note ending
    if (STATE_0 == 1){

        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            main_wave)) + 2048 ; // limit to amp

        // Ramp down amplitude
        if (count_0 < DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        if (count_0 < MOD_DECAY_TIME) {
            current_mod_depth = current_mod_depth - mod_decay_inc ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;
        
        // State transition?
        if (count_0 == DECAY_TIME) {
            STATE_0 = 2 ;
        }

    }


    // retrieve core number of execution
    corenum_0 = get_core_num();

    return true;
}


void button_callback(uint gpio, uint32_t events) {
    static absolute_time_t last_time;

    absolute_time_t now = get_absolute_time();

    // debounce
    if (absolute_time_diff_us(last_time, now) < 5000) return;
    last_time = now;

    // read actual pin state
    if (gpio_get(0) && STATE_0 != 0) {
        // play the note
        count_0 = 0;
        mod_accum = 0;
        main_accum = 0;
        current_mod_depth = 0;
        current_amplitude_0 = 0;
        STATE_0 = 0;
    } else if (STATE_0 == 0) {
        //end the note
        count_0 = 0;
        STATE_0 = 1;
    }
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        pitch = (float)adc_read() / 4095.0f;
        // more modulation depth lower notes?
        max_mod_depth = 80000 + 9000*(1-pitch);
        mod_attack_inc = divfix(max_mod_depth, int2fix15(MOD_ATTACK_TIME));
        mod_decay_inc = divfix(max_mod_depth, int2fix15(MOD_DECAY_TIME)) ;
        PT_YIELD_usec(500) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}



// Core 0 entry point
int main() {
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();
    printf("Hello, friends!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Configure GPIO interrupt
    gpio_set_irq_enabled_with_callback(0, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &button_callback);
    // // Configure GPIO interrupt
    // gpio_set_irq_enabled_with_callback(0, GPIO_IRQ_EDGE_FALL, true, &end_note);

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    gpio_init(BUTTON);
    gpio_set_dir(BUTTON, GPIO_IN);
    gpio_pull_down(0);

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    noise_attack_inc = divfix(max_noise_amplitude, int2fix15(NOISE_ATTACK));
    noise_decay_inc = divfix(max_noise_amplitude, int2fix15(NOISE_DECAY)) ;

    pitch_attack_inc = divfix(max_pitch_bend, int2fix15(PITCH_ATTACK));
    pitch_decay_inc = divfix(max_pitch_bend, int2fix15(PITCH_DECAY));

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sine_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Create a repeating timer that calls 
    // repeating_timer_callback (defaults core 0)
    struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    add_repeating_timer_us(-25, 
        repeating_timer_callback_core_0, NULL, &timer_core_0);

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
