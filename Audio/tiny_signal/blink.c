#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "blink.pio.h"

int main() {

    static const uint led_pin = 16;

    // Choose PIO instance (0 or 1)
    PIO pio = pio0;

    // Get first free state machine in PIO 0
    uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    uint offset = pio_add_program(pio, &blink_program);

    // set clock divider to run at system freq
    float div = 1.0;

    // Initialize the program using the helper function in our .pio file
    blink_program_init(pio, sm, offset, led_pin, div);

    pio_sm_set_enabled(pio, sm, true);

    while (true) {
        sleep_ms(500);
        pio_sm_put_blocking(pio, sm, 0);  // exactly one pulse
    }
}