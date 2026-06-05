#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart1
#define BAUD_RATE 115200

#define RX_PIN 7
#define TX_PIN 6

int main() {
    stdio_init_all();
    sleep_ms(2000);

    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);

    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    printf("Listening on UART1...\r\n");

    while (true) {
        while (uart_is_readable(UART_ID)) {
            uint8_t c = uart_getc(UART_ID);
            printf("%02X ", c);
        }
    }
}