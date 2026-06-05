add_executable(pico-uart
    pico-uart.c
)

target_link_libraries(pico-uart
    pico_stdlib
)

# USB for laptop communication (printf, getchar, etc.)
pico_enable_stdio_usb(pico-uart 1)

# Disable UART stdio so UARTs are free for your own use
pico_enable_stdio_uart(pico-uart 0)

pico_add_extra_outputs(pico-uart)