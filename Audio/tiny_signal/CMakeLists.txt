add_executable(blink blink.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(blink pico_stdlib hardware_pio)

# Create C header file with the name <pio program>.pio.h
pico_generate_pio_header(blink 
        ${CMAKE_CURRENT_LIST_DIR}/blink.pio
)

# create map/bin/hex file etc.
pico_add_extra_outputs(blink)