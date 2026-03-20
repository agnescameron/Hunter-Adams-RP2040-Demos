add_executable(fm_synth fm_synth.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(fm_synth pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_spi hardware_sync hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(fm_synth)