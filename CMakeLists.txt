add_executable(lora_dma
        multicore.c
        )

  pico_generate_pio_header(lora_dma ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
        
# Add pico_multicore which is required for multicore functionality
target_link_libraries(lora_dma
        pico_stdlib
        hardware_pio
        pico_multicore)

pico_enable_stdio_usb(lora_dma 1)
pico_enable_stdio_uart(lora_dma 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(lora_dma)

# add url via pico_set_program_url
example_auto_set_url(lora_dma)
