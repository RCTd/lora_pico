#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "blink.pio.h"

#define OUTPUT_PIN 1
#define BUFFER_SIZE 1024

uint32_t data_buffer[BUFFER_SIZE];

void setup_data(uint32_t *buffer, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        // Alternating 1s and 0s: 0xAAAAAAAA is 10101010...
        buffer[i] = 0xAAAAAAAA;
    }
}

void core1_entry() {
    printf("Core 1: Setting up PIO and DMA\n");

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);

    // Initialize PIO with no clock division (max speed)
    lora_out_program_init(pio, sm, offset, OUTPUT_PIN, 1.0f);

    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    // For continuous output, we can use two DMA channels to chain each other,
    // but for now, we'll just trigger it once or loop in code.
    
    while (1) {
        dma_channel_configure(
            dma_chan,
            &c,
            &pio->txf[sm], // Write to PIO TX FIFO
            data_buffer,   // Read from our buffer
            BUFFER_SIZE,   // Number of 32-bit words
            true           // Start immediately
        );

        dma_channel_wait_for_finish_blocking(dma_chan);
        // Small delay if we don't want to swamp the bus, though PIO will throttle via DREQ
    }
}

int main() {
    stdio_init_all();
    printf("Pico W LoRa DMA Output Example\n");

    setup_data(data_buffer, BUFFER_SIZE);

    multicore_launch_core1(core1_entry);

    while (1) {
        printf("Core 0 is alive and handling USB...\n");
        sleep_ms(1000);
    }
}
