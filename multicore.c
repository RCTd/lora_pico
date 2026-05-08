#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "blink.pio.h"

#define OUTPUT_PIN 1
#define BUFFER_SIZE 1024

// Equation parameters
const double target_freq = 868000000.0; // 868 MHz
const double bit_rate = 125000000.0;    // 125 Mbps
const float threshold = 0.1f;

// Phase accumulator for DDS (Direct Digital Synthesis)
uint32_t phase_acc = 0;
uint32_t phase_inc;

// Sine lookup table for performance
float sine_table[256];

void init_sine_table() {
    for (int i = 0; i < 256; i++) {
        sine_table[i] = sinf(2.0f * M_PI * i / 256.0f);
    }
    // Calculate phase increment per bit
    // phase_inc = (f / bit_rate) * 2^32
    phase_inc = (uint32_t)((target_freq / bit_rate) * (double)(1ULL << 32));
}

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

void fill_buffer_from_equation(uint32_t *buffer) {
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t word = 0;
        for (int bit = 0; bit < 32; ++bit) {
            // Get value from lookup table using top 8 bits of accumulator
            float val = sine_table[phase_acc >> 24];
            if (val > threshold) {
                word |= (1u << bit);
            }
            phase_acc += phase_inc;
        }
        buffer[i] = word;
    }
}

void core1_entry() {
    printf("Core 1: Sine Equation Output Started\n");
    printf("Target Freq: %.1f Hz, Bit Rate: %.1f Hz\n", target_freq, bit_rate);
    
    if (target_freq >= bit_rate / 2.0) {
        printf("Warning: Frequency is above Nyquist (%.1f Hz). Aliasing will occur.\n", bit_rate / 2.0);
    }

    init_sine_table();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    lora_out_program_init(pio, sm, offset, OUTPUT_PIN, 1.0f);

    int chan0 = dma_claim_unused_channel(true);
    int chan1 = dma_claim_unused_channel(true);

    dma_channel_config c0 = dma_channel_get_default_config(chan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c0, chan1);

    dma_channel_config c1 = dma_channel_get_default_config(chan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c1, chan0);

    // Initial fill
    fill_buffer_from_equation(buffer_0);
    fill_buffer_from_equation(buffer_1);

    dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, false);
    dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);

    dma_channel_start(chan0);

    while (1) {
        dma_channel_wait_for_finish_blocking(chan0);
        fill_buffer_from_equation(buffer_0);
        dma_channel_set_read_addr(chan0, buffer_0, false);

        dma_channel_wait_for_finish_blocking(chan1);
        fill_buffer_from_equation(buffer_1);
        dma_channel_set_read_addr(chan1, buffer_1, false);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: Sine Equation (125 MHz Sampling)\n");

    multicore_launch_core1(core1_entry);

    while (1) {
        tight_loop_contents();
    }
}
