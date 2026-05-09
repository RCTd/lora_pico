#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "blink.pio.h"

#define OUTPUT_PIN 1
#define BUFFER_SIZE 1024

// Signal Parameters
const double f_start  = 863770000.0; 
const double f_end    = 863780000.0; 
const double f_sample = 10000000.0;  // 10 MHz Bitrate (100ns per bit)

// Timing Parameters
#define SAMPLES_PER_SEC 10000000ULL
#define ON_SAMPLES      (3ULL * SAMPLES_PER_SEC)
#define OFF_MS          3000

// Buffers per 3s burst: (3 * 10,000,000) / (1024 * 32) = ~915.5
#define ON_BUFFERS 916

// Phase increments pre-calculated for each buffer to save CPU
uint32_t buffer_phase_incs[ON_BUFFERS];

// High-speed DDS state
uint32_t phase_acc = 0;
uint8_t bit_lookup[256];

void init_tables() {
    // 1. Bit Lookup Table
    for (int i = 0; i < 256; i++) {
        float val = sinf(2.0f * M_PI * i / 256.0f);
        bit_lookup[i] = (val > 0.1f) ? 1 : 0;
    }

    // 2. Pre-calculate frequency ramp for every buffer
    for (int b = 0; b < ON_BUFFERS; b++) {
        double progress = (double)b / (double)ON_BUFFERS;
        double current_f = progress * (f_end - f_start) + f_start;
        double ratio = current_f / f_sample;
        buffer_phase_incs[b] = (uint32_t)((ratio - floor(ratio)) * 4294967296.0);
    }
}

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

// ULTRALIGHT bit loop
static inline void fill_buffer_ultrafast(uint32_t *buffer, uint32_t phase_inc) {
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t sample_word = 0;
        // 32 bits per word
        for (int b = 0; b < 32; b++) {
            sample_word = (sample_word << 1) | bit_lookup[phase_acc >> 24];
            phase_acc += phase_inc;
        }
        buffer[i] = sample_word;
    }
}

void core1_entry() {
    printf("Core 1: 10 MHz Bitrate Chirp (Optimized Loop)\n");
    init_tables();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    pio_sm_config c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&c, false, true, 32); 
    sm_config_set_clkdiv(&c, 12.5f); // 125 / 12.5 = 10 MHz

    pio_sm_init(pio, sm, offset, &c);

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

    while (1) {
        printf("Chirp Start (863 to 873 MHz)\n");
        phase_acc = 0;
        pio_sm_set_enabled(pio, sm, true);
        
        // Initial 2 buffers
        fill_buffer_ultrafast(buffer_0, buffer_phase_incs[0]);
        fill_buffer_ultrafast(buffer_1, buffer_phase_incs[1]);

        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);

        // Run the 3-second burst
        for (int b = 2; b < ON_BUFFERS; b += 2) {
            dma_channel_wait_for_finish_blocking(chan0);
            fill_buffer_ultrafast(buffer_0, buffer_phase_incs[b]);
            dma_channel_set_read_addr(chan0, buffer_0, false);
            
            dma_channel_wait_for_finish_blocking(chan1);
            if (b + 1 < ON_BUFFERS) {
                fill_buffer_ultrafast(buffer_1, buffer_phase_incs[b+1]);
                dma_channel_set_read_addr(chan1, buffer_1, false);
            }
        }

        // Wait for the final buffers to finish outputting
        dma_channel_wait_for_finish_blocking(chan0);
        dma_channel_wait_for_finish_blocking(chan1);

        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0);

        printf("Silence (3s)\n");
        sleep_ms(OFF_MS);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: Zero-Jitter Chirp Implementation\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
