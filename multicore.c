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
const double f_sample = 25000000.0; // Lower bit rate to 25 MHz to give CPU time

// Timing Parameters
#define SAMPLES_PER_SEC 25000000
#define ON_SAMPLES  (3ULL * SAMPLES_PER_SEC)
#define OFF_MS      3000
#define ON_BUFFERS  ((3 * SAMPLES_PER_SEC) / (BUFFER_SIZE * 32))

// High-speed DDS state
uint32_t phase_acc = 0;
float sine_table[256];

void init_sine_table() {
    for (int i = 0; i < 256; i++) {
        sine_table[i] = sinf(2.0f * M_PI * i / 256.0f);
    }
}

uint64_t global_sample_count = 0;
uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

// Optimized real-time bit generation
void fill_signal_buffer_dds(uint32_t *buffer) {
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t sample_word = 0;
        
        // Linear frequency interpolation
        double progress = (double)global_sample_count / (double)ON_SAMPLES;
        if (progress > 1.0) progress = 1.0;
        double current_f = progress * (f_end - f_start) + f_start;
        
        // Calculate phase increment (fractional part of f/fs * 2^32)
        double ratio = current_f / f_sample;
        double fractional_ratio = ratio - floor(ratio);
        uint32_t phase_inc = (uint32_t)(fractional_ratio * (double)(1ULL << 32));

        for (int bit_idx = 0; bit_idx < 32; ++bit_idx) {
            // DDS Lookup
            float val = sine_table[phase_acc >> 24];
            int bit = (val > 0.1f) ? 1 : 0;

            sample_word = (sample_word << 1) | (uint32_t)bit;
            
            phase_acc += phase_inc;
            global_sample_count++;
        }
        buffer[i] = sample_word;
    }
}

void core1_entry() {
    printf("Core 1: Starting 25 MHz DDS Chirp (Optimized)\n");
    init_sine_table();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    pio_sm_config c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    // Shift Left (MSB first)
    sm_config_set_out_shift(&c, false, true, 32); 
    
    // Set PIO clock to 25 MHz (125 / 5 = 25)
    sm_config_set_clkdiv(&c, 5.0f);

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
        printf("ON\n");
        global_sample_count = 0;
        phase_acc = 0;
        pio_sm_set_enabled(pio, sm, true);
        
        fill_signal_buffer_dds(buffer_0);
        fill_signal_buffer_dds(buffer_1);

        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);

        for (uint32_t i = 0; i < ON_BUFFERS; i += 2) {
            dma_channel_wait_for_finish_blocking(chan0);
            fill_signal_buffer_dds(buffer_0);
            dma_channel_set_read_addr(chan0, buffer_0, false);
            
            dma_channel_wait_for_finish_blocking(chan1);
            fill_signal_buffer_dds(buffer_1);
            dma_channel_set_read_addr(chan1, buffer_1, false);
        }

        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0);

        printf("OFF\n");
        sleep_ms(OFF_MS);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: 25 MHz DDS Chirp implementation\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
