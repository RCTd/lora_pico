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
const double f_low   = 865030000.0; 
const double f_high  = 865170000.0; // 140 kHz sweep
const double f_sample = 10000000.0;  // 10 MHz Bitrate (100ns per bit)

// Timing Parameters
#define SAMPLES_PER_SEC 10000000ULL
#define ON_SAMPLES      (3ULL * SAMPLES_PER_SEC)

// Buffers per 3s burst: (3 * 10,000,000) / (1024 * 32) = ~915.5
#define ON_BUFFERS 916

// Phase increments pre-calculated for each buffer
uint32_t phase_incs_up[ON_BUFFERS];
uint32_t phase_incs_down[ON_BUFFERS];

// High-speed DDS state
uint32_t phase_acc = 0;
uint8_t bit_lookup[256];

void init_tables() {
    // 1. Bit Lookup Table
    for (int i = 0; i < 256; i++) {
        float val = sinf(2.0f * M_PI * i / 256.0f);
        bit_lookup[i] = (val > 0.1f) ? 1 : 0;
    }

    // 2. Pre-calculate frequency ramps
    for (int b = 0; b < ON_BUFFERS; b++) {
        double progress = (double)b / (double)ON_BUFFERS;
        
        // UP Ramp (low to high)
        double current_f_up = progress * (f_high - f_low) + f_low;
        double ratio_up = current_f_up / f_sample;
        phase_incs_up[b] = (uint32_t)((ratio_up - floor(ratio_up)) * 4294967296.0);
        
        // DOWN Ramp (high to low)
        double current_f_down = (1.0 - progress) * (f_high - f_low) + f_low;
        double ratio_down = current_f_down / f_sample;
        phase_incs_down[b] = (uint32_t)((ratio_down - floor(ratio_down)) * 4294967296.0);
    }
}

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

static inline void fill_buffer_ultrafast(uint32_t *buffer, uint32_t phase_inc) {
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t sample_word = 0;
        for (int b = 0; b < 32; b++) {
            sample_word = (sample_word << 1) | bit_lookup[phase_acc >> 24];
            phase_acc += phase_inc;
        }
        buffer[i] = sample_word;
    }
}

void core1_entry() {
    printf("Core 1: Robust Chirp Sequence Started\n");
    init_tables();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    
    int chan0 = dma_claim_unused_channel(true);
    int chan1 = dma_claim_unused_channel(true);

    while (1) {
        // 1. Reset PIO SM Hard
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_restart(pio, sm);
        pio_sm_clkdiv_restart(pio, sm);
        
        pio_sm_config sm_c = lora_out_program_get_default_config(offset);
        sm_config_set_out_pins(&sm_c, OUTPUT_PIN, 1);
        sm_config_set_fifo_join(&sm_c, PIO_FIFO_JOIN_TX);
        sm_config_set_out_shift(&sm_c, true, true, 32); 
        sm_config_set_clkdiv(&sm_c, 12.5f); 
        pio_sm_init(pio, sm, offset, &sm_c);
        pio_sm_clear_fifos(pio, sm);

        // 2. Fresh DMA configuration
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

        // 3. Reset state and fill initial buffers
        phase_acc = 0;
        fill_buffer_ultrafast(buffer_0, phase_incs_up[0]);
        fill_buffer_ultrafast(buffer_1, phase_incs_up[1]);

        // 4. Start Sequence
        printf("Starting Sequence...\n");
        pio_sm_set_enabled(pio, sm, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);
        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true); // Trigger chan0

        // --- 4a. 3x UP Chirps ---
        printf("3x UP: ");
        for(int n=0; n<3; n++) {
            for (int i = 0; i < ON_BUFFERS; i += 2) {
                if (i % 200 == 0) { printf("."); fflush(stdout); }
                dma_channel_wait_for_finish_blocking(chan0);
                fill_buffer_ultrafast(buffer_0, phase_incs_up[i]);
                dma_channel_set_read_addr(chan0, buffer_0, false);
                dma_channel_wait_for_finish_blocking(chan1);
                fill_buffer_ultrafast(buffer_1, phase_incs_up[i+1]);
                dma_channel_set_read_addr(chan1, buffer_1, false);
            }
        }
        printf(" Done\n");

        // --- 4b. 1.25x DOWN Chirps ---
        printf("1.25x DOWN: ");
        for (int i = 0; i < ON_BUFFERS; i += 2) {
            dma_channel_wait_for_finish_blocking(chan0);
            fill_buffer_ultrafast(buffer_0, phase_incs_down[i]);
            dma_channel_set_read_addr(chan0, buffer_0, false);
            dma_channel_wait_for_finish_blocking(chan1);
            fill_buffer_ultrafast(buffer_1, phase_incs_down[i+1]);
            dma_channel_set_read_addr(chan1, buffer_1, false);
        }
        for (int i = 0; i < (ON_BUFFERS / 4); i += 2) {
            dma_channel_wait_for_finish_blocking(chan0);
            fill_buffer_ultrafast(buffer_0, phase_incs_down[i]);
            dma_channel_set_read_addr(chan0, buffer_0, false);
            dma_channel_wait_for_finish_blocking(chan1);
            fill_buffer_ultrafast(buffer_1, phase_incs_down[i+1]);
            dma_channel_set_read_addr(chan1, buffer_1, false);
        }
        printf(" Done\n");

        // --- 4c. 1.0x Wrap-around UP from 60% ---
        printf("1.0x UP (60%% wrap): ");
        int start_idx = (ON_BUFFERS * 60 / 100);
        if (start_idx % 2 != 0) start_idx++; 
        for (int i = 0; i < ON_BUFFERS; i += 2) {
            int b = (start_idx + i) % ON_BUFFERS;
            int b_next = (start_idx + i + 1) % ON_BUFFERS;
            dma_channel_wait_for_finish_blocking(chan0);
            fill_buffer_ultrafast(buffer_0, phase_incs_up[b]);
            dma_channel_set_read_addr(chan0, buffer_0, false);
            dma_channel_wait_for_finish_blocking(chan1);
            fill_buffer_ultrafast(buffer_1, phase_incs_up[b_next]);
            dma_channel_set_read_addr(chan1, buffer_1, false);
        }
        printf(" Done\n");

        // 5. OFF PERIOD (1 second)
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        sleep_ms(10); // Hardware cooldown
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0); // Ensure pin is LOW
        printf("OFF (1s)\n");
        sleep_ms(1000);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: Continuous Bidirectional Chirp\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
