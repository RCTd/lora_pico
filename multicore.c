#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "blink.pio.h"

#include "pico/binary_info.h"

#define OUTPUT_PIN 1
#define BUFFER_SIZE 1024

bi_decl(bi_program_description("Hardware-Accelerated Lora Chirp Generator"));
bi_decl(bi_1pin_with_name(OUTPUT_PIN, "RF Output"));

// Signal Parameters
const double f_low   = 865030000.0; 
const double f_high  = 865170000.0; // 140 kHz sweep
const double f_sample = 25000000.0;  // 25 MHz Bitrate (40ns per bit)

// Timing Parameters
#define SAMPLES_PER_SEC 25000000ULL
#define ON_SAMPLES      (3ULL * SAMPLES_PER_SEC)

// Buffers per 3s burst: (3 * 25,000,000) / (1024 * 32) = ~2288.8
#define ON_BUFFERS 2288

// Phase increments pre-calculated for each buffer
uint32_t phase_incs_up[ON_BUFFERS];
uint32_t phase_incs_down[ON_BUFFERS];

// High-speed DDS state (Hardware Interpolator will handle phase_acc)
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

// Hardware-Accelerated: Uses RP2040 Interpolator for phase accumulation
static inline void fill_buffer_optimized(uint32_t *buffer, uint32_t phase_inc, int32_t phase_inc_step) {
    uint32_t current_phase_inc = phase_inc;

    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t sample_word = 0;
        
        // Update hardware frequency for this word
        interp0->base[0] = current_phase_inc;

        // Process 32 bits, hardware handles the add and shift
        for (int b = 0; b < 8; b++) {
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
        }
        buffer[i] = sample_word;

        // Smoothly interpolate frequency for the next word
        current_phase_inc += phase_inc_step;
    }
}

// Helper to run a sweep with optimized interpolation and progress logging
void run_sweep_optimized(int chan0, int chan1, uint32_t *phase_incs, int start_index, int total_to_play) {
    for (int i = 0; i < total_to_play; i += 2) {
        if (i % 400 == 0) { printf("."); fflush(stdout); }
        
        int b = (start_index + i) % ON_BUFFERS;
        int b_next = (start_index + i + 1) % ON_BUFFERS;
        int b_after = (start_index + i + 2) % ON_BUFFERS;

        // Calculate steps for frequency interpolation (Must use signed arithmetic!)
        int32_t diff = (int32_t)(phase_incs[b_next] - phase_incs[b]);
        int32_t s = diff / (int32_t)BUFFER_SIZE;

        int32_t diff_next = (int32_t)(phase_incs[b_after] - phase_incs[b_next]);
        int32_t s_next = diff_next / (int32_t)BUFFER_SIZE;

        dma_channel_wait_for_finish_blocking(chan0);
        fill_buffer_optimized(buffer_0, phase_incs[b], s);
        dma_channel_set_read_addr(chan0, buffer_0, false);

        dma_channel_wait_for_finish_blocking(chan1);
        fill_buffer_optimized(buffer_1, phase_incs[b_next], s_next);
        dma_channel_set_read_addr(chan1, buffer_1, false);
    }
}

void core1_entry() {
    printf("Core 1: Hardware-Accelerated Chirp Started (25 MHz)\n");
    init_tables();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);

    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);

    // --- Configure Interpolator 0 ---
    interp_config cfg = interp_default_config();
    interp_config_set_add_raw(&cfg, true); // Essential for phase accumulation
    interp_set_config(interp0, 0, &cfg);

    int chan0 = dma_claim_unused_channel(true);
    int chan1 = dma_claim_unused_channel(true);

    // Static DMA configuration
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

    // Static PIO configuration
    pio_sm_config sm_c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&sm_c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&sm_c, false, true, 32); 
    sm_config_set_clkdiv(&sm_c, 5.0f); // 125 / 5 = 25 MHz
    pio_sm_init(pio, sm, offset, &sm_c);

    while (1) {
        // 1. Hardware Reset (Clean Abort)
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        
        pio_sm_restart(pio, sm);
        pio_sm_clkdiv_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset));

        // 2. Reset state (Phase and Hardware Accumulator)
        interp0->accum[0] = 0;
        int32_t step_up = (int32_t)((phase_incs_up[1] - phase_incs_up[0]) / BUFFER_SIZE);
        fill_buffer_optimized(buffer_0, phase_incs_up[0], step_up);
        fill_buffer_optimized(buffer_1, phase_incs_up[1], step_up);

        // 3. Start Sequence
        printf("Starting 25 MHz Sequence...\n");
        pio_sm_set_enabled(pio, sm, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);
        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true); 

        // 4. Run Sequence
        printf("3x UP: ");
        for(int n=0; n<3; n++) {
            run_sweep_optimized(chan0, chan1, phase_incs_up, 0, ON_BUFFERS);
        }
        printf(" Done\n");

        printf("1.25x DOWN: ");
        run_sweep_optimized(chan0, chan1, phase_incs_down, 0, ON_BUFFERS);      
        run_sweep_optimized(chan0, chan1, phase_incs_down, 0, ON_BUFFERS / 4); 
        printf(" Done\n");

        printf("1.0x UP (60%% wrap): ");
        int start_idx = (ON_BUFFERS * 60 / 100);
        if (start_idx % 2 != 0) start_idx++; 
        run_sweep_optimized(chan0, chan1, phase_incs_up, start_idx, ON_BUFFERS);
        printf(" Done\n");

        // 5. OFF PERIOD (1 second)
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        sleep_ms(10); 
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0); 
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
