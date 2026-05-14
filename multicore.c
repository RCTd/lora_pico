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

// LoRa Parameters (Centering exactly at 865.1 MHz, 125kHz BW)
const double f_center = 865100000.0;
const double bandwidth = 125000.0;
const double f_low    = f_center - bandwidth/2.0;
const double f_high   = f_center + bandwidth/2.0;
const double f_sample = 25000000.0;  // 25 MHz

// Timing Parameters
#define SAMPLES_PER_SEC 25000000ULL
#define ON_SAMPLES      (12500000ULL) // 500ms at 25MHz
#define ON_BUFFERS 381

// Buffers are pre-calculated for "Symbol 0" (base up-chirp)
uint32_t base_up_phase_incs[ON_BUFFERS];
uint32_t base_down_phase_incs[ON_BUFFERS];

uint8_t bit_lookup[256];

void init_tables() {
    printf("Initializing lookup tables (50%% Duty Cycle)...\n");
    for (int i = 0; i < 256; i++) {
        float val = sinf(2.0f * M_PI * i / 256.0f);
        // Use 0.0f for perfect 50% duty cycle to eliminate even harmonics
        bit_lookup[i] = (val > 0.0f) ? 1 : 0;
    }


    for (int b = 0; b < ON_BUFFERS; b++) {
        double progress = (double)b / (double)ON_BUFFERS;
        
        // Up-chirp
        double f_up = progress * bandwidth + f_low;
        double ratio_up = f_up / f_sample;
        base_up_phase_incs[b] = (uint32_t)((ratio_up - floor(ratio_up)) * 4294967296.0);

        // Down-chirp
        double f_down = (1.0 - progress) * bandwidth + f_low;
        double ratio_down = f_down / f_sample;
        base_down_phase_incs[b] = (uint32_t)((ratio_down - floor(ratio_down)) * 4294967296.0);
    }
    printf("Tables ready.\n");
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
        int b = (start_index + i) % ON_BUFFERS;
        int b_next = (start_index + i + 1) % ON_BUFFERS;
        int b_after = (start_index + i + 2) % ON_BUFFERS;

        // Calculate steps for frequency interpolation
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

void play_symbol(int chan0, int chan1, uint32_t *base_incs, uint16_t symbol_shift, int sf, PIO pio, uint sm) {
    int wrap_point = (int)((double)symbol_shift / (double)(1 << sf) * ON_BUFFERS);
    if (wrap_point % 2 != 0) wrap_point++; // Keep even for double-buffering

    int remaining = ON_BUFFERS - wrap_point;
    if (remaining > 0) {
        run_sweep_optimized(chan0, chan1, base_incs, wrap_point, remaining);
    }
    if (wrap_point > 0) {
        run_sweep_optimized(chan0, chan1, base_incs, 0, wrap_point);
    }
    printf("."); fflush(stdout);
}

void core1_entry() {
    printf("Core 1: LoRa-Mimic Started (865.03-865.17 MHz)\n");
    init_tables();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);

    interp_config cfg = interp_default_config();
    interp_config_set_add_raw(&cfg, true);
    interp_set_config(interp0, 0, &cfg);

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

    pio_sm_config sm_c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&sm_c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&sm_c, false, true, 32); 
    sm_config_set_clkdiv(&sm_c, 5.0f); 
    pio_sm_init(pio, sm, offset, &sm_c);

    while (1) {
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        pio_sm_restart(pio, sm);
        pio_sm_clkdiv_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset));

        interp0->accum[0] = 0;
        int32_t step_up = (int32_t)((base_up_phase_incs[1] - base_up_phase_incs[0]) / BUFFER_SIZE);
        fill_buffer_optimized(buffer_0, base_up_phase_incs[0], step_up);
        fill_buffer_optimized(buffer_1, base_up_phase_incs[1], step_up);

        printf("TX Packet: "); fflush(stdout);
        pio_sm_set_enabled(pio, sm, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);
        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true); 

        // 1. Preamble: 8 base up-chirps
        for(int i=0; i<8; i++) {
            play_symbol(chan0, chan1, base_up_phase_incs, 0, 7, pio, sm);
        }

        // 2. Sync Word
        play_symbol(chan0, chan1, base_up_phase_incs, 0x12, 7, pio, sm);
        play_symbol(chan0, chan1, base_up_phase_incs, 0x34, 7, pio, sm);

        // 3. SFD
        play_symbol(chan0, chan1, base_down_phase_incs, 0, 7, pio, sm);
        play_symbol(chan0, chan1, base_down_phase_incs, 0, 7, pio, sm);

        // 4. Data Payload
        uint16_t mock_data[] = {0x42, 0x13, 0x37, 0x69, 0xDE, 0xAD};
        for(int i=0; i<6; i++) {
            play_symbol(chan0, chan1, base_up_phase_incs, mock_data[i], 7, pio, sm);
        }
        printf(" Done\n");

        // 5. OFF
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0); 
        printf("OFF (2s)\n");
        sleep_ms(2000);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: LoRa Chirp Emulator\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
