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

// LoRa Parameters (Mimicking SF7, 125kHz)
const double f_center = 865100000.0;
const double bandwidth = 125000.0;
const double f_low    = f_center - bandwidth/2.0;
const double f_high   = f_center + bandwidth/2.0;
const double f_sample = 25000000.0;  // 25 MHz

// Timing Parameters
#define SAMPLES_PER_SEC 25000000ULL
#define ON_SAMPLES      (3ULL * SAMPLES_PER_SEC)
#define ON_BUFFERS 2288

// Buffers are pre-calculated for "Symbol 0" (base up-chirp)
uint32_t base_up_phase_incs[ON_BUFFERS];
uint32_t base_down_phase_incs[ON_BUFFERS];

uint8_t bit_lookup[256];

void init_tables() {
    for (int i = 0; i < 256; i++) {
        float val = sinf(2.0f * M_PI * i / 256.0f);
        bit_lookup[i] = (val > 0.1f) ? 1 : 0;
    }

    for (int b = 0; b < ON_BUFFERS; b++) {
        double progress = (double)b / (double)ON_BUFFERS;
        
        // Up-chirp
        double f_up = progress * bandwidth + f_low;
        base_up_phase_incs[b] = (uint32_t)((f_up / f_sample) * 4294967296.0);

        // Down-chirp
        double f_down = (1.0 - progress) * bandwidth + f_low;
        base_down_phase_incs[b] = (uint32_t)((f_down / f_sample) * 4294967296.0);
    }
}

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

static inline void fill_buffer_with_shift(uint32_t *buffer, uint32_t *base_incs, uint16_t symbol_shift, int sf, int start_index) {
    // In LoRa, symbol_shift (0 to 2^SF-1) shifts the start of the chirp.
    // We simplify here: for 3s long chirps, symbol_shift wraps the 2288 buffers.
    int wrap_point = (int)((double)symbol_shift / (double)(1 << sf) * ON_BUFFERS);

    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t sample_word = 0;
        int idx = (start_index + i/32) % ON_BUFFERS; // Simplified: 1 freq per word
        
        // Apply cyclic shift
        int shifted_idx = (idx + wrap_point) % ON_BUFFERS;
        interp0->base[0] = base_incs[shifted_idx];

        for (int b = 0; b < 8; b++) {
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
            sample_word = (sample_word << 1) | bit_lookup[interp0->pop[0] >> 24];
        }
        buffer[i] = sample_word;
    }
}

void play_symbol(int chan0, int chan1, uint32_t *base_incs, uint16_t symbol_shift, int sf, PIO pio, uint sm) {
    for (int i = 0; i < ON_BUFFERS; i += 2) {
        dma_channel_wait_for_finish_blocking(chan0);
        fill_buffer_with_shift(buffer_0, base_incs, symbol_shift, sf, i);
        dma_channel_set_read_addr(chan0, buffer_0, false);

        dma_channel_wait_for_finish_blocking(chan1);
        fill_buffer_with_shift(buffer_1, base_incs, symbol_shift, sf, i+1);
        dma_channel_set_read_addr(chan1, buffer_1, false);
    }
}

void core1_entry() {
    printf("Core 1: LoRa-Mimic Started (865.1 MHz, 125kHz BW)\n");
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
        pio_sm_clear_fifos(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset));

        interp0->accum[0] = 0;
        fill_buffer_with_shift(buffer_0, base_up_phase_incs, 0, 7, 0);
        fill_buffer_with_shift(buffer_1, base_up_phase_incs, 0, 7, 1);

        printf("Starting LoRa Packet Mimic...\n");
        pio_sm_set_enabled(pio, sm, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);
        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true); 

        // 1. Preamble: 8 base up-chirps
        printf("Preamble (8 Up): ");
        for(int i=0; i<8; i++) {
            printf("%d", i); fflush(stdout);
            play_symbol(chan0, chan1, base_up_phase_incs, 0, 7, pio, sm);
        }
        printf("\n");

        // 2. Sync Word: 2 symbols (Example: 0x12)
        printf("Sync Word: ");
        play_symbol(chan0, chan1, base_up_phase_incs, 0x12, 7, pio, sm);
        play_symbol(chan0, chan1, base_up_phase_incs, 0x34, 7, pio, sm);
        printf("Done\n");

        // 3. SFD: 2.25 Down-chirps
        printf("SFD (Down): ");
        play_symbol(chan0, chan1, base_down_phase_incs, 0, 7, pio, sm);
        play_symbol(chan0, chan1, base_down_phase_incs, 0, 7, pio, sm);
        printf("Done\n");

        // 4. Data Payload (Mimic with cyclic shifts)
        printf("Payload: ");
        uint16_t mock_data[] = {0x42, 0x13, 0x37, 0x69, 0xDE, 0xAD};
        for(int i=0; i<6; i++) {
            printf("[%02X]", mock_data[i]); fflush(stdout);
            play_symbol(chan0, chan1, base_up_phase_incs, mock_data[i], 7, pio, sm);
        }
        printf("\n");

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
