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

// 24 / 125 repeats every 125 bits. 
// LCM(125, 32) = 4000 bits = 125 words.
#define PATTERN_WORDS 125
uint32_t pattern_24mhz[PATTERN_WORDS];

// Timing Parameters
// 125,000,000 bits/sec / (1024 * 32) bits/buffer = ~3814.7 buffers/sec
#define BUFFERS_PER_SEC 3815
#define ON_BUFFERS (BUFFERS_PER_SEC * 2)
#define OFF_BUFFERS (BUFFERS_PER_SEC * 1)

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

void precalculate_24mhz() {
    double f_target = 24000000.0;
    double f_sample = 125000000.0;
    float threshold = 0.1f;

    for (int i = 0; i < PATTERN_WORDS; i++) pattern_24mhz[i] = 0;

    for (int n = 0; n < PATTERN_WORDS * 32; n++) {
        double val = sin(2.0 * M_PI * f_target * ((double)n / f_sample));
        if (val > threshold) {
            // MSB-first packing to match PIO shift direction
            int word_idx = n / 32;
            int bit_in_word = 31 - (n % 32); 
            pattern_24mhz[word_idx] |= (1u << bit_in_word);
        }
    }
}

void fast_fill_buffer(uint32_t *buffer, bool enabled) {
    if (!enabled) {
        memset(buffer, 0, BUFFER_SIZE * sizeof(uint32_t));
        return;
    }
    
    // Efficiently fill the buffer with the repeating 24MHz pattern
    for (int i = 0; i < BUFFER_SIZE; i += PATTERN_WORDS) {
        int to_copy = (i + PATTERN_WORDS <= BUFFER_SIZE) ? PATTERN_WORDS : (BUFFER_SIZE - i);
        memcpy(buffer + i, pattern_24mhz, to_copy * sizeof(uint32_t));
    }
}

void core1_entry() {
    printf("Core 1: Starting pulsed 24 MHz output (2s ON, 1s OFF)\n");
    precalculate_24mhz();

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    pio_sm_config c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    // MSB-first shifting
    sm_config_set_out_shift(&c, false, true, 32); 
    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

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

    bool is_on = true;
    uint32_t buffer_counter = 0;

    fast_fill_buffer(buffer_0, is_on);
    fast_fill_buffer(buffer_1, is_on);

    dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, false);
    dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);

    dma_channel_start(chan0);

    while (1) {
        dma_channel_wait_for_finish_blocking(chan0);
        buffer_counter++;
        if (is_on && buffer_counter >= ON_BUFFERS) {
            is_on = false;
            buffer_counter = 0;
            printf("Switching to OFF\n");
        } else if (!is_on && buffer_counter >= OFF_BUFFERS) {
            is_on = true;
            buffer_counter = 0;
            printf("Switching to ON\n");
        }
        fast_fill_buffer(buffer_0, is_on);
        dma_channel_set_read_addr(chan0, buffer_0, false);

        dma_channel_wait_for_finish_blocking(chan1);
        buffer_counter++;
        if (is_on && buffer_counter >= ON_BUFFERS) {
            is_on = false;
            buffer_counter = 0;
            printf("Switching to OFF\n");
        } else if (!is_on && buffer_counter >= OFF_BUFFERS) {
            is_on = true;
            buffer_counter = 0;
            printf("Switching to ON\n");
        }
        fast_fill_buffer(buffer_1, is_on);
        dma_channel_set_read_addr(chan1, buffer_1, false);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: Reverted Pulsed 24 MHz Output\n");

    multicore_launch_core1(core1_entry);

    while (1) {
        tight_loop_contents();
    }
}
