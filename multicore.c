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

// 863.77 / 125 = 86377 / 12500 repeats every 12500 bits. 
// LCM(12500, 32) = 100,000 bits = 3125 words.
#define PATTERN_WORDS 3125
uint32_t pattern_signal[PATTERN_WORDS];

// Timing Parameters
// 125,000,000 bits/sec / (1024 * 32) bits/buffer = ~3814.7 buffers/sec
#define BUFFERS_PER_SEC 3815
#define ON_BUFFERS (BUFFERS_PER_SEC * 3)

void precalculate_signal() {
    double f_target = 863770000.0; // 863.77 MHz
    double f_sample = 125000000.0;
    float threshold = 0.1f;

    for (int i = 0; i < PATTERN_WORDS; i++) pattern_signal[i] = 0;

    for (int n = 0; n < PATTERN_WORDS * 32; n++) {
        double val = sin(2.0 * M_PI * f_target * ((double)n / f_sample));
        if (val > threshold) {
            int word_idx = n / 32;
            int bit_in_word = 31 - (n % 32); 
            pattern_signal[word_idx] |= (1u << bit_in_word);
        }
    }
}

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

void core1_entry() {
    printf("Core 1: Starting hardware-stop optimized 863.77 MHz output\n");
    precalculate_signal();

    // In this pulsed mode, we'll just fill the buffers from the start of the pattern 
    // each time we restart the ON burst.
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer_0[i] = pattern_signal[i % PATTERN_WORDS];
        buffer_1[i] = pattern_signal[i % PATTERN_WORDS];
    }

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    pio_sm_config c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&c, false, true, 32); 
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(pio, sm, offset, &c);

    int chan0 = dma_claim_unused_channel(true);
    int chan1 = dma_claim_unused_channel(true);

    dma_channel_config c0 = dma_channel_get_default_config(chan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));

    dma_channel_config c1 = dma_channel_get_default_config(chan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));

    while (1) {
        // --- ON Phase (3 Seconds) ---
        printf("Switching to ON (DMA active)\n");
        pio_sm_set_enabled(pio, sm, true);

        // Re-enable chaining for continuous ON output
        channel_config_set_chain_to(&c0, chan1);
        channel_config_set_chain_to(&c1, chan0);
        
        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);

        for (uint32_t i = 0; i < ON_BUFFERS; i += 2) {
            dma_channel_wait_for_finish_blocking(chan0);
            dma_channel_set_read_addr(chan0, buffer_0, false);
            
            dma_channel_wait_for_finish_blocking(chan1);
            dma_channel_set_read_addr(chan1, buffer_1, false);
        }

        // --- OFF Phase (1 Second) ---
        // 1. Stop DMA from triggering more transfers
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        
        // 2. Stop PIO and force pin LOW
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
        gpio_put(OUTPUT_PIN, 0);

        printf("Switching to OFF (DMA and PIO IDLE)\n");
        sleep_ms(3000);
    }
}

int main() {
    stdio_init_all();
    printf("Pico W: Hardware-Stop Pulse Example\n");

    multicore_launch_core1(core1_entry);

    while (1) {
        tight_loop_contents();
    }
}
