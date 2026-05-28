#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "blink.pio.h"

#include "pico/binary_info.h"

#include "../lolra/lib/LoRa-SDR-Code.h"

#define OUTPUT_PIN 1

bi_decl(bi_program_description("Gapless 64MHz LoRa SDR with No-Touch Safety Architecture"));
bi_decl(bi_1pin_with_name(OUTPUT_PIN, "RF Output"));

#include "chirp_tables.h"

int chan0, chan1;
dma_channel_config c0, c1;

void play_symbol_gapless(const uint8_t *chirp_buf, uint16_t symbol_shift, uint32_t total_len, PIO pio, uint sm) {
    uint32_t byte_offset = (uint32_t)(symbol_shift * BYTE_OFFSET_PER_SYMBOL); 
    uint32_t first_part_len = CHIRP_SIZE - byte_offset;
    uint32_t second_part_len = (total_len > first_part_len) ? (total_len - first_part_len) : 0;
    
    dma_channel_wait_for_finish_blocking(chan0);
    dma_channel_wait_for_finish_blocking(chan1);
    
    // Feed watchdog after every symbol (every 32ms at SF12)
    watchdog_update();

    if (second_part_len == 0) {
        dma_channel_set_read_addr(chan0, chirp_buf + byte_offset, false);
        dma_channel_set_trans_count(chan0, total_len / 4, true);
    } else {
        dma_channel_set_read_addr(chan1, chirp_buf, false);
        dma_channel_set_trans_count(chan1, second_part_len / 4, false);
        dma_channel_set_read_addr(chan0, chirp_buf + byte_offset, false);
        dma_channel_set_trans_count(chan0, first_part_len / 4, true);
    }
}

void core1_entry() {
    // 1. SAFETY: Boot delay to allow picotool to catch the device
    sleep_ms(3000);
    printf("Core 1: Safe-Burst 64MHz Engine Started\n");
    
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_sm_config sm_c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&sm_c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&sm_c, true, true, 32);
    sm_config_set_clkdiv(&sm_c, PIO_DIVIDER); 
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    pio_sm_init(pio, sm, offset, &sm_c);

    chan0 = dma_claim_unused_channel(true);
    chan1 = dma_claim_unused_channel(true);
    
    c0 = dma_channel_get_default_config(chan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c0, chan1); 

    c1 = dma_channel_get_default_config(chan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));

    dma_channel_configure(chan0, &c0, &pio->txf[sm], NULL, 0, false);
    dma_channel_configure(chan1, &c1, &pio->txf[sm], NULL, 0, false);

    uint16_t symbols[1024];
    int sym_count = 0;
    const char *payloads[] = {"Hello", "World", "Rebeca"};
    int payload_idx = 0;

    while (1) {
        // BURST: Send 3 packets
        for (int b = 0; b < 3; b++) {
            const char *current_payload = payloads[payload_idx];
            uint8_t payload_buf[64]; memset(payload_buf, 0, sizeof(payload_buf));
            memcpy(payload_buf, current_payload, strlen(current_payload));

            CreateMessageFromPayload(symbols, &sym_count, 1024, LORA_SF, 1, payload_buf, strlen(current_payload));
            
            printf("TX BURST %d/3 [%s]...", b+1, current_payload); fflush(stdout);
            pio_sm_set_enabled(pio, sm, true);
            
            // Physical Frame
            for(int i=0; i<12; i++) play_symbol_gapless(up_chirp, 0, CHIRP_SIZE, pio, sm);
            
            // Sync Word (8, 16)
            play_symbol_gapless(up_chirp, 8, CHIRP_SIZE, pio, sm);
            play_symbol_gapless(up_chirp, 16, CHIRP_SIZE, pio, sm);
            
            // Down-chirps (2.25)
            play_symbol_gapless(down_chirp, 0, CHIRP_SIZE, pio, sm);
            play_symbol_gapless(down_chirp, 0, CHIRP_SIZE, pio, sm);
            play_symbol_gapless(down_chirp, 0, CHIRP_SIZE / 4, pio, sm);
            
            // Data
            for(int i=0; i<sym_count; i++) play_symbol_gapless(up_chirp, symbols[i], CHIRP_SIZE, pio, sm);
            
            dma_channel_wait_for_finish_blocking(chan0);
            dma_channel_wait_for_finish_blocking(chan1);
            printf(" Done\n");
            
            pio_sm_set_enabled(pio, sm, false);
            gpio_put(OUTPUT_PIN, 0); 
            payload_idx = (payload_idx + 1) % 3;
            watchdog_update();
            sleep_ms(1000);
        }

        // 2. MAINTENANCE WINDOW: Idle for 5 seconds to free up USB/System Bus
        printf("Maintenance Window (USB Safe)... feeding watchdog.\n");
        watchdog_update();
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0);
        dma_channel_abort(chan1);
        gpio_put(OUTPUT_PIN, 0); 
        sleep_ms(5000);
    }
}

int main() {
    // BOOT-TIME RESCUE WINDOW (Always available)
    sleep_ms(3000);
    
    set_sys_clock_khz(125000, true);
    stdio_init_all();
    
    // Enable Hardware Watchdog (8 seconds timeout)
    watchdog_enable(8000, 1);
    
    printf("Pico W: No-Touch LoRa Engine Starting...\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
