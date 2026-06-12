#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/irq.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "blink.pio.h"

#include "pico/binary_info.h"

#define LORAWAN 1
#include "../lolra/lib/LoRa-SDR-Code.h"
#include "../lolra/lib/lorawan_simple.h"

#define OUTPUT_PIN 1

bi_decl(bi_program_description("25MHz Zero-Jitter RAM-Backed LoRaWAN Architecture"));
bi_decl(bi_1pin_with_name(OUTPUT_PIN, "RF Output"));

#include "chirp_tables.h"

// Dedicated RAM Buffers (50KB Total)
uint8_t up_chirp_ram[CHIRP_SIZE] __attribute__((aligned(4)));
uint8_t down_chirp_ram[CHIRP_SIZE] __attribute__((aligned(4)));

// DMA Resources
int chan0, chan1;
dma_channel_config c0, c1;

// DMA Block Queue
struct dma_block {
    const uint8_t *addr;
    uint32_t count_bytes;
};
struct dma_block packet_blocks[2048];
volatile int num_blocks = 0;
volatile int next_block_to_queue = 0;
volatile bool packet_done = false;

// Flash Configuration
#define FLASH_TARGET_OFFSET (1536 * 1024) 
#define FLASH_MAGIC 0xDEADC0DE
const uint32_t *flash_target_contents = (const uint32_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void queue_symbol(const uint8_t *chirp_buf, uint16_t symbol_shift, uint32_t total_len_bytes) {
    uint32_t byte_offset = (uint32_t)(symbol_shift * BYTE_OFFSET_PER_SYMBOL);
    uint32_t first_part_len = CHIRP_SIZE - byte_offset;
    
    if (total_len_bytes > CHIRP_SIZE) total_len_bytes = CHIRP_SIZE;

    if (total_len_bytes <= first_part_len) {
        packet_blocks[num_blocks].addr = chirp_buf + byte_offset;
        packet_blocks[num_blocks].count_bytes = total_len_bytes;
        num_blocks++;
    } else {
        uint32_t second_part_len = total_len_bytes - first_part_len;
        packet_blocks[num_blocks].addr = chirp_buf + byte_offset;
        packet_blocks[num_blocks].count_bytes = first_part_len;
        num_blocks++;
        
        packet_blocks[num_blocks].addr = chirp_buf;
        packet_blocks[num_blocks].count_bytes = second_part_len;
        num_blocks++;
    }
}

void dma_handler() {
    int finished_chan = -1;
    if (dma_channel_get_irq0_status(chan0)) {
        dma_channel_acknowledge_irq0(chan0);
        finished_chan = chan0;
    } else if (dma_channel_get_irq0_status(chan1)) {
        dma_channel_acknowledge_irq0(chan1);
        finished_chan = chan1;
    }

    if (finished_chan != -1) {
        if (next_block_to_queue < num_blocks) {
            dma_channel_set_read_addr(finished_chan, packet_blocks[next_block_to_queue].addr, false);
            dma_channel_set_trans_count(finished_chan, packet_blocks[next_block_to_queue].count_bytes, false);
            next_block_to_queue++;
        } else {
            int other_chan = (finished_chan == chan0) ? chan1 : chan0;
            dma_channel_config other_c = (finished_chan == chan0) ? c1 : c0;
            channel_config_set_chain_to(&other_c, other_chan);
            dma_channel_set_config(other_chan, &other_c, false);
            if (!dma_channel_is_busy(other_chan)) packet_done = true;
        }
    }
}

void __not_in_flash_func(core1_entry)() {
    multicore_lockout_victim_init();
    sleep_ms(3000);
    printf("Core 1: 25MHz RAM-Backed Engine Started (LoRaWAN)\n");

    memcpy(up_chirp_ram, up_chirp, CHIRP_SIZE);
    memcpy(down_chirp_ram, down_chirp, CHIRP_SIZE);
    
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    
    pio_sm_config sm_c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&sm_c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&sm_c, true, true, 8); 
    sm_config_set_clkdiv(&sm_c, PIO_DIVIDER); 
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    pio_sm_init(pio, sm, offset, &sm_c);

    chan0 = dma_claim_unused_channel(true);
    chan1 = dma_claim_unused_channel(true);
    
    c0 = dma_channel_get_default_config(chan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));

    c1 = dma_channel_get_default_config(chan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_8);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));

    dma_channel_set_irq0_enabled(chan0, true);
    dma_channel_set_irq0_enabled(chan1, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    uint16_t symbols[1024];
    int sym_count = 0;
    const char *payloads[] = {"Hello", "World", "Rebeca"};
    int payload_idx = 0;
    
    uint32_t frame_counter = 0;
    // Try to load FCnt from Flash
    if (flash_target_contents[0] == FLASH_MAGIC) {
        frame_counter = flash_target_contents[1];
    }
    
    // FORCED WIPE: Reset to 0 to align with TTN MAC state reset
    frame_counter = 0;
    
    static const uint8_t payload_key[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
    static const uint8_t network_skey[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
    static const uint8_t devaddress[4] = { 0x25, 0x21, 0x0B, 0x26 }; // Little Endian format

    while (1) {
        for (int b = 0; b < 3; b++) {
            const char *current_payload = payloads[payload_idx];
            uint8_t inner_payload_raw[64]; memset(inner_payload_raw, 0, sizeof(inner_payload_raw));
            memcpy(inner_payload_raw, current_payload, strlen(current_payload));
            
            uint8_t raw_payload_with_b0[256 + 16] = { 0 };
            uint8_t * raw_payload = raw_payload_with_b0 + 16;
            
            int raw_payload_size = GenerateLoRaWANPacket(raw_payload_with_b0, inner_payload_raw, strlen(current_payload), payload_key, network_skey, devaddress, frame_counter);

            CreateMessageFromPayload(symbols, &sym_count, 1024, LORA_SF, 4, raw_payload, raw_payload_size);
            
            printf("TX [%s] FCnt:%d...", current_payload, frame_counter); fflush(stdout);
            
            num_blocks = 0;
            for (int i = 0; i < 12; i++) queue_symbol(up_chirp_ram, 0, CHIRP_SIZE);
            
            uint8_t syncword = 0x21; // Private Sync Word for RF mapping compatibility
            queue_symbol(up_chirp_ram, (syncword & 0x0f) * 8, CHIRP_SIZE);
            queue_symbol(up_chirp_ram, ((syncword & 0xf0) >> 4) * 8, CHIRP_SIZE);
            
            queue_symbol(down_chirp_ram, 0, CHIRP_SIZE);
            queue_symbol(down_chirp_ram, 0, CHIRP_SIZE);
            queue_symbol(down_chirp_ram, 0, CHIRP_SIZE / 4);
            for (int i = 0; i < sym_count; i++) queue_symbol(up_chirp_ram, symbols[i], CHIRP_SIZE);

            packet_done = false;
            next_block_to_queue = 2;

            channel_config_set_chain_to(&c0, chan1);
            dma_channel_configure(chan0, &c0, &pio->txf[sm], packet_blocks[0].addr, packet_blocks[0].count_bytes, false);
            
            channel_config_set_chain_to(&c1, chan0);
            dma_channel_configure(chan1, &c1, &pio->txf[sm], packet_blocks[1].addr, packet_blocks[1].count_bytes, false);

            pio_sm_set_enabled(pio, sm, true);
            dma_channel_start(chan0);
            
            while (next_block_to_queue < num_blocks || dma_channel_is_busy(chan0) || dma_channel_is_busy(chan1)) {
                tight_loop_contents();
                watchdog_update();
                if (packet_done) break; 
            }
            
            pio_sm_set_enabled(pio, sm, false);
            gpio_put(OUTPUT_PIN, 0);
            printf(" OK\n");
            
            payload_idx = (payload_idx + 1) % 3;
            sleep_ms(1500);
            
            frame_counter++; 
        }
        
        printf(" (Requesting FC Save to Flash)\n");
        multicore_fifo_push_blocking(frame_counter);
        
        printf("Maintenance Window...\n");
        watchdog_update();
        sleep_ms(5000);
    }
}

int main() {
    sleep_ms(3000);
    set_sys_clock_khz(125000, true);
    stdio_init_all();
    watchdog_enable(8000, 1);
    printf("Pico W: No-Touch 125MHz Starting...\n");
    multicore_launch_core1(core1_entry);
    
    while (1) {
        if (multicore_fifo_rvalid()) {
            uint32_t fc = multicore_fifo_pop_blocking();
            uint32_t flash_data[FLASH_PAGE_SIZE/sizeof(uint32_t)];
            memset(flash_data, 0, FLASH_PAGE_SIZE);
            flash_data[0] = FLASH_MAGIC;
            flash_data[1] = fc;

            multicore_lockout_start_blocking();
            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
            flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)flash_data, FLASH_PAGE_SIZE);
            restore_interrupts(ints);
            multicore_lockout_end_blocking();

            printf("[Core 0] Frame Counter %lu saved to Flash.\n", fc);
        }
        tight_loop_contents();
        watchdog_update();
    }
}
