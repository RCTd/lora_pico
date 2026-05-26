#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "blink.pio.h"

#include "pico/binary_info.h"

#include "../lolra/lib/LoRa-SDR-Code.h"

#define OUTPUT_PIN 1

bi_decl(bi_program_description("Hardware-Accelerated LoRa Chirp Generator"));
bi_decl(bi_1pin_with_name(OUTPUT_PIN, "RF Output"));

const double f_center = 865099975.5859375; 
const double bandwidth = 125000.0;
const double f_low    = f_center - bandwidth/2.0;
const double f_high   = f_center + bandwidth/2.0;
const double f_sample = 62500000.0; // 62.5 MHz
const double T_chirp  = 0.008192; 

#define CHIRP_SIZE 64000
#include "chirp_tables.h"

uint8_t up_chirp_ram[CHIRP_SIZE];
uint8_t down_chirp_ram[CHIRP_SIZE];

int chan0, chan1;
dma_channel_config c0, c1;

void play_symbol(const uint8_t *chirp_buf, uint16_t symbol_shift, PIO pio, uint sm) {
    uint32_t byte_offset = (uint32_t)(symbol_shift * 62.5); 
    uint32_t first_part_len = CHIRP_SIZE - byte_offset;
    uint32_t second_part_len = byte_offset;

    dma_channel_wait_for_finish_blocking(chan0);
    dma_channel_wait_for_finish_blocking(chan1);

    if (second_part_len == 0) {
        dma_channel_set_trans_count(chan1, 0, false); 
        dma_channel_set_read_addr(chan0, chirp_buf, false);
        dma_channel_set_trans_count(chan0, CHIRP_SIZE, true); 
    } else {
        dma_channel_set_read_addr(chan1, chirp_buf, false);
        dma_channel_set_trans_count(chan1, second_part_len, false);
        dma_channel_set_read_addr(chan0, chirp_buf + byte_offset, false);
        dma_channel_set_trans_count(chan0, first_part_len, true); 
    }
}

void play_partial_symbol(const uint8_t *chirp_buf, uint16_t symbol_shift, int num_bytes, PIO pio, uint sm) {
    uint32_t byte_offset = (uint32_t)(symbol_shift * 62.5);
    uint32_t bytes_left_in_buf = CHIRP_SIZE - byte_offset;
    dma_channel_wait_for_finish_blocking(chan0);
    dma_channel_wait_for_finish_blocking(chan1);

    if (num_bytes <= (int)bytes_left_in_buf) {
        dma_channel_set_trans_count(chan1, 0, false);
        dma_channel_set_read_addr(chan0, chirp_buf + byte_offset, false);
        dma_channel_set_trans_count(chan0, num_bytes, true);
    } else {
        uint32_t first_part = bytes_left_in_buf;
        uint32_t second_part = num_bytes - bytes_left_in_buf;
        dma_channel_set_read_addr(chan1, chirp_buf, false);
        dma_channel_set_trans_count(chan1, second_part, false);
        dma_channel_set_read_addr(chan0, chirp_buf + byte_offset, false);
        dma_channel_set_trans_count(chan0, first_part, true);
    }
}

void core1_entry() {
    printf("Core 1: 62.5MHz Engine Running\n");
    
    // Fast copy from Flash to RAM
    memcpy(up_chirp_ram, up_chirp, CHIRP_SIZE);
    memcpy(down_chirp_ram, down_chirp, CHIRP_SIZE);
    printf("Chirps copied to RAM.\n");

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &lora_out_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_gpio_init(pio, OUTPUT_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, OUTPUT_PIN, 1, true);
    
    chan0 = dma_claim_unused_channel(true);
    chan1 = dma_claim_unused_channel(true);
    c0 = dma_channel_get_default_config(chan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c0, chan1); 
    c1 = dma_channel_get_default_config(chan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_8);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c1, chan1);

    dma_channel_configure(chan0, &c0, &pio->txf[sm], NULL, 0, false);
    dma_channel_configure(chan1, &c1, &pio->txf[sm], NULL, 0, false);

    pio_sm_config sm_c = lora_out_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_c, OUTPUT_PIN, 1);
    sm_config_set_fifo_join(&sm_c, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&sm_c, true, true, 8);
    sm_config_set_clkdiv(&sm_c, 2.0f); 
    pio_sm_init(pio, sm, offset, &sm_c);

    int sf = 10;
    int rdd = 4;
    #define MAX_SYMBOLS 512
    uint16_t symbols[MAX_SYMBOLS];
    int sym_count = 0;
    
    const char *payloads[] = {"Hello", "World", "Rebeca"};
    const int num_payloads = 3;
    int payload_idx = 0;

    while (1) {
        const char *current_payload = payloads[payload_idx];
        int payload_len = strlen(current_payload);
        
        // Payload buffer needs space for CRC (2 bytes) and extra overhead.
        uint8_t payload_buf[64]; 
        memset(payload_buf, 0, sizeof(payload_buf));
        memcpy(payload_buf, current_payload, payload_len);

        int r = CreateMessageFromPayload(symbols, &sym_count, MAX_SYMBOLS, sf, rdd, payload_buf, payload_len);
        if (r < 0) {
            printf("Error encoding LoRa message for %s\n", current_payload);
            payload_idx = (payload_idx + 1) % num_payloads;
            continue;
        }

        printf("TX 62.5MHz Standards-Compliant LoRa Packet [%s]...", current_payload); fflush(stdout);
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0); dma_channel_abort(chan1);
        pio_sm_restart(pio, sm); pio_sm_clkdiv_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm); pio_sm_exec(pio, sm, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, sm, true);
        
        // 1. Preamble: 10 up-chirps
        for(int i=0; i<10; i++) play_symbol(up_chirp_ram, 0, pio, sm);
        
        // 2. Network Sync: 2 up-chirps (Standard value 0x12)
        // For SF10, 0x12 nibbles are 1 and 2, shifted by 3.
        play_symbol(up_chirp_ram, 8, pio, sm);
        play_symbol(up_chirp_ram, 16, pio, sm);

        // 3. Sync Frame: 2.25 down-chirps
        play_symbol(down_chirp_ram, 0, pio, sm);
        play_symbol(down_chirp_ram, 0, pio, sm);
        play_partial_symbol(down_chirp_ram, 0, CHIRP_SIZE / 4, pio, sm);
        
        // 4. Data Symbols (Header + Payload combined by CreateMessageFromPayload)
        for(int i=0; i<sym_count; i++) play_symbol(up_chirp_ram, symbols[i], pio, sm);
        
        dma_channel_wait_for_finish_blocking(chan0);
        dma_channel_wait_for_finish_blocking(chan1);
        printf(" Done\n");
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0); 
        payload_idx = (payload_idx + 1) % num_payloads;
    }
}

int main() {
    set_sys_clock_khz(125000, true);
    stdio_init_all();
    printf("Pico W: 62.5MHz LoRa Engine\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
