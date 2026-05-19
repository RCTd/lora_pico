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
uint8_t up_chirp[CHIRP_SIZE];
uint8_t down_chirp[CHIRP_SIZE];

void init_tables() {
    printf("Generating 62.5MHz single-chirps...\n");
    for (int i = 0; i < CHIRP_SIZE; i++) {
        uint8_t byte_up = 0;
        uint8_t byte_down = 0;
        for (int b = 0; b < 8; b++) {
            int t_idx = i * 8 + b;
            double t = (double)t_idx / f_sample;
            double phase_up = f_low * t + (bandwidth / (2.0 * T_chirp)) * t * t;
            double phase_down = f_high * t - (bandwidth / (2.0 * T_chirp)) * t * t;
            phase_up = phase_up - floor(phase_up);
            phase_down = phase_down - floor(phase_down);
            uint8_t bit_up = (phase_up >= 0.5) ? 1 : 0;
            uint8_t bit_down = (phase_down >= 0.5) ? 1 : 0;
            byte_up = (byte_up >> 1) | (bit_up << 7);
            byte_down = (byte_down >> 1) | (bit_down << 7);
        }
        up_chirp[i] = byte_up;
        down_chirp[i] = byte_down;
    }
    printf("62.5MHz Tables ready.\n");
}

int chan0, chan1;
dma_channel_config c0, c1;

void play_symbol(uint8_t *chirp_buf, uint16_t symbol_shift, PIO pio, uint sm) {
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

void play_partial_symbol(uint8_t *chirp_buf, uint16_t symbol_shift, int num_bytes, PIO pio, uint sm) {
    uint32_t byte_offset = (uint32_t)(symbol_shift * 62.5);
    uint32_t bytes_left_in_buf = CHIRP_SIZE - byte_offset;
    dma_channel_wait_for_finish_blocking(chan0);
    dma_channel_wait_for_finish_blocking(chan1);

    if (num_bytes <= bytes_left_in_buf) {
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

static unsigned char encodeHamming84sx(const unsigned char x) {
    int d0 = (x >> 0) & 0x1; int d1 = (x >> 1) & 0x1; int d2 = (x >> 2) & 0x1; int d3 = (x >> 3) & 0x1;
    int p4 = d0 ^ d1 ^ d2;
    int p5 = d1 ^ d2 ^ d3;
    int p6 = d0 ^ d1 ^ d3;
    int p7 = d0 ^ d2 ^ d3;
    return (x & 0xf) | (p4 << 4) | (p5 << 5) | (p6 << 6) | (p7 << 7);
}

static unsigned short binaryToGray(unsigned short num) {
    return num ^ (num >> 1);
}

static const uint8_t whitening_seq[] = {
    0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE1, 0xC2, 0x85, 0x0B, 0x17, 0x2F, 0x5E, 0xBC, 0x78, 0xF1, 0xE3,
    0xC6, 0x8D, 0x1A, 0x34, 0x68, 0xD0, 0xA0, 0x40, 0x80, 0x01, 0x02, 0x04, 0x08, 0x11, 0x23, 0x47,
    0x8E, 0x1C, 0x38, 0x71, 0xE2, 0xC4, 0x89, 0x12, 0x25, 0x4B, 0x97, 0x2E, 0x5C, 0xB8, 0x70, 0xE0,
    0xC0, 0x81, 0x03, 0x06, 0x0C, 0x19, 0x32, 0x64, 0xC9, 0x92, 0x24, 0x49, 0x93, 0x26, 0x4D, 0x9B,
    0x37, 0x6E, 0xDC, 0xB9, 0x72, 0xE4, 0xC8, 0x90, 0x20, 0x41, 0x82, 0x05, 0x0A, 0x15, 0x2B, 0x56,
    0xAD, 0x5B, 0xB6, 0x6D, 0xDA, 0xB5, 0x6B, 0xD6, 0xAC, 0x59, 0xB2, 0x65, 0xCB, 0x96, 0x2D, 0x5A,
    0xB4, 0x69, 0xD2, 0xA4, 0x48, 0x91, 0x22, 0x45, 0x8A, 0x14, 0x29, 0x52, 0xA5, 0x4A, 0x95, 0x2A,
    0x54, 0xA9, 0x53, 0xA7, 0x4E, 0x9D, 0x3B, 0x77, 0xEE, 0xDD, 0xBB, 0x76, 0xED, 0xDB, 0xB7, 0x6F,
    0xDE, 0xBD, 0x7A, 0xF5, 0xEB, 0xD7, 0xAF, 0x5F, 0xBE, 0x7C, 0xF9, 0xF2, 0xE5, 0xCA, 0x94, 0x28,
    0x50, 0xA1, 0x42, 0x84, 0x09, 0x13, 0x27, 0x4F, 0x9F, 0x3F, 0x7F
};

int encode_lora(uint16_t *symbols, const uint8_t *payload, int len, int sf, int rdd) {
    uint8_t h[8]; memset(h, 0, 8);
    h[0] = (len >> 4);
    h[1] = (len & 0x0F);
    h[2] = (4 << 1) | 0; // CR=4/8, No CRC
    
    bool c4 = (h[0] & 0x8) >> 3 ^ (h[0] & 0x4) >> 2 ^ (h[0] & 0x2) >> 1 ^ (h[0] & 0x1);
    bool c3 = (h[0] & 0x8) >> 3 ^ (h[1] & 0x8) >> 3 ^ (h[1] & 0x4) >> 2 ^ (h[1] & 0x2) >> 1 ^ (h[2] & 0x1);
    bool c2 = (h[0] & 0x4) >> 2 ^ (h[1] & 0x8) >> 3 ^ (h[1] & 0x1) ^ (h[2] & 0x8) >> 3 ^ (h[2] & 0x2) >> 1;
    bool c1 = (h[0] & 0x2) >> 1 ^ (h[1] & 0x4) >> 2 ^ (h[1] & 0x1) ^ (h[2] & 0x4) >> 2 ^ (h[2] & 0x2) >> 1 ^ (h[2] & 0x1);
    bool c0 = (h[0] & 0x1) ^ (h[1] & 0x2) >> 1 ^ (h[2] & 0x8) >> 3 ^ (h[2] & 0x4) >> 2 ^ (h[2] & 0x2) >> 1 ^ (h[2] & 0x1);
    h[3] = c4; h[4] = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0;

    uint8_t codewords[512]; memset(codewords, 0, sizeof(codewords));
    for (int i=0; i<8; i++) codewords[i] = encodeHamming84sx(h[i]);
    
    for(int i=0; i<len; i++) {
        uint8_t val = payload[i] ^ whitening_seq[i]; 
        codewords[8 + i*2] = encodeHamming84sx(val & 0xf);
        codewords[8 + i*2 + 1] = encodeHamming84sx(val >> 4);
    }
    
    // Header block (8 symbols)
    for (int i=0; i<8; i++) {
        uint16_t s = 0;
        for (int j=0; j<8; j++) {
            int cw_idx = (i - j - 1); while (cw_idx < 0) cw_idx += 8; cw_idx %= 8;
            int bit = (codewords[cw_idx] >> i) & 0x1;
            s |= (bit << j);
        }
        int parity = 0; for(int b=0; b<8; b++) if (s & (1<<b)) parity++;
        s |= ((parity % 2) << 8);
        symbols[i] = binaryToGray(s);
    }

    // Payload block (8 symbols)
    for (int i=0; i<8; i++) {
        uint16_t s = 0;
        for (int j=0; j<10; j++) {
            int cw_idx = (i - j - 1); while (cw_idx < 0) cw_idx += 10; cw_idx %= 10;
            int bit = (codewords[8 + cw_idx] >> i) & 0x1;
            s |= (bit << j);
        }
        symbols[8 + i] = binaryToGray(s);
    }
    return 16;
}

void core1_entry() {
    printf("Core 1: 62.5MHz Engine Running\n");
    init_tables();
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

    uint16_t symbols[512];
    uint8_t payload[] = "HELLO";
    int sym_count = encode_lora(symbols, payload, 5, 10, 4);
    
    while (1) {
        printf("TX 62.5MHz 'HELLO'..."); fflush(stdout);
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0); dma_channel_abort(chan1);
        pio_sm_restart(pio, sm); pio_sm_clkdiv_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm); pio_sm_exec(pio, sm, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, sm, true);

        for(int i=0; i<8; i++) play_symbol(up_chirp, 0, pio, sm);
        play_symbol(up_chirp, 8, pio, sm);
        play_symbol(up_chirp, 16, pio, sm);
        play_symbol(down_chirp, 0, pio, sm);
        play_symbol(down_chirp, 0, pio, sm);
        play_partial_symbol(down_chirp, 0, 16000, pio, sm); 
        for(int i=0; i<sym_count; i++) play_symbol(up_chirp, symbols[i], pio, sm);
        
        dma_channel_wait_for_finish_blocking(chan0);
        dma_channel_wait_for_finish_blocking(chan1);
        sleep_us(100);
        printf(" Done\n");
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0); 
        sleep_ms(3000);
    }
}

int main() {
    set_sys_clock_khz(125000, true);
    stdio_init_all();
    printf("Pico W: 62.5MHz LoRa Engine\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
