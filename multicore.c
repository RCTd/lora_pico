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
    return (x & 0xf) | ((d0 ^ d1 ^ d2) << 4) | ((d1 ^ d2 ^ d3) << 5) | ((d0 ^ d1 ^ d3) << 6) | ((d0 ^ d2 ^ d3) << 7);
}

static unsigned short grayToBinary16(unsigned short num) {
    num = num ^ (num >> 8); num = num ^ (num >> 4); num = num ^ (num >> 2); num = num ^ (num >> 1);
    return num;
}

static void diagonalInterleaveSx(const uint8_t *codewords, const size_t numCodewords, uint16_t *symbols, const size_t PPM, const size_t RDD){
	for (size_t x = 0; x < numCodewords / PPM; x++)	{
		const size_t cwOff = x*PPM;
		const size_t symOff = x*(4 + RDD);
		for (size_t k = 0; k < 4 + RDD; k++){
			uint16_t s = symbols[symOff + k];
			for (size_t m = 0; m < PPM; m++){
				const size_t i = (m + k + PPM) % PPM;
				const int bit = (codewords[cwOff + i] >> k) & 0x1;
				s |= (bit << m);
			}
			symbols[symOff + k] = s;
		}
	}
}

int encode_lora(uint16_t *symbols, const uint8_t *payload, int len, int sf, int rdd) {
    uint8_t codewords[512]; memset(codewords, 0, sizeof(codewords));
    memset(symbols, 0, 512 * sizeof(uint16_t));
    uint8_t whitening[] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};
    codewords[0] = encodeHamming84sx(0);
    codewords[1] = encodeHamming84sx(len & 0xf);
    codewords[2] = encodeHamming84sx(len >> 4);
    codewords[3] = encodeHamming84sx(0x08); 
    codewords[4] = encodeHamming84sx(0);    

    for(int i=0; i<len; i++) {
        uint8_t val = payload[i] ^ whitening[i % 16]; 
        codewords[8 + i*2] = encodeHamming84sx(val & 0xf);
        codewords[8 + i*2 + 1] = encodeHamming84sx(val >> 4);
    }
    diagonalInterleaveSx(codewords, 8, symbols, 8, 4);
    int num_payload_codewords = len * 2;
    diagonalInterleaveSx(codewords + 8, num_payload_codewords, symbols + 8, sf, 4);
    for (int i = 0; i < 8 + (len*2); i++) {
        symbols[i] = grayToBinary16(symbols[i]);
        if (i < 8) symbols[i] <<= 2;
    }
    return 8 + (len * 2);
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
    int payload_len = strlen((char*)payload);
    int sym_count = encode_lora(symbols, payload, payload_len, 10, 4);
    
    while (1) {
        printf("TX 62.5MHz '%s'...", payload); fflush(stdout);
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0); dma_channel_abort(chan1);
        pio_sm_restart(pio, sm); pio_sm_clkdiv_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm); pio_sm_exec(pio, sm, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, sm, true);

        for(int i=0; i<8; i++) play_symbol(up_chirp, 0, pio, sm);
        play_symbol(up_chirp, 192, pio, sm);
        play_symbol(up_chirp, 256, pio, sm);
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
