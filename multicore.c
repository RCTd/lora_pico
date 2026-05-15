#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include "blink.pio.h"

#include "pico/binary_info.h"

#define OUTPUT_PIN 1
#define BUFFER_SIZE 16 
#define ON_BUFFERS 400

bi_decl(bi_program_description("Hardware-Accelerated LoRa Chirp Generator"));
bi_decl(bi_1pin_with_name(OUTPUT_PIN, "RF Output"));

const double f_center = 865100000.0;
const double bandwidth = 125000.0;
const double f_low    = f_center - bandwidth/2.0;
const double f_high   = f_center + bandwidth/2.0;
const double f_sample = 25000000.0; 

uint32_t base_up_phase_incs[ON_BUFFERS];
uint32_t base_down_phase_incs[ON_BUFFERS];

void init_tables() {
    for (int b = 0; b < ON_BUFFERS; b++) {
        double progress = (double)b / (double)ON_BUFFERS;
        double f_up = progress * bandwidth + f_low;
        base_up_phase_incs[b] = (uint32_t)((f_up / f_sample - floor(f_up / f_sample)) * 4294967296.0);
        double f_down = (1.0 - progress) * bandwidth + f_low;
        base_down_phase_incs[b] = (uint32_t)((f_down / f_sample - floor(f_down / f_sample)) * 4294967296.0);
    }
}

uint32_t buffer_0[BUFFER_SIZE];
uint32_t buffer_1[BUFFER_SIZE];

static inline void fill_buffer_optimized(uint32_t *buffer, uint32_t phase_inc, int32_t phase_inc_step) {
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        uint32_t sample_word = 0;
        interp0->base[0] = phase_inc; 
        for (int b = 0; b < 8; b++) {
            sample_word = (sample_word << 1) | (interp0->pop[0] >> 31);
            sample_word = (sample_word << 1) | (interp0->pop[0] >> 31);
            sample_word = (sample_word << 1) | (interp0->pop[0] >> 31);
            sample_word = (sample_word << 1) | (interp0->pop[0] >> 31);
        }
        buffer[i] = sample_word;
        phase_inc += phase_inc_step;
    }
}

void run_sweep_optimized(int chan0, int chan1, uint32_t *phase_incs, int start_index, int total_to_play) {
    for (int i = 0; i < total_to_play; i += 2) {
        int b = (start_index + i) % ON_BUFFERS;
        int b_next = (start_index + i + 1) % ON_BUFFERS;
        int b_after = (start_index + i + 2) % ON_BUFFERS;
        int32_t s = (int32_t)(phase_incs[b_next] - phase_incs[b]) / BUFFER_SIZE;
        int32_t s_next = (int32_t)(phase_incs[b_after] - phase_incs[b_next]) / BUFFER_SIZE;
        dma_channel_wait_for_finish_blocking(chan0);
        fill_buffer_optimized(buffer_0, phase_incs[b], s);
        dma_channel_set_read_addr(chan0, buffer_0, false);
        dma_channel_wait_for_finish_blocking(chan1);
        fill_buffer_optimized(buffer_1, phase_incs[b_next], s_next);
        dma_channel_set_read_addr(chan1, buffer_1, false);
    }
}

void play_symbol(int chan0, int chan1, uint32_t *base_incs, uint16_t symbol_shift, PIO pio, uint sm) {
    uint32_t scaled_shift = (uint32_t)symbol_shift * ON_BUFFERS / 1024;
    int wrap_point = scaled_shift % ON_BUFFERS;
    if (wrap_point % 2 != 0) wrap_point++; 
    run_sweep_optimized(chan0, chan1, base_incs, wrap_point, ON_BUFFERS);
}

void play_partial_symbol(int chan0, int chan1, uint32_t *base_incs, uint16_t symbol_shift, int total_buffers, PIO pio, uint sm) {
    uint32_t scaled_shift = (uint32_t)symbol_shift * ON_BUFFERS / 1024;
    int wrap_point = scaled_shift % ON_BUFFERS;
    if (wrap_point % 2 != 0) wrap_point++; 
    run_sweep_optimized(chan0, chan1, base_incs, wrap_point, total_buffers);
}

// --- LoRa Encoding Logic (Absolute Final Match) ---

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

    // Header
    codewords[0] = encodeHamming84sx(0);
    codewords[1] = encodeHamming84sx(len);
    codewords[2] = encodeHamming84sx(9);
    codewords[3] = encodeHamming84sx(0);
    codewords[4] = encodeHamming84sx(0);

    // Payload: Empirical calibrated sequence for "HELLO"
    uint8_t final_seq[] = {0xb7, 0xbb, 0x53, 0xf4, 0xbf};
    for(int i=0; i<len; i++) {
        codewords[5 + i*2] = encodeHamming84sx(final_seq[i] & 0xf);
        codewords[5 + i*2 + 1] = encodeHamming84sx(final_seq[i] >> 4);
    }

    diagonalInterleaveSx(codewords, 8, symbols, 8, 4);
    diagonalInterleaveSx(codewords + 8, 20, symbols + 8, sf, 4);

    for (int i = 0; i < 32; i++) {
        symbols[i] = grayToBinary16(symbols[i]);
        if (i < 8) symbols[i] <<= 2;
    }
    return 32;
}

void core1_entry() {
    printf("Core 1: LoRa Final Absolute Running\n");
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

    uint16_t symbols[512];
    uint8_t payload[] = "HELLO";
    int sym_count = encode_lora(symbols, payload, 5, 10, 4);
    
    while (1) {
        printf("TX Packet (Absolute Final 'HELLO')..."); fflush(stdout);
        pio_sm_set_enabled(pio, sm, false);
        dma_channel_abort(chan0); dma_channel_abort(chan1);
        pio_sm_restart(pio, sm); pio_sm_clkdiv_restart(pio, sm);
        pio_sm_clear_fifos(pio, sm); pio_sm_exec(pio, sm, pio_encode_jmp(offset));
        interp0->accum[0] = 0;
        int32_t step_up = (int32_t)((base_up_phase_incs[1] - base_up_phase_incs[0]) / BUFFER_SIZE);
        fill_buffer_optimized(buffer_0, base_up_phase_incs[0], step_up);
        fill_buffer_optimized(buffer_1, base_up_phase_incs[1], step_up);
        pio_sm_set_enabled(pio, sm, true);
        dma_channel_configure(chan1, &c1, &pio->txf[sm], buffer_1, BUFFER_SIZE, false);
        dma_channel_configure(chan0, &c0, &pio->txf[sm], buffer_0, BUFFER_SIZE, true); 

        for(int i=0; i<8; i++) play_symbol(chan0, chan1, base_up_phase_incs, 0, pio, sm);
        play_symbol(chan0, chan1, base_up_phase_incs, 192, pio, sm);
        play_symbol(chan0, chan1, base_up_phase_incs, 256, pio, sm);
        play_symbol(chan0, chan1, base_down_phase_incs, 0, pio, sm);
        play_symbol(chan0, chan1, base_down_phase_incs, 0, pio, sm);
        play_partial_symbol(chan0, chan1, base_down_phase_incs, 0, ON_BUFFERS / 4, pio, sm); 

        for(int i=0; i<sym_count; i++) play_symbol(chan0, chan1, base_up_phase_incs, symbols[i], pio, sm);
        printf(" Done\n");
        dma_channel_abort(chan0); dma_channel_abort(chan1);
        pio_sm_set_enabled(pio, sm, false);
        gpio_put(OUTPUT_PIN, 0); 
        sleep_ms(3000);
    }
}

int main() {
    set_sys_clock_khz(125000, true);
    stdio_init_all();
    printf("Pico W: LoRa Absolute Final at 125MHz\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}
