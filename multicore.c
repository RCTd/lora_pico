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

bi_decl(bi_program_description("Hardware-Accelerated Lora Chirp Generator"));
bi_decl(bi_1pin_with_name(OUTPUT_PIN, "RF Output"));

const double f_center = 865100000.0;
const double bandwidth = 125000.0;
const double f_low    = f_center - bandwidth/2.0;
const double f_high   = f_center + bandwidth/2.0;
const double f_sample = 25000000.0; 

uint32_t base_up_phase_incs[ON_BUFFERS];
uint32_t base_down_phase_incs[ON_BUFFERS];

void init_tables() {
    printf("Initializing %d-buffer resolution tables at 25MSps...\n", ON_BUFFERS);
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

// --- LoRa Encoding Logic ---
// --- LoRa Encoding Logic (Standard-Compliant) ---

static uint8_t headerChecksum(const uint8_t *h) {
    int a0 = (h[0] >> 4) & 0x1; int a1 = (h[0] >> 5) & 0x1; int a2 = (h[0] >> 6) & 0x1; int a3 = (h[0] >> 7) & 0x1;
    int b0 = (h[0] >> 0) & 0x1; int b1 = (h[0] >> 1) & 0x1; int b2 = (h[0] >> 2) & 0x1; int b3 = (h[0] >> 3) & 0x1;
    int c0 = (h[1] >> 0) & 0x1; int c1 = (h[1] >> 1) & 0x1; int c2 = (h[1] >> 2) & 0x1; int c3 = (h[1] >> 3) & 0x1;
    return ((a0 ^ a1 ^ a2 ^ a3) << 4) | ((a3 ^ b1 ^ b2 ^ b3 ^ c0) << 3) | ((a2 ^ b0 ^ b3 ^ c1 ^ c3) << 2) | ((a1 ^ b0 ^ b2 ^ c0 ^ c1 ^ c2) << 1) | (a0 ^ b1 ^ c0 ^ c1 ^ c2 ^ c3);
}

static uint16_t crc16sx(uint16_t crc, const uint16_t poly) {
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) crc = (crc << 1) ^ poly;
        else crc <<= 1;
    }
    return crc;
}

static uint8_t xsum8(uint8_t t) {
    t ^= t >> 4; t ^= t >> 2; t ^= t >> 1;
    return (t & 1);
}

static uint16_t sx1272DataChecksum(const uint8_t *data, int length) {
    uint16_t res = 0; uint8_t v = 0xff;
    for (int i = 0; i < length; i++) {
        uint16_t crc = crc16sx(res, 0x1021);
        v = xsum8(v & 0xB8) | (v << 1);
        res = crc ^ data[i];
    }
    res ^= v; 
    v = xsum8(v & 0xB8) | (v << 1);
    res ^= (uint16_t)v << 8;
    return res;
}

static unsigned char encodeHamming84sx(const unsigned char x) {
    int d0 = (x >> 0) & 0x1; int d1 = (x >> 1) & 0x1; int d2 = (x >> 2) & 0x1; int d3 = (x >> 3) & 0x1;
    return (x & 0xf) | ((d0 ^ d1 ^ d2) << 4) | ((d1 ^ d2 ^ d3) << 5) | ((d0 ^ d1 ^ d3) << 6) | ((d0 ^ d2 ^ d3) << 7);
}

static unsigned char encodeParity54(const unsigned char b) {
    int x = b ^ (b >> 2); x = x ^ (x >> 1);
    return (b & 0xf) | ((x << 4) & 0x10);
}

static void diagonalInterleaveSx(const uint8_t *codewords, size_t numCodewords, uint16_t *symbols, size_t sf, size_t rdd) {
    size_t nb = 4 + rdd;
    for (size_t x = 0; x < numCodewords / sf; x++) {
        const size_t cwOff = x * sf;
        const size_t symOff = x * nb;
        for (size_t k = 0; k < nb; k++) {
            uint16_t s = 0;
            for (size_t m = 0; m < sf; m++) {
                const size_t i = (m + k + sf) % sf;
                const int bit = (codewords[cwOff + i] >> k) & 0x1;
                s |= (bit << m);
            }
            symbols[symOff + k] = s;
        }
    }
}

static unsigned short binaryToGray16(unsigned short num) {
    return num ^ (num >> 1);
}

static void Sx1272ComputeWhitening(uint8_t *buffer, uint16_t bufferSize, const int bitOfs, const int RDD) {
    static const int ofs0[8] = {6,4,2,0,-112,-114,-302,-34 };
    static const int ofs1[5] = {6,4,2,0,-360 };
    static const int whiten_len = 510;
    static const uint64_t whiten_seq[8] = {
        0x0102291EA751AAFFL,0xD24B050A8D643A17L,0x5B279B671120B8F4L,0x032B37B9F6FB55A2L,
        0x994E0F87E95E2D16L,0x7CBCFC7631984C26L,0x281C8E4F0DAEF7F9L,0x1741886EB7733B15L
    };
    const int *ofs = (1 == RDD) ? ofs1 : ofs0;
    for (int j = 0; j < bufferSize; j++) {
        uint8_t x = 0;
        for (int i = 0; i < 4 + RDD; i++) {
            int t = (ofs[i] + j + bitOfs + whiten_len) % whiten_len;
            if (whiten_seq[t >> 6] & ((uint64_t)1 << (t & 0x3F))) x |= 1 << i;
        }
        buffer[j] ^= x;
    }	
}

static unsigned short grayToBinary16(unsigned short num) {
    num = num ^ (num >> 8); num = num ^ (num >> 4); num = num ^ (num >> 2); num = num ^ (num >> 1);
    return num;
}

int encode_lora(uint16_t *symbols, const uint8_t *payload, int len, int sf, int rdd) {
    uint8_t data[255+2]; memcpy(data, payload, len);
    uint16_t crc = sx1272DataChecksum(data, len);
    data[len] = crc & 0xff; data[len+1] = (crc >> 8) & 0xff;
    
    int nHeaderCodewords = 8; // For SF10
    int header_shift = (sf > 6) ? 2 : 0;
    int header_sf = sf - header_shift;

    int total_payload_nibbles = (len + 2) * 2;
    int numCodewords = ((total_payload_nibbles + nHeaderCodewords + sf - 1) / sf) * sf;
    uint8_t codewords[512]; memset(codewords, 0, sizeof(codewords));
    
    uint8_t hdr[3];
    hdr[0] = len; hdr[1] = (1 /* CRC present */) | (rdd << 1); hdr[2] = headerChecksum(hdr);
    
    int cOfs = 0;
    codewords[cOfs++] = encodeHamming84sx(hdr[0] >> 4);
    codewords[cOfs++] = encodeHamming84sx(hdr[0] & 0xf);
    codewords[cOfs++] = encodeHamming84sx(hdr[1] & 0xf);
    codewords[cOfs++] = encodeHamming84sx(hdr[2] >> 4);
    codewords[cOfs++] = encodeHamming84sx(hdr[2] & 0xf);
    while (cOfs < nHeaderCodewords) codewords[cOfs++] = 0; // Pad header to 8
    
    int dOfs = 0;
    for (int i = 0; i < total_payload_nibbles; i++) {
        uint8_t nibble = (i % 2 == 0) ? (data[i/2] & 0xf) : (data[i/2] >> 4);
        if (rdd == 1) codewords[cOfs++] = encodeParity54(nibble);
        else if (rdd == 4) codewords[cOfs++] = encodeHamming84sx(nibble);
    }
    while (cOfs < numCodewords) codewords[cOfs++] = 0;

    // Whitening
    Sx1272ComputeWhitening(codewords, header_sf, 0, 4); // Header
    Sx1272ComputeWhitening(codewords + header_sf, numCodewords - header_sf, header_sf, rdd); // Payload

    uint16_t raw_symbols[512]; memset(raw_symbols, 0, sizeof(raw_symbols));
    diagonalInterleaveSx(codewords, header_sf, raw_symbols, header_sf, 4);
    diagonalInterleaveSx(codewords + nHeaderCodewords, numCodewords - nHeaderCodewords, raw_symbols + 8, sf, rdd);
    
    int sym_count = 8 + ((numCodewords - nHeaderCodewords) / sf) * (4 + rdd);
    for (int i = 0; i < sym_count; i++) {
        uint16_t s = grayToBinary16(raw_symbols[i]); 
        if (i < 8) s <<= header_shift;
        symbols[i] = binaryToGray16(s); 
    }
    return sym_count;
}

void play_partial_symbol(int chan0, int chan1, uint32_t *base_incs, uint16_t symbol_shift, int total_buffers, PIO pio, uint sm) {
    uint32_t scaled_shift = (uint32_t)symbol_shift * ON_BUFFERS / 1024;
    int wrap_point = scaled_shift % ON_BUFFERS;
    if (wrap_point % 2 != 0) wrap_point++; 
    int to_play = total_buffers;
    if (to_play % 2 != 0) to_play++; 
    run_sweep_optimized(chan0, chan1, base_incs, wrap_point, to_play);
}

void core1_entry() {
    printf("Core 1: LoRa SF10 Encoder Running\n");
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
    int sym_count = encode_lora(symbols, payload, 5, 10, 1);

    while (1) {
        printf("TX Packet (SF10, '%s')...", payload); fflush(stdout);
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

        // Targeted RF at 865.1MHz (25MSps) - Forward image behavior.
        // base_up (baseband up-chirp) results in an up-chirp at RF.
        for(int i=0; i<12; i++) play_symbol(chan0, chan1, base_up_phase_incs, 0, pio, sm);
        // Sync Word (Public: 0x34 -> symbols 3 and 4 shifted)
        play_symbol(chan0, chan1, base_up_phase_incs, 0x3 << (10 - 4), pio, sm);
        play_symbol(chan0, chan1, base_up_phase_incs, 0x4 << (10 - 4), pio, sm);
        // SFD (2.25 symbols) - must be down-chirps at RF, so use base_down.
        play_symbol(chan0, chan1, base_down_phase_incs, 0, pio, sm);
        play_symbol(chan0, chan1, base_down_phase_incs, 0, pio, sm);
        play_partial_symbol(chan0, chan1, base_down_phase_incs, 0, ON_BUFFERS / 4, pio, sm); 

        // Payload - up-chirps at RF, so use base_up.
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
    printf("Pico W: LoRa SF10 Encoder at 125MHz\n");
    multicore_launch_core1(core1_entry);
    while (1) tight_loop_contents();
}

