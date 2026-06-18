# LoRaWAN SDR Configuration Guide

This document details the exact code changes required to switch the Zero-BOM SDR ecosystem between different Spreading Factors (SF) and Bandwidths (BW).

Because the Raspberry Pi Pico SDR relies on precise mathematical timing and phase accumulation, changing the SF and BW requires adjusting the Pico's system clock, regenerating the chirp tables, and updating the ESP32 receiver logic.

---

## 1. Stable Baseline: SF10 / BW 125 kHz
This is the default "Gold Master" state. It provides the best balance of range and signal stability (zero bit-errors).

### Pico SDR Changes
1.  **Generate Chirp Tables:**
    ```bash
    python3 generate_chirp_tables.py --sf 10 --bw 125000
    ```
2.  **`generate_chirp_tables.py` parameters:**
    ```python
    f_sys = 125000000.0
    f_sample = 25000000.0
    pio_divider = f_sys / f_sample # Exactly 5.0
    ```
3.  **`multicore.c` System Clock:**
    ```c
    set_sys_clock_khz(125000, true);
    ```

### ESP32 Gateway Changes (`main.c`)
1.  **Radio Config:**
    ```c
    .bandwidth      = BW_125_KHZ,  
    .datarate       = SF_10,  
    ```
2.  **TTN JSON Masquerade:**
    ```c
    "\"datr\":\"SF10BW125\""
    ```

---

## 2. High Speed / Extended Range: SF11 / BW 500 kHz
To run SF11 without phase drift, the bandwidth must be increased to 500 kHz to shorten the chirp duration (`T_symbol`). To achieve a perfect integer divider at 500kHz, the Pico's system clock must be overclocked to 128 MHz.

### Pico SDR Changes
1.  **Generate Chirp Tables:**
    ```bash
    python3 generate_chirp_tables.py --sf 11 --bw 500000
    ```
2.  **`generate_chirp_tables.py` parameters:**
    ```python
    f_sys = 128000000.0
    f_sample = 32000000.0
    pio_divider = f_sys / f_sample # Exactly 4.0
    ```
3.  **`multicore.c` System Clock:**
    ```c
    set_sys_clock_khz(128000, true);
    ```

### ESP32 Gateway Changes (`main.c`)
1.  **Radio Config:**
    ```c
    .bandwidth      = BW_500_KHZ,  
    .datarate       = SF_11,  
    ```
2.  **TTN JSON Masquerade:**
    ```c
    "\"datr\":\"SF11BW500\""
    ```

---

## 3. Extreme Range Limit: SF12 / BW 500 kHz
This configuration pushes the Pico SDR memory to its absolute limits (32KB chirp tables) to achieve the maximum theoretical LoRa range, while maintaining the 500kHz bandwidth to prevent crystal oscillator phase drift.

### Pico SDR Changes
1.  **Generate Chirp Tables:**
    ```bash
    python3 generate_chirp_tables.py --sf 12 --bw 500000
    ```
2.  **`generate_chirp_tables.py` parameters (Same as SF11/500k):**
    ```python
    f_sys = 128000000.0
    f_sample = 32000000.0
    pio_divider = f_sys / f_sample # Exactly 4.0
    ```
3.  **`multicore.c` System Clock (Same as SF11/500k):**
    ```c
    set_sys_clock_khz(128000, true);
    ```

### ESP32 Gateway Changes (`main.c`)
1.  **Radio Config:**
    ```c
    .bandwidth      = BW_500_KHZ,  
    .datarate       = SF_12,  
    ```
2.  **TTN JSON Masquerade:**
    ```c
    "\"datr\":\"SF12BW500\""
    ```

---

## Important Note on Redundancy
For all configurations above, the physical transmission on the Pico is fixed to **CR 4/8** (Coding Rate) to ensure maximum signal resilience against jitter. The ESP32 is set to expect `CR_4_8`, but the JSON forwarded to TTN always masquerades as `"codr":"4/5"` to comply with the region's standard TTN backend requirements.