import math
import os
import argparse

# Ensure we are in the right directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

def main():
    parser = argparse.ArgumentParser(description="Generate LoRa Chirp Look-Up Tables (25MHz Fallback)")
    parser.add_argument("--sf", type=int, default=10, help="Spreading Factor")
    parser.add_argument("--bw", type=float, default=125000.0, help="Bandwidth in Hz")
    parser.add_argument("--freq", type=float, default=865099975.5859375, help="Center Frequency in Hz")
    parser.add_argument("--offset", type=float, default=-170000.0, help="Frequency calibration offset in Hz")
    args = parser.parse_args()

    calibrated_freq = args.freq + args.offset

    # Fallback to 25MHz to fit in RAM while maintaining Integer Divider and Integer Offset
    f_sys = 125000000.0
    f_sample = 25000000.0
    pio_divider = f_sys / f_sample # Exactly 5.0 (Zero Jitter)
    
    num_chips = 2**args.sf
    T_symbol = num_chips / args.bw
    
    total_samples = int(f_sample * T_symbol)
    CHIRP_SIZE = total_samples // 8 # 25,600 bytes
    
    # (25,000,000 / 125,000) / 8 = 200 / 8 = 25.0 bytes exactly!
    BYTE_OFFSET_PER_SYMBOL = (f_sample / args.bw) / 8.0

    print(f"Generating 25MHz Zero-Jitter RAM Chirp Tables:")
    print(f"  SF: {args.sf}, BW: {args.bw/1000} kHz")
    print(f"  Target Freq: {args.freq/1e6} MHz, Calibrated: {calibrated_freq/1e6} MHz")
    print(f"  CHIRP_SIZE: {CHIRP_SIZE} bytes (Fits easily in RAM)")
    print(f"  Offset/Symbol: {BYTE_OFFSET_PER_SYMBOL} bytes (Perfect Integer)")

    with open("chirp_tables.h", "w") as f:
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")
        
        f.write(f"#define LORA_SF {args.sf}\n")
        f.write(f"#define LORA_BW {args.bw}\n")
        f.write(f"#define CHIRP_SIZE {CHIRP_SIZE}\n")
        f.write(f"#define PIO_DIVIDER {pio_divider}f\n")
        f.write(f"#define BYTE_OFFSET_PER_SYMBOL {int(BYTE_OFFSET_PER_SYMBOL)}\n\n")

        f.write(f"const uint8_t up_chirp[{CHIRP_SIZE}] = {{\n")
        for i in range(CHIRP_SIZE):
            byte_up = 0
            for b in range(8):
                t_idx = i * 8 + b
                tau = t_idx / total_samples
                phase_up = calibrated_freq * (t_idx / f_sample) + \
                        num_chips * (tau * tau / 2.0 - 0.5 * tau)
                phase_up = phase_up - math.floor(phase_up)
                bit_up = 1 if phase_up >= 0.5 else 0
                byte_up = (byte_up >> 1) | (bit_up << 7)
            f.write(f"0x{byte_up:02X},")
            if i % 16 == 15: f.write("\n")
        f.write("};\n\n")

        f.write(f"const uint8_t down_chirp[{CHIRP_SIZE}] = {{\n")
        for i in range(CHIRP_SIZE):
            byte_down = 0
            for b in range(8):
                t_idx = i * 8 + b
                tau = t_idx / total_samples
                phase_down = calibrated_freq * (t_idx / f_sample) + \
                        num_chips * (-tau * tau / 2.0 + 0.5 * tau)
                phase_down = phase_down - math.floor(phase_down)
                bit_down = 1 if phase_down >= 0.5 else 0
                byte_down = (byte_down >> 1) | (bit_down << 7)
            f.write(f"0x{byte_down:02X},")
            if i % 16 == 15: f.write("\n")
        f.write("};\n")

if __name__ == "__main__":
    main()
