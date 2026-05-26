import math
import os
import argparse

# Ensure we are in the right directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

def main():
    parser = argparse.ArgumentParser(description="Generate LoRa Chirp Look-Up Tables for Pico W")
    parser.add_argument("--sf", type=int, default=10, help="Spreading Factor (7-12, default: 10)")
    parser.add_argument("--bw", type=float, default=125000.0, help="Bandwidth in Hz (default: 125000.0)")
    parser.add_argument("--freq", type=float, default=865099975.5859375, help="Center Frequency in Hz")
    args = parser.parse_args()

    f_sample = 62500000.0
    T_chirp = (2**args.sf) / args.bw
    CHIRP_SIZE = int(f_sample * T_chirp / 8)
    BYTE_OFFSET_PER_SYMBOL = (f_sample / args.bw) / 8.0

    f_low = args.freq - args.bw / 2.0
    f_high = args.freq + args.bw / 2.0

    print(f"Generating Chirp Tables:")
    print(f"  SF: {args.sf}, BW: {args.bw/1000} kHz, Freq: {args.freq/1e6} MHz")
    print(f"  T_chirp: {T_chirp*1000:.2f} ms")
    print(f"  CHIRP_SIZE: {CHIRP_SIZE} bytes")
    print(f"  Offset/Symbol: {BYTE_OFFSET_PER_SYMBOL:.2f} bytes")

    with open("chirp_tables.h", "w") as f:
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")
        
        # Macros for C code
        f.write(f"#define LORA_SF {args.sf}\n")
        f.write(f"#define LORA_BW {args.bw}\n")
        f.write(f"#define CHIRP_SIZE {CHIRP_SIZE}\n")
        f.write(f"#define BYTE_OFFSET_PER_SYMBOL {BYTE_OFFSET_PER_SYMBOL}\n\n")

        f.write(f"const uint8_t up_chirp[{CHIRP_SIZE}] = {{\n")
        for i in range(CHIRP_SIZE):
            byte_up = 0
            for b in range(8):
                t_idx = i * 8 + b
                t = t_idx / f_sample
                phase_up = f_low * t + (args.bw / (2.0 * T_chirp)) * t * t
                phase_up = phase_up - math.floor(phase_up)
                bit_up = 1 if phase_up >= 0.5 else 0
                byte_up = (byte_up >> 1) | (bit_up << 7)
            f.write(f"0x{byte_up:02X},")
            if i % 16 == 15:
                f.write("\n")
        f.write("};\n\n")

        f.write(f"const uint8_t down_chirp[{CHIRP_SIZE}] = {{\n")
        for i in range(CHIRP_SIZE):
            byte_down = 0
            for b in range(8):
                t_idx = i * 8 + b
                t = t_idx / f_sample
                phase_down = f_high * t - (args.bw / (2.0 * T_chirp)) * t * t
                phase_down = phase_down - math.floor(phase_down)
                bit_down = 1 if phase_down >= 0.5 else 0
                byte_down = (byte_down >> 1) | (bit_down << 7)
            f.write(f"0x{byte_down:02X},")
            if i % 16 == 15:
                f.write("\n")
        f.write("};\n")

    print("chirp_tables.h generated successfully.")

if __name__ == "__main__":
    main()
