import math
import os

# Ensure we are in the right directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

CHIRP_SIZE = 64000
f_center = 865099975.5859375
bandwidth = 125000.0
f_low = f_center - bandwidth / 2.0
f_high = f_center + bandwidth / 2.0
f_sample = 62500000.0
T_chirp = 0.008192

print("Generating 62.5MHz single-chirps header file...")

with open("chirp_tables.h", "w") as f:
    f.write("#pragma once\n\n")
    f.write("#include <stdint.h>\n\n")
    
    f.write(f"const uint8_t up_chirp[{CHIRP_SIZE}] = {{\n")
    for i in range(CHIRP_SIZE):
        byte_up = 0
        for b in range(8):
            t_idx = i * 8 + b
            t = t_idx / f_sample
            phase_up = f_low * t + (bandwidth / (2.0 * T_chirp)) * t * t
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
            phase_down = f_high * t - (bandwidth / (2.0 * T_chirp)) * t * t
            phase_down = phase_down - math.floor(phase_down)
            bit_down = 1 if phase_down >= 0.5 else 0
            byte_down = (byte_down >> 1) | (bit_down << 7)
        f.write(f"0x{byte_down:02X},")
        if i % 16 == 15:
            f.write("\n")
    f.write("};\n")

print("chirp_tables.h generated successfully.")
