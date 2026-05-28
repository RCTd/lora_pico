import math
from collections import defaultdict

# Fixed parameters for SF12 @ 62.5Msps
f_center = 865099975.5859375
bandwidth = 125000.0
f_sample = 62500000.0
sf = 12

f_low = f_center - bandwidth / 2.0
T_chirp = (2 ** sf) / bandwidth
total_samples = int(T_chirp * f_sample)

print(f"Generating SF{sf} up-chirp at {f_sample/1e6} Msps...")
print(f"Total samples to process: {total_samples}")

# Variables for run-length encoding
current_bit = -1
current_run_length = 0
run_lengths_0 = defaultdict(int)
run_lengths_1 = defaultdict(int)

# precalculate constants
k1 = f_low / f_sample
k2 = bandwidth / (2.0 * T_chirp * f_sample * f_sample)

for n in range(total_samples):
    # phase = f_low * t + (bandwidth / (2.0 * T_chirp)) * t * t
    # where t = n / f_sample
    phase = k1 * n + k2 * n * n
    phase -= math.floor(phase)
    bit = 1 if phase >= 0.5 else 0
    
    if bit == current_bit:
        current_run_length += 1
    else:
        if current_bit == 0:
            run_lengths_0[current_run_length] += 1
        elif current_bit == 1:
            run_lengths_1[current_run_length] += 1
            
        current_bit = bit
        current_run_length = 1

# Catch the final run
if current_bit == 0:
    run_lengths_0[current_run_length] += 1
elif current_bit == 1:
    run_lengths_1[current_run_length] += 1

print("\n--- Run-Length Analysis ---")
print("Continuous ZEROS (0s):")
for length in sorted(run_lengths_0.keys()):
    print(f"  Length {length}: {run_lengths_0[length]} occurrences")

print("\nContinuous ONES (1s):")
for length in sorted(run_lengths_1.keys()):
    print(f"  Length {length}: {run_lengths_1[length]} occurrences")

total_runs = sum(run_lengths_0.values()) + sum(run_lengths_1.values())
print(f"\nTotal Number of Flips (Edges): {total_runs}")
