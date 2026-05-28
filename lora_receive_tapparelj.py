#!/usr/bin/env python3
import lora_sdr
import osmosdr
from gnuradio import gr, blocks
import argparse
import sys
import signal
import time
import numpy as np

class lora_rx_top_block(gr.top_block):
    def __init__(self, freq, samp_rate, sf, bw):
        gr.top_block.__init__(self, "LoRa Receiver (Tapparelj)")
        
        # Source
        self.src = osmosdr.source(args="rtl=0")
        self.src.set_sample_rate(samp_rate)
        self.src.set_center_freq(freq, 0)
        self.src.set_gain(49, 0)
        
        # Buffer size optimization for RTL-SDR
        self.src.set_min_output_buffer(int(np.ceil(samp_rate/bw*(2**sf+2))))
        
        # Receiver hierarchical block from tapparelj/gr-lora_sdr
        # Note: sync_word is a list of integers
        self.lora_rx = lora_sdr.lora_sdr_lora_rx(
            center_freq=int(freq),
            bw=int(bw),
            cr=1,
            has_crc=True,
            impl_head=False,
            pay_len=255,
            samp_rate=int(samp_rate),
            sf=sf,
            sync_word=[16, 8], # Matches the updated Pico W's sync word
            soft_decoding=False,
            ldro_mode=2,
            print_rx=[True, True] # Prints headers and payload to console
        )
        
        # Message debug sink to capture frames from the receiver's 'out' port
        self.debug = blocks.message_debug()
        
        # Connections
        self.connect(self.src, self.lora_rx)
        self.msg_connect((self.lora_rx, "out"), (self.debug, "print"))

def main():
    parser = argparse.ArgumentParser(description="LoRa Receiver with RTL-SDR (tapparelj)")
    parser.add_argument("--freq", type=float, default=865.1e6, help="Frequency in Hz (default: 865.1e6)")
    parser.add_argument("--samp-rate", type=float, default=2e6, help="Sample rate in Hz (default: 2e6)")
    parser.add_argument("--sf", type=int, default=10, help="Spreading Factor 7-12 (default: 10)")
    parser.add_argument("--bw", type=float, default=125000, help="Bandwidth in Hz (default: 125000)")
    args = parser.parse_args()

    tb = lora_rx_top_block(args.freq, args.samp_rate, args.sf, args.bw)
    
    def signal_handler(sig, frame):
        print("\nStopping...")
        tb.stop()
        tb.wait()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"--- LoRa Receiver Configuration (tapparelj) ---")
    print(f"Frequency: {args.freq/1e6:.2f} MHz")
    print(f"Spreading Factor: SF{args.sf}")
    print(f"Bandwidth: {args.bw/1e3:.1f} kHz")
    print(f"Sample Rate: {args.samp_rate/1e6:.1f} Msps")
    print(f"Hardware: RTL-SDR v4")
    print(f"-----------------------------------------------")
    
    tb.start()
    print("Receiver started. Press Ctrl+C to stop.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    
    tb.stop()
    tb.wait()

if __name__ == "__main__":
    main()
