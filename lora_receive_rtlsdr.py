#!/usr/bin/env python3
import lora
import osmosdr
from gnuradio import gr, blocks
import argparse
import sys
import signal
import time

class lora_rx_top_block(gr.top_block):
    def __init__(self, freq, samp_rate, sf, bw):
        gr.top_block.__init__(self, "LoRa Receiver")
        
        # Source
        self.src = osmosdr.source(args="rtl=0")
        self.src.set_sample_rate(samp_rate)
        self.src.set_center_freq(freq, 0)
        self.src.set_gain(40, 0)
        self.src.set_if_gain(20, 0)
        self.src.set_bb_gain(20, 0)
        
        # Receiver hierarchical block
        # sf, bandwidth, center_freq, channel_list
        # Note: the lora_receiver init might vary between versions, 
        # but let's use the one we saw in gr-lora/python/lora_receiver.py
        self.lora_rx = lora.lora_receiver(
            samp_rate=samp_rate, 
            center_freq=freq, 
            channel_list=[freq], 
            bandwidth=int(bw), 
            sf=sf, 
            implicit=False, 
            cr=1, 
            crc=True
        )
        
        # Debug sink to print messages to console
        self.debug = blocks.message_debug()
        
        # Socket sink (optional, for external tools like nc)
        self.snk = lora.message_socket_sink("127.0.0.1", 40868, 0)
        
        # Connections
        self.connect(self.src, self.lora_rx)
        #self.msg_connect((self.lora_rx, "frames"), (self.debug, "print"))
        self.msg_connect((self.lora_rx, "frames"), (self.snk, "in"))

def main():
    parser = argparse.ArgumentParser(description="LoRa Receiver with RTL-SDR")
    parser.add_argument("--freq", type=float, default=865.1e6, help="Frequency in Hz (default: 865.1e6)")
    parser.add_argument("--samp-rate", type=float, default=1e6, help="Sample rate in Hz (default: 1e6)")
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
    
    print(f"--- LoRa Receiver Configuration ---")
    print(f"Frequency: {args.freq/1e6:.2f} MHz")
    print(f"Spreading Factor: SF{args.sf}")
    print(f"Bandwidth: {args.bw/1e3:.1f} kHz")
    print(f"Sample Rate: {args.samp_rate/1e6:.1f} Msps")
    print(f"Hardware: RTL-SDR v4")
    print(f"-----------------------------------")
    
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
