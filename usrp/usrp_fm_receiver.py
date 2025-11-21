#!/usr/bin/env python3  # Corrected for execution on host
# -*- coding: utf-8 -*-

import time
import signal
import sys
import pmt
from gnuradio import gr, blocks, analog, filter, uhd, zeromq
from gnuradio.filter import firdes, window

class message_to_gain(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(self,
            name="message_to_gain",
            in_sig=None,
            out_sig=[float])
        self.message_port_register_in(pmt.intern('msg_in'))
        self.set_msg_handler(pmt.intern('msg_in'), self.handle_msg)

    def handle_msg(self, msg):
        try:
            gain = float(pmt.symbol_to_string(msg))
            self.output_gain(gain)
        except:
            pass

    def output_gain(self, gain):
        # Push the gain value to the output stream
        self.add_item_tag(0, self.nitems_written(0), pmt.intern("gain"), pmt.from_double(gain))


class message_to_freq(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(self,
            name="message_to_freq",
            in_sig=None,
            out_sig=[float])
        self.message_port_register_in(pmt.intern('msg_in'))
        self.set_msg_handler(pmt.intern('msg_in'), self.handle_msg)

    def handle_msg(self, msg):
        try:
            freq = float(pmt.symbol_to_string(msg))
            self.output(freq)
        except:
            pass

    def output(self, freq):
        self.add_item_tag(0, self.nitems_written(0), pmt.intern("freq"), pmt.from_double(freq))


class RX_FM_USRP_UDP(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self, "Rx FM USRP UDP Headless")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = 2e6           # Sample rate for USRP
        self.gain = 15                 # Initial gain (dB)
        self.freq = 102.5e6            # Center frequency (Hz)
        self.freq_cos = 300e3          # Offset for cosine mixing (Hz)
        self.bw = 200e3                # RF bandwidth (Hz)

        ##################################################
        # USRP Source
        ##################################################
        self.uhd_source = uhd.usrp_source(
            ",".join(('', '')),      # Empty args: will use default device
            uhd.stream_args(cpu_format="fc32", channels=[0]),
        )
        self.uhd_source.set_samp_rate(self.samp_rate)
        self.uhd_source.set_center_freq(self.freq, 0)
        self.uhd_source.set_gain(self.gain, 0)
        self.uhd_source.set_antenna("RX2", 0)
        self.uhd_source.set_bandwidth(self.bw, 0)

        ##################################################
        # Signal Processing Chain
        ##################################################
        # Generate a cosine wave for mixing
        self.sig_source = analog.sig_source_c(
            self.samp_rate, analog.GR_COS_WAVE, self.freq_cos, 1, 0
        )
        # Multiply RF signal with cosine to shift frequency
        self.mult = blocks.multiply_vcc(1)

        # Low-pass filter to isolate FM bandwidth
        self.lowpass = filter.fir_filter_ccf(
            decimation=10,
            taps=firdes.low_pass(1, self.samp_rate, 90e3, 5e3, window.WIN_HAMMING)
        )

        # Rational resampler to adjust sample rate for UDP sink
        self.resampler = filter.rational_resampler_ccc(
            interpolation=12, decimation=15
        )

        # Send complex baseband samples over UDP to host
        self.udp_sink = blocks.udp_sink(
            gr.sizeof_gr_complex, '172.16.2.116', 9997, 1472, True
        )

        ##################################################
        # ZMQ Control: Frequency Updates
        ##################################################
        self.zmq_pull_freq = zeromq.pull_msg_source(
            'tcp://172.16.2.116:9996', 100
        )
        self.msg_freq_handler = message_to_freq(self.set_freq)
        self.msg_connect(
            (self.zmq_pull_freq, 'out'),
            (self.msg_freq_handler, 'in')
        )

        ##################################################
        # ZMQ Control: Gain Updates
        ##################################################
        self.zmq_pull_gain = zeromq.pull_msg_source(
            'tcp://172.16.2.116:9995', 100
        )
        self.msg_gain_handler = message_to_gain(self.set_gain)
        self.msg_connect(
            (self.zmq_pull_gain, 'out'),
            (self.msg_gain_handler, 'in')
        )

        ##################################################
        # Block Connections
        ##################################################
        self.connect((self.uhd_source, 0), (self.mult, 0))
        self.connect((self.sig_source, 0), (self.mult, 1))
        self.connect((self.mult, 0), (self.lowpass, 0))
        self.connect((self.lowpass, 0), (self.resampler, 0))
        self.connect((self.resampler, 0), (self.udp_sink, 0))

    def set_freq(self, freq):
        """Update center frequency on the fly."""
        # print(f"[INFO] Updating frequency to {freq/1e6:.2f} MHz")
        self.freq = freq
        self.uhd_source.set_center_freq(freq, 0)

    def set_gain(self, gain):
        """Update gain on the fly."""
        # print(f"[INFO] Updating gain to {gain:.1f} dB")
        self.gain = gain
        self.uhd_source.set_gain(gain, 0)

def main():
    tb = RX_FM_USRP_UDP()

    def cleanup(signum=None, frame=None):
        """Handle termination signals to stop flowgraph cleanly."""
        print("[INFO] Stop requested via signal.")
        tb.stop()
        tb.wait()
        sys.exit(0)

    # Catch Ctrl+C and termination signals
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    tb.start()
    print("[INFO] Flowgraph started. Waiting for ZMQ commands (freq/gain)...")

    try:
        while True:
            time.sleep(1)
    except Exception as e:
        print(f"[ERROR] Unexpected exception: {e}")
        cleanup()

if __name__ == '__main__':
    main()