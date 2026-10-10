#!/usr/bin/env python3
# =============================================================================
# pc_frame.py - send a frame to the UART network interface and print the reply
# -----------------------------------------------------------------------------
# The UART NI (node 3) turns bytes from the PC into network packets.  In FRAME
# mode a frame is   [wlen] [w bytes ...] [rlen]   and the reply comes back as
# rlen raw bytes.  In ADDRESSED mode the frame starts with [dest] [arg].
#
#   pip install pyserial
#   python tools/pc_frame.py COM5 01 9F 03            (flash ID -> 20 BA 19)
#   python tools/pc_frame.py COM5 --addr 05 4B 01 00 02   (TMP2 -> 0C 80, SW7 = 1)
#   python tools/pc_frame.py /dev/ttyUSB1 01 9F 03
#
# Arguments after the port are hex bytes.  The script waits up to --timeout
# seconds for the reply and prints every byte it receives.  With --temp the
# last two bytes are also shown as a TMP2 temperature.
# =============================================================================
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: run  pip install pyserial")


def main():
    p = argparse.ArgumentParser(description="Send a frame to the v2 UART NI")
    p.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB1")
    p.add_argument("bytes", nargs="+", help="frame bytes in hex, e.g. 01 9F 03")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--addr", action="store_true",
                   help="just a reminder that the frame is an ADDRESSED one (SW7 = 1)")
    p.add_argument("--temp", action="store_true",
                   help="decode the last two reply bytes as a TMP2 temperature")
    a = p.parse_args()

    frame = bytes(int(b, 16) for b in a.bytes)
    with serial.Serial(a.port, a.baud, timeout=0.05) as s:
        s.reset_input_buffer()
        s.write(frame)
        print("sent :", " ".join("%02X" % b for b in frame))
        reply = bytearray()
        end = time.time() + a.timeout
        while time.time() < end:
            reply += s.read(64)
        print("reply:", " ".join("%02X" % b for b in reply) if reply else "(nothing)")
        if a.temp and len(reply) >= 2:
            raw = (reply[-2] << 8) | reply[-1]
            print("temperature: %.4f C" % ((raw >> 3) * 0.0625))


if __name__ == "__main__":
    main()
