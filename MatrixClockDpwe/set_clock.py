#!/usr/bin/env python3
"""Set the MatrixClockDpwe's Teensy 3.6 RTC to the current UTC time over USB serial.

Usage: set_clock.py [/dev/cu.usbmodemXXXX]

With no port given, uses the single /dev/cu.usbmodem* device if there is
exactly one.  Sends "Z<yyyymmddhhmmss>" (UTC; the firmware applies US Eastern
DST itself) at the top of a second, then echoes the clock's reply.  The
firmware resyncs by blocking for the next RTC tick, so the reply can take a
second or two to arrive.  Uses only the standard library (no pyserial).
"""
import datetime
import glob
import os
import select
import sys
import termios
import time

if len(sys.argv) > 1:
    port = sys.argv[1]
else:
    ports = glob.glob("/dev/cu.usbmodem*")
    if len(ports) != 1:
        sys.exit("Specify the port; found: %s" % (ports or "none (charge-only cable?)"))
    port = ports[0]
print("Using", port)

fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[0] = 0                                    # iflag
attrs[1] = 0                                    # oflag
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0                                    # lflag (raw)
# Teensy USB serial ignores baud; set something sane anyway.
attrs[4] = attrs[5] = termios.B9600
termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_for(secs):
    out = b""
    end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            out += os.read(fd, 1024)
    return out.decode(errors="replace")


# Teensy doesn't reset on open, but drain anything already queued.
read_for(0.5)

# Send exactly at the top of a second, so the whole-second time is accurate.
target = int(time.time()) + 2
while time.time() < target - 0.005:
    time.sleep(0.001)
cmd = "Z" + datetime.datetime.fromtimestamp(
    target, datetime.timezone.utc).strftime("%Y%m%d%H%M%S") + "\n"
while time.time() < target:
    pass
os.write(fd, cmd.encode())
print("--- sent", cmd.strip(), "(UTC) ---")
print(read_for(4))
os.close(fd)
