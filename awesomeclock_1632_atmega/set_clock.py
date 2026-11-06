#!/usr/bin/env python3
"""Set the awesomeclock's DS3231 to the current UTC time over serial.

Usage: set_clock.py /dev/cu.usbserial-XXXX [baud]

Sends "Z<yyyymmddhhmmss>" (UTC) at the top of a second, then echoes the
time the clock reports back.  Default baud is 9600, which is what the
deployed atmega board talks at.  Uses only the standard library (no pyserial).
"""
import datetime
import os
import select
import sys
import termios
import time

port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[0] = 0                                    # iflag
attrs[1] = 0                                    # oflag
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0                                    # lflag (raw)
attrs[4] = attrs[5] = getattr(termios, "B%d" % baud)
termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_for(secs):
    out = b""
    end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            out += os.read(fd, 1024)
    return out.decode(errors="replace")


# Opening the port may reset the board via DTR; let the bootloader + setup finish.
print("--- startup output ---")
print(read_for(4))

# Ask for current time without setting (short Z prints error + current time).
os.write(fd, b"Z\n")
print("--- before ---")
print(read_for(1))

# Send at the top of the next second, so whole-second time is accurate.
now = time.time()
time.sleep(1 - (now % 1))
t = datetime.datetime.now(datetime.timezone.utc)
cmd = "Z" + t.strftime("%Y%m%d%H%M%S") + "\n"
os.write(fd, cmd.encode())
print("--- sent", cmd.strip(), "---")
print(read_for(1.5))
os.close(fd)
