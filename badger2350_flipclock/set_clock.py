#!/usr/bin/env python3
"""Set the Badger 2350's RTC to this computer's local time over USB serial.

Usage: set_clock.py [/dev/cu.usbmodemXXXX]

The badge firmware keeps local time in its RTC (the stock Clock app sets it
from NTP plus a DST rule), so this sends local, not UTC, time.  Sets both the
PCF85063A (survives deep sleep) and the RP2350's own RTC at the top of a
second, reads it back, then restarts the running app.  Standard library only.
"""
import datetime
import sys
import time

from rawrepl import RawREPL, find_port

port = find_port(sys.argv)
print("Using", port)
r = RawREPL(port)
r.enter()

target = int(time.time()) + 2
t = datetime.datetime.fromtimestamp(target)
code = ("from machine import RTC\n"
        "rtc.datetime((%d, %d, %d, %d, %d, %d, %d))\n"
        "RTC().datetime((%d, %d, %d, %d, %d, %d, %d, 0))\n"
        % (t.year, t.month, t.day, t.hour, t.minute, t.second, t.weekday(),
           t.year, t.month, t.day, t.weekday(), t.hour, t.minute, t.second))
while time.time() < target - 0.02:
    time.sleep(0.005)
r.exec(code)
print("Set to   ", t.strftime("%Y-%m-%d %H:%M:%S %a"))
print("Badge RTC", r.exec("print(rtc.datetime())").strip())
r.restart_app()
r.close()
