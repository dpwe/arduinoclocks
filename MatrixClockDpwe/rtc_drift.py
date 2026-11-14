#!/usr/bin/env python3
"""Measure the MatrixClockDpwe's Teensy RTC drift against this computer's clock.

Usage:
  rtc_drift.py [port]                 measure offset, log it, report drift
  rtc_drift.py [port] --set-comp N    set RTC compensation to N (saved in EEPROM)
  rtc_drift.py --reset                start a new drift log segment

Each run sends "Q" a few times; the firmware replies with the RTC's Unix time
right at a seconds tick, so the reply's arrival time gives the RTC offset to
~1 ms (USB latency is a small constant bias that cancels out of drift).  The
offset is appended to a log, and the drift rate (ppm) is fitted over every
entry since the last reset, clock set (set_clock.py logs those), or
compensation change.  A day apart is enough for ~0.01 ppm resolution.
Keep the computer's clock NTP-synced.  Uses only the standard library.
"""
import argparse
import glob
import json
import os
import select
import sys
import termios
import time

LOG = os.path.expanduser("~/.matrixclock_rtc_drift.log")
PPM_PER_STEP = 0.1192   # Teensy3Clock.compensate() units.


def open_port(port):
    if port is None:
        ports = glob.glob("/dev/cu.usbmodem*")
        if len(ports) != 1:
            sys.exit("Specify the port; found: %s" % (ports or "none (charge-only cable?)"))
        port = ports[0]
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0          # raw
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B9600         # ignored by Teensy USB serial
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_line(fd, prefix, timeout):
    """Return (line, arrival_time) for the first line starting with prefix."""
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            continue
        buf += os.read(fd, 1024)
        arrival = time.time()
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.decode(errors="replace").strip()
            if line.startswith(prefix):
                return line, arrival
    return None, None


def drain(fd):
    while select.select([fd], [], [], 0.2)[0]:
        os.read(fd, 1024)


def get_comp(fd, cmd="C"):
    os.write(fd, (cmd + "\n").encode())
    line, _ = read_line(fd, "clock_compensate=", 3)
    if line is None:
        return None
    return int(line.split()[0].split("=")[1])


def measure_offset(fd, n=5):
    """RTC minus host time, in seconds (positive = RTC fast)."""
    offsets = []
    for _ in range(n):
        os.write(fd, b"Q\n")
        line, arrival = read_line(fd, "Q ", 3)
        if line is None:
            break
        # The tick happened at or before arrival, so each sample is a lower
        # bound on the offset; the max is the least delayed.
        offsets.append(int(line.split()[1]) - arrival)
    if not offsets:
        sys.exit("No reply to Q: is the clock running firmware with the Q command?")
    return max(offsets), (max(offsets) - min(offsets))


def read_log():
    if not os.path.exists(LOG):
        return []
    with open(LOG) as f:
        return [json.loads(line) for line in f if line.strip()]


def append_log(entry):
    with open(LOG, "a") as f:
        f.write(json.dumps(entry) + "\n")


def current_segment(entries):
    """Measurements since the last reset/set event or compensation change."""
    seg = []
    for e in entries:
        if "event" in e:
            seg = []
        elif seg and e["comp"] != seg[-1]["comp"]:
            seg = [e]
        else:
            seg.append(e)
    return seg


def fit_ppm(seg):
    ts = [e["t"] for e in seg]
    os_ = [e["offset"] for e in seg]
    tm, om = sum(ts) / len(ts), sum(os_) / len(os_)
    num = sum((t - tm) * (o - om) for t, o in zip(ts, os_))
    den = sum((t - tm) ** 2 for t in ts)
    return 1e6 * num / den


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("port", nargs="?")
    ap.add_argument("--set-comp", type=int, metavar="N")
    ap.add_argument("--reset", action="store_true")
    args = ap.parse_args()

    if args.reset:
        append_log({"t": time.time(), "event": "reset"})
        print("Started new drift segment in", LOG)
        return

    fd = open_port(args.port)
    drain(fd)

    if args.set_comp is not None:
        comp = get_comp(fd, "C%d" % args.set_comp)
        if comp != args.set_comp:
            sys.exit("Clock reports compensation %s, not %d" % (comp, args.set_comp))
        print("Compensation set to %d (saved in EEPROM)." % comp)
        return

    comp = get_comp(fd)
    if comp is None:
        sys.exit("No reply to C: is the clock running firmware with the C command?")
    offset, spread = measure_offset(fd)
    os.close(fd)
    now = time.time()
    append_log({"t": now, "offset": round(offset, 4), "comp": comp})
    print("comp=%d  RTC offset %+.3f s  (sample spread %.1f ms)" % (comp, offset, 1e3 * spread))

    seg = current_segment(read_log())
    span_days = (seg[-1]["t"] - seg[0]["t"]) / 86400
    if len(seg) < 2 or span_days < 0.25:
        print("Need another measurement at least 6 hours later to estimate drift.")
        return
    ppm = fit_ppm(seg)
    steps = round(ppm / PPM_PER_STEP)
    print("Drift over %d measurements, %.1f days: %+.3f ppm (%+.1f s/month)"
          % (len(seg), span_days, ppm, ppm * 1e-6 * 30 * 86400))
    if steps:
        print("To cancel it: rtc_drift.py --set-comp %d" % (comp - steps))
    else:
        print("Within one compensation step; no change needed.")


if __name__ == "__main__":
    main()
