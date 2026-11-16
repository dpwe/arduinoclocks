#!/usr/bin/env python3
"""Install the flipclock app onto a Badger 2350 and start it.

Usage: install.py [/dev/cu.usbmodemXXXX]

/system (where apps live) is a FAT volume that the firmware mounts read-only,
because it is shared over USB mass storage.  So this interrupts the running
app, launches the badge's Mass Storage app, copies flipclock/ onto the BADGER
drive (macOS), ejects it, then marks flipclock as the running app and starts
it.  Standard library only (see rawrepl.py).
"""
import glob
import os
import shutil
import subprocess
import sys
import time

from rawrepl import RawREPL, find_port

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "flipclock")
VOLUME = "/Volumes/BADGER"
FILES = ("__init__.py", "flipdigits.py", "icon.png")


def wait_for(pred, secs, what):
    end = time.time() + secs
    while not pred():
        if time.time() > end:
            sys.exit("Timed out waiting for " + what)
        time.sleep(0.5)


port = find_port(sys.argv)
print("Using", port)

if not os.path.isdir(VOLUME):
    r = RawREPL(port)
    r.enter()
    r.write(b"\x02")
    time.sleep(0.2)
    r.write(b"launch('/system/apps/mass_storage')\r")
    time.sleep(0.5)
    r.close()
    wait_for(lambda: os.path.isdir(VOLUME + "/apps"), 30, VOLUME)

dest = os.path.join(VOLUME, "apps", "flipclock")
os.makedirs(dest, exist_ok=True)
for junk in glob.glob(os.path.join(dest, "._*")):
    os.remove(junk)
for name in FILES:
    shutil.copyfile(os.path.join(SRC, name), os.path.join(dest, name))  # no xattrs -> no ._ files
    print("  copied", name)
subprocess.run(["sync"])
subprocess.run(["diskutil", "eject", VOLUME], check=True)

# After eject the badge resets and its serial port comes back.
wait_for(lambda: glob.glob(port), 30, port)
time.sleep(1)
r = RawREPL(port)
r.enter()
r.exec("s = {'active': 0, 'running': '/system/apps/menu'}\n"
       "State.load('menu', s)\n"
       "s['running'] = '/system/apps/flipclock'\n"
       "State.save('menu', s)\n")
r.restart_app()
r.close()
print("Installed and started.")
