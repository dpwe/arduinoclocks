"""Minimal MicroPython raw-REPL client over USB serial, standard library only.

Used by install.py and set_clock.py.  Interrupting the running badge app with
Ctrl-C drops it to the REPL; restart_app() relaunches whatever app the menu
last started (without a hard reset, which would send the badge to the menu).
"""
import glob
import os
import select
import sys
import termios
import time


def find_port(argv):
    if len(argv) > 1:
        return argv[1]
    ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*")
    if len(ports) != 1:
        sys.exit("Specify the port; found: %s (badge asleep? press a button)" % (ports or "none"))
    return ports[0]


class RawREPL:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    buf = b""

    def read_until(self, marker, timeout=10):
        """Return everything up to and including marker; keep the rest."""
        end = time.time() + timeout
        while marker not in self.buf:
            if time.time() > end:
                raise TimeoutError("waiting for %r, got %r" % (marker, self.buf[-200:]))
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                self.buf += os.read(self.fd, 4096)
        i = self.buf.index(marker) + len(marker)
        out, self.buf = self.buf[:i], self.buf[i:]
        return out

    def write(self, b):
        while b:
            n = os.write(self.fd, b)
            b = b[n:]

    def enter(self, tries=5):
        # Ctrl-C can be missed while the e-ink is refreshing, so retry.
        for i in range(tries):
            self.write(b"\r\x03\x03")
            time.sleep(0.3)
            termios.tcflush(self.fd, termios.TCIFLUSH)
            self.buf = b""
            self.write(b"\x01")
            try:
                self.read_until(b"raw REPL; CTRL-B to exit\r\n>", 3)
                return
            except TimeoutError:
                if i == tries - 1:
                    raise

    def exec(self, code, timeout=10):
        if isinstance(code, str):
            code = code.encode()
        for i in range(0, len(code), 256):
            self.write(code[i:i + 256])
            time.sleep(0.01)
        self.write(b"\x04")
        self.read_until(b"OK", timeout)
        out = self.read_until(b"\x04", timeout)[:-1]
        err = self.read_until(b"\x04", timeout)[:-1]
        self.read_until(b">", timeout)
        if err:
            raise RuntimeError(err.decode(errors="replace"))
        return out.decode(errors="replace")

    def restart_app(self):
        """Leave raw REPL and rerun the system main.py (relaunches the app)."""
        self.write(b"\x02")
        time.sleep(0.2)
        self.write(b"exec(open('/system/main.py').read())\r")
        time.sleep(0.5)

    def close(self):
        os.close(self.fd)
