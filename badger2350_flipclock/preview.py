#!/usr/bin/env python3
"""Render the flip clock's screens to a PNG on the host, without a badge.

Usage: preview.py [out.png] [HH:MM]

Runs flipclock/__init__.py against minimal fakes of the badgeware drawing API
(polygon shapes only; text is shown as grey boxes), and rasterises at 3x,
showing light, light mid-flip, dark and 12-hour frames.  Standard library only.
"""
import math
import os
import struct
import sys
import types
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
SS = 3                      # supersampling / output scale
W, H = 264, 176


class Col:
    def __init__(self, r, g, b, a=255):
        self.v = (r + g + b) / 3 / 255.0


class Canvas:
    def __init__(self):
        self.px = [[1.0] * (W * SS) for _ in range(H * SS)]
        self.pen = Col(0, 0, 0)
        self.font = None
        self.antialias = None
        self.width, self.height = W, H

    def clear(self):
        for row in self.px:
            for i in range(len(row)):
                row[i] = self.pen.v

    def fill_poly(self, pts, v):
        pts = [(x * SS, y * SS) for x, y in pts]
        ys = [p[1] for p in pts]
        n = len(pts)
        for yi in range(max(0, int(min(ys))), min(H * SS, int(math.ceil(max(ys))) + 1)):
            yc = yi + 0.5
            xs = []
            for i in range(n):
                (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
                if (y0 <= yc < y1) or (y1 <= yc < y0):
                    xs.append(x0 + (yc - y0) * (x1 - x0) / (y1 - y0))
            xs.sort()
            row = self.px[yi]
            for a, b in zip(xs[0::2], xs[1::2]):
                for xi in range(max(0, int(math.ceil(a - 0.5))), min(W * SS, int(math.ceil(b - 0.5)))):
                    row[xi] = v

    def shape(self, s):
        self.fill_poly(s, self.pen.v)

    def measure_text(self, s):
        return (len(s) * 11, 20)

    def text(self, s, x, y):
        w, h = self.measure_text(s)
        self.fill_poly([(x, y + 4), (x + w, y + 4), (x + w, y + h - 4), (x, y + h - 4)],
                       0.5 * self.pen.v + 0.25)


def rrect(x, y, w, h, r1=0, r2=0, r3=0, r4=0):
    pts = []
    for (cx, cy, r, a0) in ((x + r1, y + r1, r1, 180), (x + w - r2, y + r2, r2, 270),
                            (x + w - r3, y + h - r3, r3, 0), (x + r4, y + h - r4, r4, 90)):
        for i in range(7):
            a = math.radians(a0 + 15 * i)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def make_env(canvas):
    shape = types.SimpleNamespace(
        custom=lambda pts: [(p[0], p[1]) for p in pts],
        rectangle=lambda x, y, w, h: [(x, y), (x + w, y), (x + w, y + h), (x, y + h)],
        rounded_rectangle=rrect,
    )
    color = types.SimpleNamespace(rgb=Col, white=Col(255, 255, 255), black=Col(0, 0, 0))
    env = dict(
        screen=canvas, shape=shape, color=color, vec2=lambda x, y: (x, y),
        image=types.SimpleNamespace(X4=4),
        rom_font=types.SimpleNamespace(absolute=None, ignore=None),
        rtc=types.SimpleNamespace(clear_alarm=lambda: None),
        State=types.SimpleNamespace(load=lambda n, s: None, save=lambda n, s: None),
        run=lambda f: None,
    )
    return env


def load_app(canvas):
    env = make_env(canvas)
    sys.path.insert(0, os.path.join(HERE, "flipclock"))
    src = open(os.path.join(HERE, "flipclock", "__init__.py")).read()
    src = src.replace('os.chdir(APP_DIR)', 'pass')
    env["__name__"] = "flipclock"
    exec(compile(src, "flipclock/__init__.py", "exec"), env)
    return env


def frame(hh, mm, dark=False, h24=True, prev=None):
    c = Canvas()
    app = load_app(c)
    app["state"].update(dark=dark, h24=h24)
    app["draw_frame"]((2026, 11, 16, hh, mm, 0, 0), prev)
    # downsample
    out = []
    for y in range(H):
        row = []
        for x in range(W):
            s = 0
            for dy in range(SS):
                r = c.px[y * SS + dy]
                s += sum(r[x * SS:(x + 1) * SS])
            row.append(s / (SS * SS))
        out.append(row)
    return out


def write_png(path, rows):
    h, w = len(rows), len(rows[0])
    raw = b"".join(b"\0" + bytes(int(max(0, min(1, v)) * 255) for v in r) for r in rows)
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d))
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
    open(path, "wb").write(png)


def upscale(rows, k):
    return [[v for v in r for _ in range(k)] for r in rows for _ in range(k)]


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "preview.png"
    hh, mm = (int(v) for v in (sys.argv[2] if len(sys.argv) > 2 else "19:40").split(":"))
    pm, ph = (mm - 1) % 60, hh if mm else (hh - 1) % 24
    frames = [frame(hh, mm), frame(hh, mm, prev=[ph, pm]),
              frame(hh, mm, dark=True), frame(hh, mm, h24=False)]
    pad = [[0.6] * 8 for _ in range(H)]
    top = [a + p + b for a, p, b in zip(frames[0], pad, frames[1])]
    bot = [a + p + b for a, p, b in zip(frames[2], pad, frames[3])]
    sheet = top + [[0.6] * len(top[0]) for _ in range(8)] + bot
    write_png(out, upscale(sheet, 2))
    print("wrote", out)
