# Polygon digits for the flip clock.
#
# Each digit is a set of centre-line strokes in a box 0.58 wide x 1.0 tall.
# Strokes are turned into small convex polygons (a quad per segment plus a
# round joint at each interior vertex), so they can be clipped to the top or
# bottom half of a flap in pure Python and drawn with shape.custom().
# No badge dependencies, so it can also be run on a host for previews.

import math

DW = 0.58          # digit width (height is 1.0)
T = 0.15           # stroke thickness

_l, _r, _t, _b = T / 2, DW - T / 2, T / 2, 1 - T / 2
_xm = DW / 2


def _arc(cx, cy, rx, ry, a0, a1, step=12):
    n = max(2, int(abs(a1 - a0) / step) + 1)
    pts = []
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        pts.append((cx + rx * math.cos(a), cy + ry * math.sin(a)))
    return pts


def _ellipse(cx, cy, rx, ry):
    p = _arc(cx, cy, rx, ry, 0, 360)
    return p[:-1], True          # closed


def _flip(strokes):
    # rotate 180 degrees within the digit box (6 -> 9)
    return [([(DW - x, 1 - y) for x, y in pts], closed) for pts, closed in strokes]


def _strokes():
    rx = _xm - _l
    d = {}
    d["0"] = [(_arc(_xm, _t + rx, rx, rx, 180, 360) +
               _arc(_xm, _b - rx, rx, rx, 0, 180), True)]
    d["1"] = [([(0.10, 0.25), (0.34, _t), (0.34, _b)], False)]
    d["2"] = [(_arc(_xm, _t + 0.21, rx, 0.21, 190, 395) + [(_l, _b), (_r + 0.01, _b)], False)]
    d["3"] = [(_arc(_xm, _t + 0.19, rx - 0.015, 0.19, 200, 450) +
               _arc(_xm, 0.455 + 0.235, rx, 0.235, 270, 520), False)]
    d["4"] = [([(0.41, _b), (0.41, _t), (_l, 0.66), (_r, 0.66)], False)]
    d["5"] = [([(_r, _t), (0.11, _t), (0.09, 0.48)] +
               _arc(_xm, 0.68, rx, 0.245, 222, 520), False)]
    d["6"] = [_ellipse(_xm, 0.68, rx, 0.245),
              ([(_l + 0.01, 0.62), (0.42, _t)], False)]
    d["9"] = _flip(d["6"])
    d["7"] = [([(_l, _t), (_r, _t), (0.18, _b)], False)]
    d["8"] = [_ellipse(_xm, _t + 0.185, rx - 0.025, 0.185),
              _ellipse(_xm, _b - 0.235, rx, 0.235)]
    return d


STROKES = _strokes()


def _disc(cx, cy, r, n=12):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def _polys_for(ch):
    """Unit-box polygons for one digit."""
    out = []
    h = T / 2
    for pts, closed in STROKES[ch]:
        segs = list(zip(pts, pts[1:]))
        if closed:
            segs.append((pts[-1], pts[0]))
        for (x0, y0), (x1, y1) in segs:
            dx, dy = x1 - x0, y1 - y0
            ln = math.sqrt(dx * dx + dy * dy) or 1
            nx, ny = -dy / ln * h, dx / ln * h
            out.append([(x0 + nx, y0 + ny), (x1 + nx, y1 + ny),
                        (x1 - nx, y1 - ny), (x0 - nx, y0 - ny)])
        joints = pts if closed else pts[1:-1]
        for x, y in joints:
            out.append(_disc(x, y, h))
    return out


UNIT_POLYS = {ch: _polys_for(ch) for ch in STROKES}


def _clip(poly, ysplit, keep_above):
    """Sutherland-Hodgman clip of a convex polygon against y = ysplit."""
    def inside(p):
        return p[1] <= ysplit if keep_above else p[1] >= ysplit

    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ia, ib = inside(a), inside(b)
        if ia:
            out.append(a)
        if ia != ib:
            t = (ysplit - a[1]) / (b[1] - a[1])
            out.append((a[0] + (b[0] - a[0]) * t, ysplit))
    return out if len(out) >= 3 else None


def text_polys(s, cx, cy, height, gap=0.10, half=None, ysplit=None):
    """Polygons (screen coords) for string s of digits centred on (cx, cy).
    half: None, 'top' or 'bottom' to clip at ysplit."""
    n = len(s)
    total_w = (n * DW + (n - 1) * gap) * height
    x = cx - total_w / 2
    y = cy - height / 2
    polys = []
    for ch in s:
        for p in UNIT_POLYS.get(ch, ()):
            q = [(x + px * height, y + py * height) for px, py in p]
            if half is not None:
                q = _clip(q, ysplit, half == "top")
                if q is None:
                    continue
            polys.append(q)
        x += (DW + gap) * height
    return polys
