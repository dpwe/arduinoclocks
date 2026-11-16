# Flip Clock for the badgewa.re Badger 2350.
#
# Two split-flap style cards (hours, minutes) with the date underneath.
# When a card changes, an intermediate "mid-flip" frame is shown (new top
# half, old bottom half, flap falling) before the final frame.
#
# Buttons:
#   A        toggle 12/24 hour
#   C        toggle flip animation
#   UP/DOWN  toggle dark/light background
#   B        sync time over WiFi/NTP (uses secrets.py REGION/TIMEZONE)
#   HOME     back to menu (handled by the system)
#
# The RTC holds local time, as with the stock Clock app (whose NTP/DST code
# this reuses).  set_clock.py on a host can also set it over USB.
#
# Only drawing calls used by the stock apps are used here (some other
# picovector calls hard-fault this firmware), so the digits are polygons
# from flipdigits.py, clipped to flap halves in Python.

import sys
import os

APP_DIR = "/system/apps/flipclock"
sys.path.insert(0, APP_DIR)
sys.path.insert(0, "/")
os.chdir(APP_DIR)

import time
import flipdigits

rtc.clear_alarm()

state = {
    "dark": False,
    "h24": True,
    "animate": True,
    "last": None,   # [h, m] last drawn, so we know what to flip from after sleep
}
State.load("flipclock", state)

SMALL_FONT = rom_font.absolute
LABEL_FONT = rom_font.ignore

W, H = screen.width, screen.height          # 264 x 176

# Card geometry
CARD_W = 126
CARD_H = 132
CARD_Y = 4
GAP = 6
CARD_X = ((W - 2 * CARD_W - GAP) // 2, (W + GAP) // 2)
RADIUS = 10
HALF_H = CARD_H // 2
SPLIT = CARD_Y + HALF_H
DIGIT_H = 90
DATE_Y = CARD_Y + CARD_H + 8

DAYS = ("MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY", "SUNDAY")
MONTHS = ("JANUARY", "FEBRUARY", "MARCH", "APRIL", "MAY", "JUNE", "JULY",
          "AUGUST", "SEPTEMBER", "OCTOBER", "NOVEMBER", "DECEMBER")


def palette():
    """(background, top flap, bottom flap, digits, text, flap edge)"""
    if state["dark"]:
        return (color.black, color.rgb(80, 80, 80), color.rgb(50, 50, 50),
                color.white, color.white, color.rgb(140, 140, 140))
    return (color.white, color.rgb(50, 50, 50), color.black,
            color.white, color.black, color.rgb(110, 110, 110))


# ---------------------------------------------------------------- drawing

def poly(points):
    screen.shape(shape.custom([vec2(x, y) for x, y in points]))


def draw_half(x, label, half, small=None):
    """Draw the top or bottom half of a card showing label."""
    bg, top_c, bot_c, dig_c, _, _ = palette()
    if half == "top":
        screen.pen = top_c
        screen.shape(shape.rounded_rectangle(x, CARD_Y, CARD_W, HALF_H, RADIUS, RADIUS, 0, 0))
    else:
        screen.pen = bot_c
        screen.shape(shape.rounded_rectangle(x, SPLIT, CARD_W, HALF_H, 0, 0, RADIUS, RADIUS))
    screen.pen = dig_c
    for p in flipdigits.text_polys(label, x + CARD_W / 2, SPLIT, DIGIT_H,
                                   half=half, ysplit=SPLIT):
        poly(p)
    if small and half == "top":
        screen.font = SMALL_FONT
        screen.text(small, x + 8, CARD_Y + 5)


def draw_hinge(x):
    bg = palette()[0]
    screen.pen = bg                                   # gap between flaps
    screen.shape(shape.rectangle(x, SPLIT - 1, CARD_W, 2))
    screen.pen = color.rgb(150, 150, 150)             # hinge pins
    screen.shape(shape.rounded_rectangle(x - 2, SPLIT - 7, 5, 14, 2, 2, 2, 2))
    screen.shape(shape.rounded_rectangle(x + CARD_W - 3, SPLIT - 7, 5, 14, 2, 2, 2, 2))


def draw_falling_flap(x, new_label):
    """A flap caught part-way down, past vertical: we see its back, which
    carries the new value's bottom half, foreshortened below the hinge (its
    free edge is nearer the viewer, so wider)."""
    bg, _, bot_c, dig_c, _, edge_c = palette()
    frac = 0.5                        # apparent height as fraction of a half
    fh = HALF_H * frac
    spread = 5
    cx = x + CARD_W / 2

    def squash(px, py):
        v = (py - SPLIT) / HALF_H                     # 0 at hinge, 1 at free edge
        return cx + (px - cx) * (1 + v * 2 * spread / CARD_W), SPLIT + v * fh

    screen.pen = bot_c
    poly([(x, SPLIT), (x + CARD_W, SPLIT),
          (x + CARD_W + spread, SPLIT + fh), (x - spread, SPLIT + fh)])
    screen.pen = dig_c
    for p in flipdigits.text_polys(new_label, cx, SPLIT, DIGIT_H,
                                   half="bottom", ysplit=SPLIT):
        poly([squash(px, py) for px, py in p])
    screen.pen = edge_c                               # lit free edge
    screen.shape(shape.rectangle(x - spread, SPLIT + fh, CARD_W + 2 * spread, 2))


def weekday(y, m, d):
    """0 = Monday (Sakamoto's method)."""
    if m < 3:
        y -= 1
    return (y + y // 4 - y // 100 + y // 400
            + (0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4)[m - 1] + d + 6) % 7


def draw_date(t):
    txt_c = palette()[4]
    y, mo, d = t[0], t[1], t[2]
    wd = weekday(y, mo, d)
    s = "%s %d %s" % (DAYS[wd], d, MONTHS[mo - 1])
    screen.font = LABEL_FONT
    tw, th = screen.measure_text(s)
    if tw > W - 8:
        s = "%s %d %s" % (DAYS[wd][:3], d, MONTHS[mo - 1][:3])
        tw, th = screen.measure_text(s)
    screen.pen = txt_c
    screen.text(s, (W - tw) / 2, DATE_Y)


def labels(h, m):
    """(hour label, minute label, small annotation for the hour card)"""
    if state["h24"]:
        return "%02d" % h, "%02d" % m, None
    return str(h % 12 or 12), "%02d" % m, ("AM" if h < 12 else "PM")


def draw_frame(t, prev=None):
    """Frame for time t.  If prev (h, m) is given, cards whose label changed
    are drawn mid-flip from the prev value."""
    screen.pen = palette()[0]
    screen.clear()
    screen.antialias = image.X4

    new = labels(t[3], t[4])
    old = labels(prev[0], prev[1]) if prev else new

    for i in (0, 1):
        x = CARD_X[i]
        small = new[2] if i == 0 else None
        draw_half(x, new[i], "top", small)
        if old[i] != new[i]:
            draw_half(x, old[i], "bottom")
            draw_hinge(x)
            draw_falling_flap(x, new[i])
        else:
            draw_half(x, new[i], "bottom")
            draw_hinge(x)

    draw_date(t)


# ---------------------------------------------------------------- NTP sync

def message(s):
    screen.pen = palette()[0]
    screen.clear()
    screen.font = LABEL_FONT
    screen.pen = palette()[4]
    tw, th = screen.measure_text(s)
    screen.text(s, (W - tw) / 2, (H - th) / 2)
    badge.update()


# (hemisphere, week, month, weekday, hour) in, out, minutes of change
REGIONS = {
    "us": (0, 2, 3, 6, 2, 1, 11, 6, 2, 60),
    "cuba": (0, 2, 3, 6, 0, 1, 11, 6, 1, 60),
    "eu": (0, 0, 3, 6, 1, 0, 10, 6, 1, 60),
    "moldova": (0, 0, 3, 6, 2, 0, 10, 6, 3, 60),
    "lebanon": (0, 0, 3, 6, 0, 0, 10, 6, 0, 60),
    "egypt": (0, 0, 4, 4, 0, 0, 10, 3, 24, 60),
    "chile": (1, 1, 9, 5, 24, 1, 4, 5, 24, 60),
    "australia": (1, 1, 10, 6, 2, 1, 4, 6, 3, 60),
    "nz": (1, 0, 9, 6, 2, 1, 4, 6, 3, 60),
}


def wifi_configured():
    try:
        import secrets
        return bool(secrets.WIFI_SSID) and secrets.REGION in REGIONS \
            and isinstance(secrets.TIMEZONE, (int, float))
    except Exception:
        return False


def ntp_sync():
    if not wifi_configured():
        message("No WiFi/REGION in secrets.py")
        time.sleep(2)
        return False
    import secrets
    sys.path.insert(0, "/system/apps/clock")
    try:
        from daylightsaving import DaylightSavingPolicy, DaylightSaving
    finally:
        sys.path.remove("/system/apps/clock")
    import ntptime
    import wifi
    from machine import RTC

    message("Syncing time over WiFi...")
    wifi.connect()
    while not wifi.tick():
        pass
    try:
        ntptime.settime()
    except OSError:
        message("NTP sync failed")
        time.sleep(2)
        return False

    tzm = secrets.TIMEZONE * 60
    hem, wi, mi, wdi, hi, wo, mo, wdo, ho, diff = REGIONS[secrets.REGION]
    dst = DaylightSaving(DaylightSavingPolicy(hem, wi, mi, wdi, hi, tzm + diff),
                         DaylightSavingPolicy(hem, wo, mo, wdo, ho, tzm))
    tm = time.gmtime(dst.localtime(time.mktime(time.gmtime())))
    RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))
    rtc.datetime((tm[0], tm[1], tm[2], tm[3], tm[4], tm[5], tm[6]))
    return True


# ---------------------------------------------------------------- main loop

def minute_before(hm):
    h, m = hm
    m -= 1
    if m < 0:
        m, h = 59, (h - 1) % 24
    return [h, m]


force_full = True


def update():
    global force_full

    changed = False
    if badge.pressed(BUTTON_A):
        state["h24"] = not state["h24"]
        changed = True
    if badge.pressed(BUTTON_C):
        state["animate"] = not state["animate"]
        message("Flip animation " + ("on" if state["animate"] else "off"))
        time.sleep(1)
        changed = True
    if badge.pressed(BUTTON_UP) or badge.pressed(BUTTON_DOWN):
        state["dark"] = not state["dark"]
        changed = True
    if badge.pressed(BUTTON_B) or (force_full and rtc.datetime()[0] < 2022
                                   and wifi_configured()):
        ntp_sync()
        changed = True

    t = rtc.datetime()
    hm = [t[3], t[4]]
    last = state["last"]

    if changed or force_full or hm != last:
        # Only animate a genuine one-minute tick (this also works after
        # waking from deep sleep, since the e-ink still shows the old time).
        if state["animate"] and not changed and last == minute_before(hm):
            badge.mode(FAST_UPDATE)
            draw_frame(t, last)
            badge.update()
        badge.mode(MEDIUM_UPDATE)
        draw_frame(t)
        badge.update()
        state["last"] = hm
        State.save("flipclock", state)
        force_full = False

    # Wake at the top of the next minute.  set_alarm() is relative to now
    # (minutes=1 means "in 60 s"), so aim at the boundary explicitly or the
    # redraw time accumulates and the display lags by up to a minute.
    rtc.set_alarm(seconds=60 - rtc.datetime()[5])
    wait_for_button_or_alarm(timeout=5000)


def on_exit():
    rtc.clear_alarm()
    State.save("flipclock", state)


run(update)
