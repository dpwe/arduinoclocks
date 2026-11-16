# badger2350_flipclock

Split-flap ("flip") clock app for the badgewa.re / Pimoroni **Badger 2350**
(264x176 e-ink, MicroPython `bw-1.27.0` firmware).

Two cards (hours, minutes) with the date underneath.  On each minute tick it
shows one "mid-flip" frame (new top half, old bottom half, a flap falling
over the hinge) before the final frame.  Between minutes the badge waits on
an RTC alarm, so it deep-sleeps on battery like the stock Clock app.

Buttons: **A** 12/24 h, **C** flip animation on/off, **UP/DOWN** light/dark,
**B** sync over WiFi/NTP (needs `WIFI_SSID`, `REGION`, `TIMEZONE` in the
badge's `secrets.py`), **HOME** back to the menu.

## Files

- `flipclock/` – the app (`/system/apps/flipclock` on the badge).
  `flipdigits.py` draws the digits as polygons, clipped to flap halves in
  Python.  (Loading the `.af` vector fonts or calling `shape.line()` /
  `screen.clip` hard-faults this firmware, so only drawing calls used by the
  stock apps are used.)
- `install.py` – copies the app over: switches the badge into its Mass
  Storage app (`/system` is read-only FAT otherwise), copies to the `BADGER`
  drive, ejects, and starts the app.
- `set_clock.py` – sets the badge RTC to the Mac's **local** time (the badge
  firmware keeps local time in the RTC).
- `preview.py` – renders light / mid-flip / dark / 12 h frames to a PNG on the
  host using fakes of the drawing API, for iterating without the badge.
- `rawrepl.py` – tiny stdlib raw-REPL client used by the above.

```
python3 install.py
python3 set_clock.py
```
