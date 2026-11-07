// awesomeclock_1632_picow
//
// Clock for 16x24 LED matrix using HT1632 driver,
// getting its time from NTP over WiFi (no RTC needed).
//
// This version is setup to use a Raspberry Pi Pico W
// (Philhower arduino-pico core, board "Raspberry Pi Pico W").
//
// WiFi credentials go in auth.h (not checked in) - copy auth.h.example.
//
// The lwIP SNTP client sets the Pico's system clock at startup and then
// re-syncs it in the background (hourly by default).  Between syncs, the
// clock free-runs on the Pico's crystal, so it keeps going if WiFi drops.

// Wiring for 16x24 LED matrix connected to Pico W (same as rp2040 version)
#define CS   6  // Green
#define DATA 8  // Blue
#define WR   9  // Yellow

// Uses a private copy of the HT1632 library in src/HT1632 (copied from
// old_HT1632, with an added setPixelMap() hook for this panel's wiring).
#include "src/HT1632/HT1632.h"
// Single matrix
HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS);

// ---------------------------------
// Time
// ---------------------------------

#include <time.h>
#include <sys/time.h>

class Time
{
public:
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   date;
  uint8_t   month;
  uint16_t  year;

  Time(uint16_t year=2000, uint8_t mon=1, uint8_t date=1, uint8_t hour=0, uint8_t min=0, uint8_t sec=0);
};

Time::Time(uint16_t year, uint8_t mon, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec)
{
  this->year = year;
  this->month  = mon;
  this->date = date;
  this->hour = hour;
  this->min  = min;
  this->sec  = sec;
}


// ----------------------------------------
// DST logic
// ----------------------------------------

int dst_cache_year = -1;
int dst_start_day = 0;
int dst_end_day = 0;

uint8_t days_in_month(uint8_t month, bool leapyear) {
  if (month == 2) return 28 + leapyear;
  return 30 + ((month + (month > 7)) % 2);
}

int day_of_year(uint8_t year, uint8_t month, uint8_t day) {
  // Jan 1st is 0
  bool leapyear = (year % 4) == 0;
  int day_of_year = 0;
  for (int i = 1; i < month; ++i) day_of_year += days_in_month(i, leapyear);
  return day_of_year + (day - 1);
}

void calc_timechange_days(int year) {
  // Jan 1st 2000 was a Saturday.  So what day is March 1st this year? 0 = Sun.
  year -= 2000;
  uint8_t march_first_dow = (6 + 365L * year + ((year + 4) / 4) + 31 + 28) % 7;
  uint8_t second_sunday_date = 14 - ((march_first_dow + 6) % 7);  // Avoid the Sunday March 1st bug.
  dst_start_day = 31 + 28 + ((year % 4) == 0) + second_sunday_date - 1;  
  // March and November are 245 days == 35.0 weeks apart, so 1st sunday in Nov is the same DOW
  dst_end_day = dst_start_day + 245 - 7;  // 1st sunday, not 2nd.
  dst_cache_year = year;
}

// ET DST begins at 2am local time on 2nd Sunday in March.
// At that point, local time is UTC-5, so 2am local is 7am UTC.
// The local time jumps to 3am.
// ET DST ends at 2am local time on 1st Sunday in November.
// At that point, local time is UTC-4, so 2am local is 6am UTC.
// The local time then slips back to 1am (UTC-5).

// Only works for UTC-1 to UTC-12
// so that 2am local is still the same date in UTC, even in DST,
// and logic only allows shift to local time to move date *backwards*.
#define STANDARD_TIME_DIFF_HOURS -5  // US ET

void make_localtime(Time &now) {
  if (dst_cache_year != now.year)  calc_timechange_days(now.year);
  // 2am local in North America is 2am + 4/5 in UTC
  int DoY = day_of_year(now.year - 2000, now.month, now.date);
  bool sprung_forward = (DoY > dst_start_day) ||
                        ((DoY == dst_start_day) && (now.hour >= 2 - STANDARD_TIME_DIFF_HOURS));
  bool fallen_back = (DoY > dst_end_day) ||
                      ((DoY == dst_end_day) && (now.hour >= 2 - (STANDARD_TIME_DIFF_HOURS + 1)));  // offset is DST
  bool is_dst = sprung_forward - fallen_back;
  int signed_hour = now.hour;  // Cast hour out of unsigned type to allow it to be < 0 after shifting.
  signed_hour += STANDARD_TIME_DIFF_HOURS + is_dst;
  if (signed_hour < 0) {
    signed_hour += 24;
    now.date -= 1;
    if (now.date == 0) {
      now.month -= 1;
      if (now.month == 0) {
        now.month = 12;
        now.year -= 1;
      }
      now.date = days_in_month(now.month, (now.year % 4) == 0);
    }
  }
  now.hour = signed_hour;
}


// ---------------------------------
// Matrix display
// ---------------------------------

// This panel's 8x8 blocks are wired differently from the AliExpress panel the
// library assumes: with the library's mapping, each 8x8 appears rotated 90 deg
// CCW, and the blocks appear in order 3 2 1 / 6 5 4 instead of 1 2 3 / 4 5 6.
// Pre-apply the inverse: mirror the block column and rotate each block 90 deg CW.
// (If a block still looks rotated 180 deg, swap to: lx = v; ly = 7 - u.)
void panel_pixel_map(uint8_t &x, uint8_t &y) {
  if (x >= 24 || y >= 16) return;  // Leave out-of-range for drawPixel to reject.
  uint8_t bx = x / 8, by = y / 8;  // Block column (0..2) and row (0..1).
  uint8_t u = x % 8, v = y % 8;    // Position within the block.
  uint8_t lx = 7 - v, ly = u;      // Undo 90 deg CCW rotation.
  x = 8 * (2 - bx) + lx;           // Undo 3-2-1 block order.
  y = 8 * by + ly;
}

void matrix_setup() {
  matrix.setPixelMap(panel_pixel_map);
  matrix.begin(HT1632_COMMON_16NMOS);
  matrix.clearScreen();
  matrix.setBrightness(0);
}

void draw_seconds(int seconds, int minutes) {
  // Seconds in 6 groups of 2x5 pixels
  // Turn on when minutes is even, turn off when minutes is off.
  if (seconds == 0) {
    seconds = 60;
    minutes -= 1;
  }
  seconds -= 1;  // We light the bottom-left LED at seconds == 1.
  int y = 13 - (seconds % 5);
  int x = 3 + 3 * (seconds / 10) + ((seconds % 10) >= 5) + (seconds >= 30);
  if ((minutes & 1) == 0) {
    matrix.setPixel(x, y);
  } else {
    matrix.clrPixel(x, y);
  }
}

char state[4] = {' ', ' ', ' ', ' '};

uint8_t transition_state = 0;
const uint8_t num_transition_states = 7;

void draw_hours_minutes(int minutes, int hours) {
  matrix.setTextSize(1);    // size 1 == 8 pixels high
  char new_state[4];
  new_state[0] = '0' + (hours / 10);
  new_state[1] = '0' + (hours % 10);
  new_state[2] = '0' + (minutes / 10);
  new_state[3] = '0' + (minutes % 10);
  bool state_changed = new_state[3] != state[3];
  if (state_changed) {
    for (int i = 0; i < 4; ++i) {
      if (new_state[i] != state[i]) {
        int x = 6 * i + (i > 1);  // 2nd two chars are 1 pixel over.
        // Undraw previous transition.
        matrix.drawCharTx(x, 0, state[i], new_state[i], 0, 1, transition_state);
        // Draw next transition.
        matrix.drawCharTx(x, 0, state[i], new_state[i], 1, 1, transition_state + 1);
      }
    }
    // Update state change status for each rows.
    transition_state += 1;
    if (transition_state >= num_transition_states) {
      transition_state = 0;
      for (int i = 0; i < 4; ++i) {
        state[i] = new_state[i];
      }
      // Copying new_state to state implicitly releases state_changed.
    }
  }
}

void matrix_update(uint8_t hour, uint8_t min, uint8_t sec) {
  draw_hours_minutes(min, hour);
  draw_seconds(sec, min);
  matrix.writeScreen();
}

// ---------------------------------
// Backlight
// ---------------------------------

// Config for backlight day/night mode.
const int light_low = 0;
const int light_high = 1;
const int hour_up = 7;
const int hour_down = 22;

void backlight_setup(void) {
  // Backlight
}

int brightness = 0;
int bright_tick = 0;
const int ticks_per_step = 1;

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void backlight_update(int hour) {
  int target_brightness = light_low;
  if (hour >= hour_up && hour < hour_down) {
    target_brightness = light_high;
  }
  if (++bright_tick >= ticks_per_step) {
    // Slow down the brightness change steps.
    bright_tick = 0;
    int bright_delta = target_brightness - brightness;
    if (bright_delta) {
      brightness += sgn(bright_delta);
      matrix.setBrightness(brightness);
    }
  }
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void serial_print_tm(const Time t)
{
  Serial.print(t.year);
  Serial.print("-");
  printDigits(t.month);
  Serial.print("-");
  printDigits(t.date);
  Serial.print(" ");
  printDigits(t.hour);
  Serial.print(":");
  printDigits(t.min);
  Serial.print(":");
  printDigits(t.sec);
  Serial.println();
}

// ---------------------------------
// WiFi + NTP
// ---------------------------------

#include <WiFi.h>

// Defines ssid and password for WiFi.
#include "auth.h"

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";

// Any time before this means the clock has never been set (epoch is 1970).
const time_t VALID_TIME = 1700000000;  // Nov 2023

bool time_is_valid(void) {
  return time(nullptr) > VALID_TIME;
}

// Show a "busy" animation while waiting for WiFi/NTP: walk one seconds-dot around.
int busy_step = 0;
void show_busy(void) {
  // draw_seconds lights dot on even minutes, clears it on odd.
  draw_seconds(busy_step % 60, busy_step / 60);
  busy_step = (busy_step + 1) % 120;
  matrix.writeScreen();
}

bool wifi_connect(uint32_t timeout_ms = 30000) {
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeout_ms) {
      Serial.println("WiFi connect timed out.");
      return false;
    }
    show_busy();
    delay(100);
  }
  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());
  return true;
}

bool ntp_started = false;

void ntp_start(void) {
  // NTP.begin resolves server names, so must be called once WiFi is up.
  NTP.begin(ntpServer1, ntpServer2);
  ntp_started = true;
}

void wifi_ntp_setup(void) {
  // Keep trying until we have a valid time - there's nothing to display until then.
  while (!time_is_valid()) {
    if (WiFi.status() != WL_CONNECTED) {
      if (!wifi_connect()) continue;
    }
    if (!ntp_started) ntp_start();
    Serial.println("Waiting for NTP time...");
    uint32_t start = millis();
    while (!time_is_valid() && millis() - start < 15000) {
      show_busy();
      delay(100);
    }
    if (!time_is_valid()) {
      // Maybe DNS failed or servers unreachable - restart SNTP.
      Serial.println("No NTP response, retrying.");
      ntp_started = false;
    }
  }
  // Clear any busy-animation dots.
  matrix.clearScreen();
  Serial.println("Time set from NTP.");
}

// Check WiFi occasionally and reconnect if it has dropped, so SNTP can keep re-syncing.
uint32_t last_wifi_check_ms = 0;
const uint32_t WIFI_CHECK_INTERVAL_MS = 60000;

void wifi_maintain(void) {
  if (millis() - last_wifi_check_ms < WIFI_CHECK_INTERVAL_MS) return;
  last_wifi_check_ms = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting.");
    // Short timeout - the clock keeps running on its own meanwhile.
    WiFi.begin(ssid, password);
    // Don't block; next check will see if it worked.
  }
}

Time time_from_epoch(time_t t) {
  struct tm tm;
  gmtime_r(&t, &tm);
  return Time(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

Time utc_now(void) {
  return time_from_epoch(time(nullptr));
}

// -------------------------------------------------------------------
// Input commands over serial line
// T - print current UTC and local time, WiFi status.

void print_status(void) {
  Time t = utc_now();
  Serial.print("UTC:   ");
  serial_print_tm(t);
  make_localtime(t);
  Serial.print("Local: ");
  serial_print_tm(t);
  Serial.print("WiFi: ");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("connected, RSSI=");
    Serial.print(WiFi.RSSI());
    Serial.print(" IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("not connected");
  }
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(void) {
  while (Serial.available() > 0) {
    char new_char = Serial.read();
    if (new_char == '\n' || new_char == '\r') {
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        char cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'T':
            print_status();
            break;
          default:
            Serial.println("Commands: T (show time/status)");
            break;
        }
      }
      cmd_len = 0;
    } else {
      if (cmd_len < CMD_BUF_LEN - 1) {
        cmd_buffer[cmd_len++] = new_char;
      }
    }
  }
}

// ---------------------------------
// Main
// ---------------------------------

#define MAXWAIT_SERIAL 300  // 10 ms units, so 3 seconds.
void open_serial(int baudrate = 115200) {
  Serial.begin(baudrate);
  // Wait (a little) for Serial port to open.
  int i = 0;
  while (!Serial && i < MAXWAIT_SERIAL) {
    delay(10);
    ++i;
  }
}

void serial_print_progname(void) {
  Serial.print(F("awesomeclock_1632_picow "));
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
}

void setup() {
  open_serial();
  serial_print_progname();

  pinMode(CS, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(DATA, OUTPUT);
  matrix_setup();
  backlight_setup();

  wifi_ntp_setup();
  print_status();
}

time_t last_sec = 0;

Time wait_for_next_sec(void) {
  // Poll the system clock until the second rolls over.
  time_t t;
  while ((t = time(nullptr)) == last_sec) {
    delay(5);
  }
  last_sec = t;
  return time_from_epoch(t);
}

void loop() {
  Time now = wait_for_next_sec();
  unsigned long millis_start = millis();
  make_localtime(now);

  matrix_update(now.hour, now.min, now.sec);
  while(transition_state != 0) {
    delay(20);
    matrix_update(now.hour, now.min, now.sec);
  }
  backlight_update(now.hour);

  cmd_update();
  wifi_maintain();

  // Sleep most of the rest of the second, then poll for the rollover.
  long millis_to_wait = 900 - (long)(millis() - millis_start);
  if (millis_to_wait > 0) delay(millis_to_wait);
}
