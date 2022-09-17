/*
    gfx_clock_bto based on gfx_clock_logger.
    Gets time from sntp library provided on ESP32.

   Wiring:
     Designed to run on Adafruit ESP32-S3 TFT with built-in ST7789

*/

// Enable serial monitor?
bool serial_available = false;

// Do we use the backlight?
#define BACKLIGHT

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int backlightPin = TFT_BACKLITE;  // PWM output to drive dimmable backlight


// ======================================================
// Track RTC times via WiFi/sntp.
// ======================================================

#include <WiFi.h>
#include "time.h"
#include "sntp.h"

//const char* ssid     = "YOUR_WIFI_SSID";
//const char* password = "YOUR_WIFI_PASSWD";
// Define ssid and password for my WiFi
#include "auth.h"

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";

const char* time_zone = "EST5EDT,M3.2.0,M11.1.0"; // "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

volatile bool ntp_adjusted = false;

// Callback function (called when time is adjusted via NTP).
void timeavailable(struct timeval *tv)
{
  struct tm * timeinfo;
  timeinfo = localtime (&tv->tv_sec);
  Serial.print("Got time adjustment from NTP: ");
  Serial.println(timeinfo, "%A, %B %d %Y %H:%M:%S");
  ntp_adjusted = true;
}

void setup_RTC()
{
  // Request updates every 3 hours, empirically. (see 
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html )
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  // Set notification call-back function.
  sntp_set_time_sync_notification_cb(timeavailable);

  // Setup sntp polling, including DST switching via specified time_zone.
  configTzTime(time_zone, ntpServer1, ntpServer2);

  // Connect to WiFi.
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
}

int last_sec = 0;

bool update_RTC(void) {
  // Returns true each time an RTC interrupt is cleared, i.e. 1/sec.
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return false;
  }
  if (last_sec != timeinfo.tm_sec) {
    last_sec = timeinfo.tm_sec;
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    return true;
  }
  return false;
}

time_t now_local(void) {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    return 0;
  }
  return mktime(&timeinfo);
}

// --------------------------
// Time formatting
// --------------------------

//#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

char *sprint_int2(char *s, uint8_t n)
{ // Always 2 digits, assume n nonnegative, < 99.
  *s++ = '0' + (n / 10);
  *s++ = '0' + (n % 10);
  return s;
}

char *sprint_int(char *s, int n, int decimal_place = 0)
{ // Print a signed int as may places as needed.
  // returns next char* to write to string.
  // If decimal place > 0, assumed passed int is value * 10**decimal_place
  // and print a decimal place.
  if (n < 0) {
    *s++ = '-';
    n = -n;
  }
  if (n > 9 || decimal_place > 0) {
    s = sprint_int(s, n / 10, decimal_place - 1);
  }
  if (decimal_place == 1) {
    *s++ = '.';
  }
  *s++ = '0' + (n % 10);
  return s;
}

char dow_names[] = "SunMonTueWedThuFriSat";

void sprint_date(const struct tm *timeinfo, char* datestr, bool add_spaces=false) {
  // Fake dt.toString for our format.  Pad each end with spaces to get proper clearing of previous.
  int dow = timeinfo->tm_wday;
  char *s = datestr;
  if (add_spaces) *s++ = ' ';
  for (int i = 0; i < 3; ++i) {
    *s++ = dow_names[3 * dow + i];
  }
  *s++ = ' ';
  s = sprint_int(s, timeinfo->tm_year + 1900);
  *s++ = '-';
  s = sprint_int2(s, timeinfo->tm_mon + 1);
  *s++ = '-';
  s = sprint_int2(s, timeinfo->tm_mday);
  if (add_spaces) *s++ = ' ';
  *s++ = '\0';
}

void sprint_time(const struct tm *timeinfo, char* datestr, bool add_spaces=false) {
  // Fake dt.toString for our format.  Pad each end with spaces to get proper clearing of previous.
  char *s = datestr;
  if (add_spaces) *s++ = ' ';
  s = sprint_int2(s, timeinfo->tm_hour);
  *s++ = ':';
  s = sprint_int2(s, timeinfo->tm_min);
  *s++ = ':';
  s = sprint_int2(s, timeinfo->tm_sec);
  if (add_spaces) *s++ = ' ';
  *s++ = '\0';
}
void sprint_tm(const struct tm *timeinfo, char *s) {
  // Format as YYYY-MM-DD HH:MM:SS
  sprint_date(timeinfo, s);
  s += strlen(s);
  *s++ = ' ';
  sprint_time(timeinfo, s);
}

void serial_print_tm(const struct tm *timeinfo)
{
  if (serial_available) {
    char s[32];
    sprint_tm(timeinfo, s);
    Serial.println(s);
  }
}


//------------------------
// Display
//------------------------

// Layout
const uint16_t overall_top_y = -4;
const uint16_t date_midline_y = overall_top_y + 16;
const uint16_t time_midline_y = overall_top_y + 46;
const uint16_t seconds_midline_y = overall_top_y + 66;
const uint16_t log_top_y = overall_top_y + 88;
const uint16_t ntpstat_mid_y = 114;
const uint16_t batstat_mid_y = 124;
const uint16_t log_width = 192;
const uint16_t log_height = 32;
const uint16_t display_mid_x = 120;
const uint8_t secs_x_scale = 3;
const uint8_t secs_height = 8;

#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#define SMALLFONT &FreeSans12pt7b
#define MICROFONT 0
#define CalBlk36 &FreeSansBold24pt7b

uint16_t big_colon_width = 0;
uint16_t big_colon_height = 0;
uint16_t digits_width = 0;
uint16_t digits_height = 0;
uint16_t date_width = 0;
const char *datefmt = "DDD YYYY-MM-DD";

// 565 RGB 16-bit colors
int fgcolor = 0; // ST77XX_WHITE;
int bgcolor = 0x0847E6; // Bright blueish-green // ST77XX_BLACK;

enum text_alignment {
  TOP,
  MIDDLE,
  BOTTOM,
  LEFT,
  RIGHT,
};

void print_text(const char *s, int16_t x, int16_t y,
                int8_t x_align = MIDDLE, int8_t y_align = MIDDLE,
                int16_t clear_width = 0, int16_t clear_height = 0, bool debug = false) {
  // Set clear_width to -1 to skip clear-before-write.
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  if (debug && serial_available) {
    Serial.print("print_text: x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.print(y);
    Serial.print(" x1=");
    Serial.print(x1);
    Serial.print(" y1=");
    Serial.print(y1);
    Serial.print(" w=");
    Serial.print(w);
    Serial.print(" h=");
    Serial.print(h);
    Serial.print(" c_w=");
    Serial.print(clear_width);
    Serial.print(" c_h=");
    Serial.println(clear_height);
  }
  y1 = y;
  x1 = x;
  if (x_align == MIDDLE) {
    x1 -= w / 2;
  }
  else if (x_align == RIGHT) {
    x1 -= w;
  }
  if (y_align == MIDDLE) {
    y1 += h / 2;
  }
  else if (y_align == TOP) {
    y1 += h;
  }
  if (clear_width >= 0) {
    if (clear_width == 0) {
      // flag for copy text.
      clear_width = w;
    }
    if (clear_height == 0) {
      clear_height = h;
    }
    if (x_align == RIGHT) {
      // If clear_width != w, make the right edges line up (not the left).
      // Need to stretch right edge to cover 11->12 transition.
      tft.fillRect(x - clear_width, y1 - h + 1, clear_width + 4, clear_height, bgcolor);
    } else {
      tft.fillRect(x1 - 1, y1 - h + 1, clear_width + 2, clear_height, bgcolor);
    }
  }
  // We're going to assume that h==8 -> using the default tiny font which wants
  // baseline at top.
  if (h == 8) {
    y1 -= (h - 1);
  }
  tft.setCursor(x1, y1);
  tft.print(s);
}

//int backlight_val = 32;

void setup_display(void) {

  // turn on backlite
  //pinMode(TFT_BACKLITE, OUTPUT);
  //analogWrite(TFT_BACKLITE, backlight_val);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(bgcolor);


  tft.setFont(CalBlk36);
  //tft.setTextSize(2);
  int16_t  x1, y1;
  uint16_t w, h;
  tft.getTextBounds(":", (int16_t)0, (int16_t)0, &x1, &y1, &big_colon_width, &big_colon_height);
  big_colon_width += 1;
  big_colon_height += 1;
  tft.getTextBounds("00", (int16_t)0, (int16_t)0, &x1, &y1, &digits_width, &digits_height);
  // Need to clear a little further.
  digits_width += 1;
  digits_height += 1;

  tft.setTextColor(fgcolor);

  // Serial availability status.
  tft.setFont(MICROFONT);
  tft.setCursor(234, 0);
  tft.print("S");
  if (!serial_available) {
    tft.setCursor(234, 0);
    tft.print("X");
  }

  // Seconds progress bar frame.
  // Seconds bar dimensions
  const uint8_t base_y = seconds_midline_y + secs_height / 2;
  const uint8_t base_x = display_mid_x - secs_x_scale * 30;
  // Box around the second progress bar, 1 pixel separated.
  tft.drawRect(base_x - 2, base_y - 2, 60 * secs_x_scale + 4, secs_height + 4, fgcolor);
}

void show_ntp_update(struct tm *timeinfo)
{
  // Update display.
  char msg[40];
  char *s = msg;
  strcpy(s, " Last update: ");
  s += strlen(s);
  sprint_tm(timeinfo, s);
  s += strlen(s);
  *s++ = ' ';
  *s++ = '\0';
  tft.setFont(MICROFONT);
  print_text(msg, display_mid_x, ntpstat_mid_y);
}

int8_t last_day = 0, last_hour = -1, last_minute = -1;
uint8_t colon_visible = true;

int update_display(time_t t) {
  // Returns minutes within day (0..1440).
  //u8g2.clearBuffer();   // for _F_ initializer only
  //tft.fillScreen(bgcolor);
  int mid_x = display_mid_x;

  struct tm * timeinfo;
  timeinfo = localtime (&t);
  serial_print_tm(timeinfo);
  int mins_within_day = 60 * timeinfo->tm_hour + timeinfo->tm_min;
  if (timeinfo->tm_mday != last_day) {
    last_day = timeinfo->tm_mday;
    char datestr[24];
    strcpy(datestr, datefmt);
    sprint_date(timeinfo, datestr, true);
    // Date.
    tft.setFont(SMALLFONT);
    //int16_t  x1, y1;
    //uint16_t w, h;
    //tft.getTextBounds(datestr, (int16_t)0, (int16_t)0, &x1, &y1, &w, &h);
    ////tft.drawRect(mid_x - w/2 - 8, date_midline_y - h/2 - 1, w + 16, h + 2, fgcolor);
    //print_text(datestr, mid_x - w/2, date_midline_y - h/2, LEFT, TOP, w + 16, h, false);
    print_text(datestr, mid_x, date_midline_y);
  }

  colon_visible = !colon_visible;

  // Big digits layout
  const int time_y = time_midline_y;

  // Seconds bar dimensions
  const uint8_t base_y = seconds_midline_y + secs_height / 2;
  const uint8_t base_x = mid_x - secs_x_scale * 30;
  // Bar width goes from 1 to 60 (instead of 0 to 59), so we have to work with one second ago.
  int prev_minute = timeinfo->tm_min;
  int prev_second = timeinfo->tm_sec - 1;
  if (prev_second < 0) {
    prev_minute = (prev_minute + 59) % 60;
    prev_second += 60;
  }
  // Bar width goes from 1 to 60 (instead of 0 to 59)
  uint8_t bar_width = secs_x_scale * (1 + prev_second);
  if (prev_minute & 1) {
    // bar shrinking to right
    tft.fillRect(base_x, base_y, bar_width, secs_height, bgcolor);
  } else {
    // bar growing from left
    tft.fillRect(base_x, base_y, bar_width, secs_height, fgcolor);
  }

  // Large digits time.
  tft.setFont(CalBlk36);
  char digit_string[3];
  //mid_x -= big_colon_width;
  if (timeinfo->tm_hour != last_hour) {
    last_hour = timeinfo->tm_hour;
    sprint_int2(digit_string, timeinfo->tm_hour);
    digit_string[2] = '\0';
    print_text(digit_string, mid_x - (big_colon_width / 2) - 3, time_midline_y, 
               RIGHT, MIDDLE, digits_width, digits_height);
  }
  if (timeinfo->tm_min != last_minute) {
    last_minute = timeinfo->tm_min;
    sprint_int2(digit_string, timeinfo->tm_min);
    digit_string[2] = '\0';
    print_text(digit_string, mid_x + (big_colon_width / 2) + 2, time_midline_y, 
               LEFT, MIDDLE, digits_width, digits_height);
  }
  if (colon_visible) {
    print_text(":", mid_x, time_midline_y);
  } else {
    tft.fillRect(mid_x - 1, time_midline_y - (big_colon_height >> 1),
                 big_colon_width, big_colon_height, bgcolor);
  }

  // Maybe show ntp update time.
  if (ntp_adjusted) {
    // ntp_adjusted is a semaphor set by ntp update callback.
    ntp_adjusted = false;
    show_ntp_update(timeinfo);
  }

  return mins_within_day;
}

// -------------------------------------------------------------------
// Input commands over serial line
// -------------------------------------------------------------------

void cmd_setup(void) {
  // Nothing to do?
}

byte atoi2(char *s) {
  // Convert two ascii digits to a uint8.
  return (s[1] - '0') + 10 * (s[0] - '0');
}

uint32_t htoi(char *s) {
  // Convert hex string to long int.
  uint32_t val = 0;
  while (*s) {
    uint8_t v = (*s) - '0';
    if (v > 9) v -= 'A' - '0' + 10;
    if (v > 15) v -= 'a' - 'A';
    ++s;
    val = (val << 4) + v;
  }
  return val;
}

int parse_string_to_mins(char *mins_string) {
  // Convert a string like "2359" into minutes since midnight (1439 in this case)
  if (strlen(mins_string) != 4) {
    if (serial_available) Serial.println("Error: cmd arg string is not 4 chrs.");
    return -1;
  }
  int hours = atoi2(mins_string);
  int minutes = atoi2(mins_string + 2);
  return 60 * hours + minutes;
}

struct tm parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  struct tm timeinfo;
  if (time_string[0] != '2' or time_string[1] != '0') {
    if (serial_available) Serial.println("Warn: Year does not start with 20...");
  }
  // tm_year: The number of years since 1900.
  timeinfo.tm_year = atoi2(time_string + 2) + 100;  // Interpret year as 20xx.
  timeinfo.tm_mon = atoi2(time_string + 4) - 1;
  timeinfo.tm_mday = atoi2(time_string + 6);
  timeinfo.tm_hour = atoi2(time_string + 8);
  timeinfo.tm_min = atoi2(time_string + 10);
  timeinfo.tm_sec = atoi2(time_string + 12);
  return timeinfo;
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(void) {
  int value;
  if (serial_available && Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n' || new_char == '\r') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            Serial.println("Z command");
            if (strlen(cmd_buffer) < 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              //RTC_set_time(parse_time_string(cmd_buffer + 1));
              time_t t = now_local();
              struct tm * timeinfo;
              timeinfo = localtime (&t);
              serial_print_tm(timeinfo);
            }
            break;
          case 'B':
            // Set backlight color to RRGGBB in hex.
            uint32_t brightness = htoi(cmd_buffer + 1);
            //set_backlight_color(brightness);
            analogWrite(backlightPin, brightness);
            Serial.print("new color=");
            Serial.println(brightness);
            break;
        }
      }
      cmd_len = 0;
    } else {
      if (cmd_len < CMD_BUF_LEN) {
        cmd_buffer[cmd_len++] = new_char;
      }
    }
  }
}

// ---------------------------------
// ---------- Battery monitor ----------
// ---------------------------------
#include "Adafruit_LC709203F.h"

Adafruit_LC709203F lc;
bool have_batmon = false;

void init_batmon() {
  if (!lc.begin()) {
    if (serial_available)  Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
  } else {
    have_batmon = true;
    if (serial_available) {
      Serial.println(F("Found LC709203F"));
      Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);
    }
    lc.setThermistorB(3950);
    if (serial_available) {
      Serial.print("Thermistor B = "); 
      Serial.println(lc.getThermistorB());
    }
    lc.setPackSize(LC709203F_APA_500MAH);
    lc.setAlarmVoltage(3.8);
  }
}

void draw_batmon_stat() {
  //if (serial_available) Serial.print("draw_batmon_stat");
  if (have_batmon) {
     char msg[32];
     char *s = msg;
     *s++ = ' ';
     strcpy(s, "Batt: ");
     s += strlen(s);
     s = sprint_int(s, int(round(100.0*lc.cellVoltage())), 2);
     *s++ = 'v';
     *s++ = ' ';
     s = sprint_int(s, int(round(lc.cellPercent())));
     *s++ = '%';
     *s++ = ' ';
     s = sprint_int(s, int(round(10*lc.getCellTemperature())), 1);
     *s++ = 'C';
     *s++ = ' ';
     *s++ = '\0';
     tft.setFont(MICROFONT);
     print_text(msg, display_mid_x, batstat_mid_y, MIDDLE, MIDDLE, 0, 0, false);
     //if (serial_available) Serial.println(msg);
  }
}

// ---------------------------------
// Backlight
// ---------------------------------

// Config for backlight day/night mode.
const int light_low = 8;
const int light_high = 64;
const int hour_up = 7;
const int hour_down = 22;

int brightness = 0;

void setup_backlight(void) {
  // Backlight
#ifdef BACKLIGHT
  pinMode(backlightPin, OUTPUT);  // sets the pin as output
  analogWrite(backlightPin, brightness);
#endif
}

int bright_tick = 0;
const int ticks_per_step = 1; // 1ms delay on polling, was 4096;

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

void update_backlight(int hour) {
  int target_brightness = light_low;
  if (hour >= hour_up && hour < hour_down) {
    target_brightness = light_high;
  }
  if (++bright_tick >= ticks_per_step) {
    // Slow down the brightness change steps.
    bright_tick = 0;
    int bright_delta = target_brightness - brightness;
    if (bright_delta) {
      brightness += (bright_delta >> 6) + sgn(bright_delta);
#ifdef BACKLIGHT
      analogWrite(backlightPin, brightness);
      if (serial_available) {
        //Serial.print("brightness=");
        //Serial.println(brightness);
      }
#endif // BACKLIGHT
    }
  }
}

// ---------------------------------
// Main
// ---------------------------------

void setup()
{
  Serial.begin(9600);
  // Wait for Serial port to open
  int i = 0;
#define MAXWAIT 1000
  while (!Serial) {
    delay(10);
    ++i;
    if (i > MAXWAIT) break;
  }
  if (i <= MAXWAIT /* Serial */) {
    serial_available = true;
  }
  //delay(500);

  if (serial_available) {
    Serial.print(F("gfx_clock_ntp "));
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);
  }

  Wire.begin();

  setup_RTC();
  setup_display();
  init_batmon();
  setup_backlight();
  cmd_setup();
}

time_t current_now = 0;  // 60 * (23 * 60 + 50);
int mins_within_day = 0;

void loop()
{
  if (update_RTC()) {
    current_now = now_local();
    mins_within_day = update_display(current_now);
    draw_batmon_stat();
  }
  delay(50);
  cmd_update();
  update_backlight(mins_within_day / 60);
}
