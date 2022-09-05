/*
 *  gfx_clock_logger based on u8g2_clock_logger.
 *  This version includes datalogging
 *  
 * Wiring:
 *   Designed to run on Adafruit ESP32-S3 TFT with built-in ST7789
 *   DS3231 StemmaQT on built-in I2C
 * 
 */

// Enable serial monitor?
// When enabled, boot will hang if we *don't* have a computer attached (i.e., just USB power)
//#define USE_SERIAL

// Which temperature sensor?
//#define TEMP_SHT4x
#define TEMP_DS3231

// Do we use the backlight?
#define BACKLIGHT

const int LOG_DATA_LEN = 120;     // One value per pixel, roughly.
const int LOG_INTERVAL_MINS = 12;  // Minutes between each logged value. 12 min x 120 vals = 1440 mins (24 h).
const int LOG_MAX_TIME_PERIOD = 1440;  // Make sure we roll-over correctly.
const int SUBDIV_MINS = (30 * LOG_INTERVAL_MINS);  // Where the vertical lines occur

//#include <Arduino.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

// Pin 2 is I2C CLK on FEATHER_M4, so use 4 to catch the DS3231 interrupts. (NOTUSED)
const int sqwPin = A1; // The number of the pin for monitor alarm status on DS3231

const int backlightPin = TFT_BACKLITE;  // PWM output to drive dimmable backlight (NOTUSED)

volatile unsigned long rtc_micros = 0;
volatile bool pending_RTC_interrupt = false;

void rtc_mark_isr(void)
{
  rtc_micros = micros();
  pending_RTC_interrupt = true;
}

void setup_interrupts() {
  pinMode(sqwPin, INPUT_PULLUP); // Set alarm pin as pullup
  // DS3231 is on falling edge.
  // attachInterrupt doesn't work for pin 13.
  attachInterrupt(digitalPinToInterrupt(sqwPin), rtc_mark_isr, FALLING);
} 


// ======================================================
// Track RTC times.
// ======================================================
#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

//#include "RTClib.h"
//RTC_DS3231 ds3231;
#include <DS3231.h>
#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
DS3231 ds3231;

RTClib RTC;

time_t last_time = 0;

time_t RTC_utc_get(void) {
  return RTC.now().unixtime();
}

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(now());
}

void RTC_set_time(const tmElements_t& tm) {
  // Set the DS3231 time.
  Serial.print("Set RTC: ");
  serial_print_tm(tm);
  ds3231.setYear(tmYearToY2k(tm.Year));
  ds3231.setMonth(tm.Month);
  ds3231.setDate(tm.Day);
  ds3231.setHour(tm.Hour);
  ds3231.setMinute(tm.Minute);
  ds3231.setSecond(tm.Second);
  // Resync TimeLib
  //setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
  setTime(RTC_utc_get());
}

void setup_RTC(void) {
  bool OscOn = true;
  bool OnBattery = false;
  byte frequency = 0;  // 0 == 1 Hz
  ds3231.enableOscillator(OscOn, OnBattery, frequency);
  bool h12 = false; // 24h mode
  ds3231.setClockMode(h12);

  setSyncProvider(RTC_utc_get);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet) {
#ifdef USE_SERIAL
    Serial.println("Unable to sync with the RTC");
    Serial.flush();
#endif
    abort();
  } else {
#ifdef USE_SERIAL
     Serial.println("RTC has set the system time");
#endif
  }
#ifdef USE_SERIAL
  Serial.println(now_local());
  Serial.println("RTC OK");
#endif
  last_time = now_local();
}

bool update_RTC(void) {
  // Returns true each time an RTC interrupt is cleared, i.e. 1/sec.
#ifdef INTERRUPT_DRIVEN
  bool handled_interrupt = false;
  if (pending_RTC_interrupt) {
    pending_RTC_interrupt = false;
    handled_interrupt = true;
  }
  return handled_interrupt;
#else // Time-driven
  time_t now_time = now_local();
  if (last_time == now_time) {
    return false;
  }
  last_time = now_time;
  return true;
#endif
}

// --------------------------
// Temp sensor
// --------------------------

#ifdef TEMP_SHT4x
#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

void setup_temp_F(void) {
#ifdef TEMP_SHT4x
  if (! sht4.begin()) {
#ifdef USE_SERIAL
    Serial.println("Couldn't find SHT4x");
#endif
    while (1) delay(1);
  }
#ifdef USE_SERIAL
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);
#endif // USE_SERIAL
#endif // TEMP_SHT4x
}

#ifdef TEMP_SHT4x
int read_temp_F(void) {
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
#ifdef USE_SERIAL
  //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  //Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
#endif
  return (int)round(temp.temperature * 1.8 + 32.0);
}
#endif

#ifdef TEMP_DS3231
int read_temp_F(void) {
  return (int)round(ds3231.getTemperature() * 1.8 + 32.0);
}
#endif


// --------------------------
// Time formatting
// --------------------------

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void serial_print_tm(const tmElements_t &tm)
{
#ifdef USE_SERIAL
  Serial.print(1970 + tm.Year);
  Serial.print("-");
  printDigits(tm.Month);
  Serial.print("-");
  printDigits(tm.Day);
  Serial.print(" ");
  printDigits(tm.Hour);
  Serial.print(":");
  printDigits(tm.Minute);
  Serial.print(":");
  printDigits(tm.Second);
  Serial.println();
#endif
}

// -------------------
// Logger
// -------------------

uint8_t log_data[LOG_DATA_LEN];
int log_times[LOG_DATA_LEN];
int data_min = 70;
int data_max = 75;

#define INVALID_TIME (-1)

void init_data(int init_val, int init_time) {
  for (int i = 0; i < LOG_DATA_LEN; ++i) {
    log_data[i] = init_val;
    log_times[i] = init_time;
  }
  set_min_max(log_data, log_times, LOG_DATA_LEN, &data_min, &data_max);
}

void set_min_max(uint8_t *log_data, int *data_valid, int log_data_len, int *pdata_min, int *pdata_max) {
  int data_min = 0;
  int data_max = 0;
  bool seen_valid_data = false;
  for (int i = 0; i < log_data_len; ++i) {
    if (data_valid[i] != INVALID_TIME) {
      int new_data = log_data[i];
      if (seen_valid_data) {
        data_min = min(data_min, new_data);
        data_max = max(data_max, new_data);
      } else {
        data_min = new_data;
        data_max = new_data;
        seen_valid_data = true;
      }
    }
  }
  if (seen_valid_data) {
    // Ensure there's a nonzero range in the data.
    *pdata_min = min(data_min, data_max - 1);
    *pdata_max = max(data_max, data_min + 1);
  }
}

void update_most_recent_data(uint8_t new_data) {
  // Simply change the final data point (provided it's already been set).
  if (log_times[LOG_DATA_LEN - 1] != INVALID_TIME) {
    log_data[LOG_DATA_LEN - 1] = new_data;
  }
}

void push_data(uint8_t new_data, int new_time) {
  // Move forward data
#ifdef USE_SERIAL
  Serial.print("new_data: ");
  Serial.println(new_data);
#endif
  for (int i = 0; i < LOG_DATA_LEN - 1; ++i) {
    log_data[i] = log_data[i + 1];
    log_times[i] = log_times[i + 1];
  }
  log_data[LOG_DATA_LEN - 1] = new_data;
  log_times[LOG_DATA_LEN - 1] = new_time;
  // Reset the min/max limits based on current data.
  set_min_max(log_data, log_times, LOG_DATA_LEN, &data_min, &data_max);
}

int last_log_time = 0;

void update_logger(int data_val, int data_time) {
  if ( (data_time - last_log_time + LOG_MAX_TIME_PERIOD) % LOG_MAX_TIME_PERIOD >= LOG_INTERVAL_MINS) {
    // Time to log a new value.
    last_log_time = data_time;
    // Record new temperature every minute.
    push_data(data_val, data_time);
  } else {
    // Update the most recent data so that logger shows current temp.
    update_most_recent_data(data_val);
  }
}

void setup_logger(void) {
  // Called during power up.  Clear logger to current time, value.
  init_data(0, INVALID_TIME);
}

//------------------------
// Display
//------------------------

// Layout
const uint16_t overall_top_y = 0;
const uint16_t date_midline_y = overall_top_y - 0;
const uint16_t time_midline_y = overall_top_y + 52;
const uint16_t seconds_midline_y = overall_top_y + 80;
const uint16_t log_top_y = overall_top_y + 102;
const uint16_t log_width = 128;
const uint16_t log_height = 32;
const uint16_t display_mid_x = 120;
const uint8_t secs_x_scale = 3;
const uint8_t secs_height = 8;

#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#define SMALLFONT &FreeSans12pt7b
#define MICROFONT
#define CalBlk36 &FreeSansBold24pt7b

uint16_t big_colon_width = 0;
uint16_t big_colon_height = 0;
uint16_t digits_width = 0;
uint16_t digits_height = 0;
uint16_t date_width = 0;
const char *datefmt = "DDD YYYY-MM-DD";

// 565 RGB 16-bit colors
int fgcolor = 0; // ST77XX_WHITE;
int bgcolor = 0x47E6; // Bright blueish-green // ST77XX_BLACK;

enum text_alignment {
  TOP,
  MIDDLE,
  BOTTOM,
  LEFT,
  RIGHT,
};

void print_text(char *s, int16_t x, int16_t y, int8_t x_align = MIDDLE, int8_t y_align = MIDDLE,
                int16_t clear_width=0, int16_t clear_height=0) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(s, x, y, &x1, &y1, &w, &h);
  y1 = y;
  x1 = x;
  if (x_align == MIDDLE) {x1 -= w / 2;}
  else if (x_align == RIGHT) {x1 -= w;}
  if (y_align == MIDDLE) {y1 += h / 2;}
  else if (y_align == TOP) {y1 += h;}
  if (clear_width) {
    if (x_align == RIGHT) {
      // If clear_width != w, make the right edges line up (not the left).
      tft.fillRect(x1 + 1 + w - clear_width, y1 - h + 1, clear_width, clear_height, bgcolor);
    } else {
      tft.fillRect(x1 + 1, y1 - h + 1, clear_width, clear_height, bgcolor);
    }
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

#ifdef USE_SERIAL
  Serial.println(F("Initialized"));
#endif

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
  
  // Seconds progress bar frame.
  // Seconds bar dimensions
  const uint8_t base_y = seconds_midline_y + secs_height / 2;
  const uint8_t base_x = display_mid_x - secs_x_scale * 30;
  // Box around the second progress bar, 1 pixel separated.
  tft.drawRect(base_x - 2, base_y - 2, 60 * secs_x_scale + 4, secs_height + 4, fgcolor);
}

char *sprint_int2(char *s, uint8_t n)
{  // Always 2 digits, assume n nonnegative, < 99.
  *s++ = '0' + (n / 10);
  *s++ = '0' + (n % 10);
  return s;
}

char *sprint_int(char *s, int n, int decimal_place=0)
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

#define SMALL_TEXT_H  (8)
#define SMALL_TEXT_W  (6)

int y_offset_per_data(int data_y, int mid_y) {
    if (data_y >= mid_y) {
    return -(SMALL_TEXT_H + 1);  // Line is in lower half; label above final point.
  } else {
    return 1;   // Line is in upper half; label below final point.
  }
}

void draw_time(int x, int y, int day_mins, bool show_minutes=true) {
  // Plot a time as HH:MM (or just HH if not show_minutes) using tiny font.
  char str[3];
  *(sprint_int2(str, day_mins / 60)) = '\0';
  tft.setCursor(x, y);
  tft.print(str);
  if (show_minutes) {
    *(sprint_int2(str, day_mins % 60)) = '\0';
    tft.setCursor(x + 2 * SMALL_TEXT_W + 2, y);
    tft.print(str);   // Over by 2 chr + 2 px for colon.
    // Add colon.
    tft.fillRect(x + 2 * SMALL_TEXT_W, y + 2, 1, 1, fgcolor);
    tft.fillRect(x + 2 * SMALL_TEXT_W, y + 4, 1, 1, fgcolor);
  }
}

void draw_log_output(int x, int y, int w, int h) {
  // Clear canvas
  tft.fillRect(x, y, w, h, bgcolor);
  //tft.setTextSize(1);
  // Figure scaling
  tft.setFont(MICROFONT);
  const int legend_w = 3 * SMALL_TEXT_W; // for legends up to 3 digits.
  uint8_t data_x = x + legend_w;
  uint8_t data_w = w - legend_w;
  int data_scale = (h - 1) * 256 / (data_max - data_min);
  uint8_t last_x;
  bool last_x_valid = false;
  uint8_t last_y = y;
  uint8_t last_data = 0;
  uint8_t first_x = 0;
  uint8_t first_y = 0;
  int first_time = INVALID_TIME;
  int8_t last_quarter_day = -1;
  for (int i = 0; i < LOG_DATA_LEN; ++i) {
    last_data = log_data[i];
    if (log_times[i] != INVALID_TIME) {
      int new_y = y + (h - 1) - ((data_scale * (last_data - data_min)) >> 8);
      int new_x = data_x + (i * (data_w - 1) / (LOG_DATA_LEN - 1));
      if (last_x_valid) {
        tft.drawLine(last_x, last_y, new_x, new_y, fgcolor);
      }
      // Add vertical lines every 6h transition.
      int8_t quarter_day = log_times[i] / SUBDIV_MINS;
      if (quarter_day != last_quarter_day) {
        if (last_quarter_day >= 0) {
          tft.drawLine(new_x, y, new_x, y + h, fgcolor);
          // Label it with 2 digits to the left.
          int vert_line_legend_x = new_x - (2*SMALL_TEXT_W);
          // Don't draw if it's going to splay off the left.
          if (vert_line_legend_x >= data_x) {
            draw_time(vert_line_legend_x, y - 1, quarter_day * SUBDIV_MINS, /* show_minutes= */ false);
          }
        }
        last_quarter_day = quarter_day;
      }
      last_x = new_x;
      last_y = new_y;
      last_x_valid = true;
      if (first_time == INVALID_TIME) {
        first_time = log_times[i];
        first_x = last_x;
        first_y = last_y;  // Needed to position earliest timestamp below.
      }
    }
  }
  // Text labels.
  char legend_str[6];
  int mid_y = (y + h / 2);
  // Do we have some actual data?
  if (last_x_valid) {
    // Add time of earliest point.
    // Bias time to be below if it's at the mid point so that it might miss the current value.
    int y_offset = y_offset_per_data(first_y - 1, mid_y);
    // Only draw first_time if it doesn't impinge on legend at left.
    int first_time_x = first_x - (4*SMALL_TEXT_W + 1);
    if (first_time_x >= data_x) {
      draw_time(first_time_x, first_y + y_offset, first_time);
    }
    // Label current value.
    y_offset = y_offset_per_data(last_y, mid_y);
    *(sprint_int(legend_str, last_data)) = '\0';
    tft.setCursor(last_x - SMALL_TEXT_W * strlen(legend_str) + 2, last_y + y_offset);
    tft.print(legend_str);
  }
  // Add legend at left.
  *(sprint_int(legend_str, data_max)) = '\0';
  tft.setCursor(x, y);
  tft.print(legend_str);
  *(sprint_int(legend_str, data_min)) = '\0';
  tft.setCursor(x, y + h - SMALL_TEXT_H);
  tft.print(legend_str);  // Font is 6 pixels high.
  //tft.setTextSize(2);
}

int8_t last_day = 0, last_hour = -1, last_minute = -1;
uint8_t colon_visible = true;

char dow_names[] = "SunMonTueWedThuFriSat";

void sprint_date(class DateTime& dt, char* datestr) {
  // Fake dt.toString for our format.
  int dow = dt.dayOfTheWeek();
  char *s = datestr;
  for (int i = 0; i < 3; ++i) {
    *s++ = dow_names[3 * dow + i];
  }
  *s++ = ' ';
  s = sprint_int(s, dt.year());
  *s++ = '-';
  s = sprint_int2(s, dt.month());
  *s++ = '-';
  s = sprint_int2(s, dt.day()); 
  *s++ = '\0'; 
}

int update_display(time_t t) {
  // Returns minutes within day (0..1440).
  //u8g2.clearBuffer();   // for _F_ initializer only
  //tft.fillScreen(bgcolor);
  int mid_x = display_mid_x;
  
  tmElements_t tm;
  breakTime(t, tm);
  serial_print_tm(tm);
  int mins_within_day = 60 * tm.Hour + tm.Minute;
  if (tm.Day != last_day) {
    last_day = tm.Day;
    char datestr[16];
    strcpy(datestr, datefmt);
    DateTime dt(t);
    sprint_date(dt, datestr);
    // Date.
    tft.setFont(SMALLFONT);
    print_text(datestr, mid_x + 32, date_midline_y);
  }

  colon_visible = !colon_visible;

  // Big digits layout
  const int time_y = time_midline_y;

  // Seconds bar dimensions
  const uint8_t base_y = seconds_midline_y + secs_height / 2;
  const uint8_t base_x = mid_x - secs_x_scale * 30;
  // Bar width goes from 1 to 60 (instead of 0 to 59), so we have to work with one second ago.
  int prev_minute = tm.Minute;
  int prev_second = tm.Second - 1;
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

  // Logging plot setup
  const int log_x = mid_x - (log_width >> 1);
  const int log_y = log_top_y;
  const int log_w = log_width;
  const int log_h = log_height;

  // Large digits time.
  tft.setFont(CalBlk36);
  char digit_string[3];
  //mid_x -= big_colon_width;
  if (tm.Hour != last_hour) {
    last_hour = tm.Hour;
    sprint_int2(digit_string, tm.Hour);
    digit_string[2] = '\0';
    print_text(digit_string, mid_x - (big_colon_width/2) - 3, time_midline_y, RIGHT, MIDDLE, digits_width, digits_height);
  }
  if (tm.Minute != last_minute) {
    last_minute = tm.Minute;
    sprint_int2(digit_string, tm.Minute);
    digit_string[2] = '\0';
    print_text(digit_string, mid_x + (big_colon_width/2) + 2, time_midline_y, LEFT, MIDDLE, digits_width, digits_height);
    // Update log when minutes change.
    draw_log_output(log_x, log_y, log_w, log_h);
  }
  if (colon_visible) {
    print_text(":", mid_x, time_midline_y);
  } else {
    tft.fillRect(mid_x - 1, time_midline_y - (big_colon_height >> 1),
                 big_colon_width, big_colon_height, bgcolor);
  }

  return mins_within_day;
}

// -------------------------------------------------------------------
// Input commands over serial line

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
  while(*s) {
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
    Serial.println("Error: cmd arg string is not 4 chrs.");
    return -1;
  }
  int hours = atoi2(mins_string);
  int minutes = atoi2(mins_string + 2);
  return 60 * hours + minutes;
}

tmElements_t parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  tmElements_t tm;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  tm.Year = y2kYearToTm(atoi2(time_string + 2));
  tm.Month = atoi2(time_string + 4);
  tm.Day = atoi2(time_string + 6);
  tm.Hour = atoi2(time_string + 8);
  tm.Minute = atoi2(time_string + 10);
  tm.Second = atoi2(time_string + 12);
  return tm;
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

bool enable_rtc_updates = true;

void cmd_update(void) {
  int value;
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n' || new_char == '\r') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'A':
            // Set aging offset.
            value = atoi(cmd_buffer + 1);
            ds3231.setAgingOffset(value);
            Serial.print("Aging offset=");
            Serial.println((int8_t)ds3231.getAgingOffset());
            break;
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            Serial.println("Z command");
            if (strlen(cmd_buffer) < 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              RTC_set_time(parse_time_string(cmd_buffer + 1));
              tmElements_t tm;
              breakTime(now_local(), tm);
              serial_print_tm(tm);
            }
            break;
          case 'D':
            // Specify display request: D2359
            // Any invalid time string (eg. D-1) will stop the system trying to reach the time.
            //move_to_time(parse_string_to_mins(cmd_buffer + 1));
            break;
          case 'W':
            // Warp display state (to match actual display): W2359
            //flip_set_state_mins(parse_string_to_mins(cmd_buffer + 1));
            break;
          case 'T':
            // Enable (T1) / disable (T0) RTC time updates, or toggle: T
            enable_rtc_updates = (cmd_len==1) ? !enable_rtc_updates : (cmd_buffer[1] == '1');
            Serial.print("enable_rtc_updates: ");
            Serial.println(enable_rtc_updates);
            break;
          case 'S':
            // Execute some individual steps, bypassing the flip manager.
            // Typically, one minute is 170 or 171 steps (142 for Copal227).
            // We can use this to trim the stepper to be "mid cycle": 
            //  - slowly wind on by ~10 steps until flap falls
            //  - step on 85 (71) steps to be mid-cycle
            //  - Warp display to current reading, you are now good.
            //add_steps(atoi(cmd_buffer + 1));
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
// Backlight
// ---------------------------------

// Config for backlight day/night mode.
const int light_low = 32;
const int light_high = 255;
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
const int ticks_per_step = 4096;

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
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
#ifdef USE_SERIAL
      //Serial.print("brightness=");
      //Serial.println(brightness);
#endif // USE_SERIAL
#endif // BACKLIGHT
    }
  }
}

// ---------------------------------
// Main
// ---------------------------------

#define SECS_PER_DAY (24 * 60 * 60)

void setup()
{
#ifdef USE_SERIAL
  Serial.begin(9600);
  // Wait for Serial port to open
  while (!Serial) {
    delay(10);
  }
  //delay(500);

  Serial.print(F("gfx_clock_logger "));
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
#endif

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  Wire.begin();

  setup_RTC();
  setup_interrupts();
  setup_display();
  setup_backlight();
  setup_temp_F();
  setup_logger();
  cmd_setup();

}

time_t current_now = 0;

void loop()
{
  cmd_update();
  if (update_RTC()) {
    current_now = now_local();
    int mins_within_day = update_display(current_now);
    update_logger(read_temp_F(), mins_within_day);
  }
  update_backlight(hour(current_now));
}
