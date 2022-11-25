/*
 *  u8g2_clock based on glcd_clock.
 *  This version includes datalogging
 *  Currently just using the DS3231 temp sensor.
 *  
 * Wiring:
 * 
 *  FeatherRP2040 : 128x64 SH1107 display
 *    D13 (SCK)  : CLK 
 *    D11 (MOSI) : DAT
 *    D10        : CS
 *    D8 (PWM)   : LEDA (via 220 ohm resistor)
 *    GND        : LEDK
 *    GND        : PSB (to select serial i/f)
 *    5V         : VCC
 *    GND        : GND
 *    
 *  FeatherRP2040 : DS3231
 *    5V         : VCC
 *    GND        : GND
 *    A4         : CLK
 *    A5         : DAT
 *    D2 (INT)   : SQW
 */

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

const int sqwPin = 2; // The number of the pin for monitor alarm status on DS3231

const int backlightPin = 5;  // PWM output to drive dimmable backlight.

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

#include "RTClib.h"
RTC_DS3231 rtc;

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(rtc.now().unixtime());
}

void setup_RTC(void) {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz); // Place SQW pin into 1 Hz mode
  Serial.println(now_local());
  Serial.println("RTC OK");
}

bool update_RTC(void) {
  // Returns true each time an RTC interrupt is cleared, i.e. 1/sec.
  bool handled_interrupt = false;
  if (pending_RTC_interrupt) {
    pending_RTC_interrupt = false;
    handled_interrupt = true;
  }
  return handled_interrupt;
}

int rtc_temp_F(void) {
  return (int)round(rtc.getTemperature() * 1.8 + 32.0);
}

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
}

// -------------------
// Logger
// -------------------

const int log_data_len = 120;
uint8_t log_data[log_data_len];
int log_times[log_data_len];
int data_min = 70;
int data_max = 75;

#define INVALID_TIME (-1)

void init_data(int init_val, int init_time) {
  for (int i = 0; i < log_data_len; ++i) {
    log_data[i] = init_val;
    log_times[i] = init_time;
  }
  set_min_max(log_data, log_times, log_data_len, &data_min, &data_max);
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
  // Ensure there's a nonzero range in the data.
  *pdata_min = min(data_min, data_max - 1);
  *pdata_max = max(data_max, data_min + 1);
}

void push_data(uint8_t new_data, int new_time) {
  // Move forward data
  Serial.print("new_data: ");
  Serial.println(new_data);
  for (int i = 0; i < log_data_len - 1; ++i) {
    log_data[i] = log_data[i + 1];
    log_times[i] = log_times[i + 1];
  }
  log_data[log_data_len - 1] = new_data;
  log_times[log_data_len - 1] = new_time;
  // Reset the min/max limits based on current data.
  set_min_max(log_data, log_times, log_data_len, &data_min, &data_max);
}

int last_log_time = 0;
const int log_interval = 12;  // Minutes between each logged value. 12 min x 120 vals = 1440 mins (24 h).

void update_logger(int data_val, int data_time) {
  if (data_time >= last_log_time + log_interval) {
    // Time to log a new value.
    last_log_time = data_time;
    // Record new temperature every minute.
    push_data(data_val, data_time);
  }
}

void setup_logger(void) {
  // Called during power up.  Clear logger to current time, value.
  init_data(0, INVALID_TIME);
}

//------------------------
// Display
//------------------------

//#define YWROBOT_12864
#ifdef YWROBOT_12864
#pragma message("Using YwRobot")
// YwRobot 128x64
//U8G2_UC1701_MINI12864_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_ST7565_ERC12864_ALT_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);  // contrast improved version for ERC12864
#else
// SPI Green LCD panel
//U8G2_ST7920_128X64_2_HW_SPI u8g2(U8G2_R0, /* CS=*/ 10, /* reset=*/ 8); // Feather HUZZAH ESP8266, E=clock=14, RW=data=13, RS=CS
#endif
// RP2040 feather
//#define SPI SPI0
#define Wire Wire1
U8G2_SH1107_64X128_2_SW_I2C u8g2 (U8G2_R0, /* SCL=*/ 3, /* SDA=*/ 2, U8X8_PIN_NONE);

#define SMALLFONT u8g2_font_profont11_tr
//#define SMALLFONT System5x7
#define BIGFONT u8g2_font_logisoso24_tf
//#define BIGFONT CalBlk36

uint8_t big_colon_width = 0;
uint8_t date_width = 0;
const char *datefmt = "DDD YYYY-MM-DD";

void setup_display(void) {
  u8g2.begin();
#ifdef YWROBOT_12864
  u8g2.setContrast(0);  
#endif
  //u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  // Precalculate the width of the average big digit.
  u8g2.setFont(u8g2_font_logisoso34_tn);
  big_colon_width = u8g2.getStrWidth(":");
  u8g2.setFont(SMALLFONT);
  date_width = u8g2.getStrWidth(datefmt);
  u8g2.setFont(BIGFONT); // So first call to u8g2.getStrWidth is using the right font
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

#define SMALL_TEXT_H  (6)
#define SMALL_TEXT_W  (4)

int y_offset_per_data(int data_y, int mid_y) {
    if (data_y >= mid_y) {
    return -(SMALL_TEXT_H + 1);  // Line is in lower half; label above final point.
  } else {
    return 1;   // Line is in upper half; label below final point.
  }
}

void draw_time(int x, int y, int day_mins) {
  // Plot a time as HH:MM using tiny font.
  char str[3];
  *(sprint_int2(str, day_mins / 60)) = '\0';
  u8g2.drawStr(x, y, str);  
  *(sprint_int2(str, day_mins % 60)) = '\0';
  u8g2.drawStr(x + 2 * SMALL_TEXT_W + 2, y, str);   // Over by 2 chr + 2 px for colon.
  // Add colon.
  u8g2.drawPixel(x + 2 * SMALL_TEXT_W, y + 2);
  u8g2.drawPixel(x + 2 * SMALL_TEXT_W, y + 4);  
}

void draw_log_output(int x, int y, int w, int h) {
  // Figure scaling
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
  for (int i = 0; i < log_data_len; ++i) {
    last_data = log_data[i];
    if (log_times[i] != INVALID_TIME) {
      int new_y = y + (h - 1) - ((data_scale * (last_data - data_min)) >> 8);
      int new_x = data_x + (i * (data_w - 1) / (log_data_len - 1));
      if (last_x_valid) {
        u8g2.drawLine(last_x, last_y, new_x, new_y);
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
  u8g2.setFont(u8g2_font_micro_tn);
  char legend_str[6];
  int mid_y = (y + h / 2);
  // Do we have some actual data?
  if (last_x_valid) {
    // Label current value
    int y_offset = y_offset_per_data(last_y, mid_y);
    *(sprint_int(legend_str, last_data)) = '\0';
    u8g2.drawStr(last_x - SMALL_TEXT_W * strlen(legend_str) + 2, last_y + y_offset, legend_str);
    // Add time of earliest point.
    // Bias time to be below if it's at the mid point so that it might miss the current value.
    y_offset = y_offset_per_data(first_y - 1, mid_y);
    // Make sure time stays on-screen.
    draw_time(min(first_x, x + w - (4*SMALL_TEXT_W + 1)), first_y + y_offset, first_time);  
  }
  // Add legend at left.
  *(sprint_int(legend_str, data_max)) = '\0';
  u8g2.drawStr(x, y, legend_str);
  *(sprint_int(legend_str, data_min)) = '\0';
  u8g2.drawStr(x, y + h - SMALL_TEXT_H, legend_str);  // Font is 6 pixels high.
}

uint8_t last_day = 0;
uint8_t colon_visible = true;

int update_display(time_t t) {
  // Returns minutes within day (0..1440).
  //u8g2.clearBuffer();   // for _F_ initializer only

  tmElements_t tm;
  breakTime(t, tm);
  serial_print_tm(tm);
  int mins_within_day = 60 * tm.Hour + tm.Minute;
  last_day = tm.Day;
  char datestr[16];
  strcpy(datestr, datefmt);
  DateTime dt(t);
  dt.toString(datestr);
  const int mid_x = 64;
  int date_x = mid_x - (date_width >> 1);
  char hr_string[3], min_string[3];
  colon_visible = !colon_visible;
  sprint_int2(hr_string, tm.Hour);
  hr_string[2] = '\0';
  sprint_int2(min_string, tm.Minute);
  min_string[2] = '\0';

  // Big digits layout
  const int time_y = 9;
  int colon_x = mid_x - (big_colon_width >> 1) - 4;  // Manual fix
  int minute_x = colon_x + big_colon_width;
  int hour_x = colon_x - u8g2.getStrWidth(hr_string);
  // Manual adjustment
  colon_x += 2;

  // Seconds bar dimensions
  const uint8_t x_scale = 2;
  const uint8_t height = 4;
  const uint8_t base_y = 40;
  const uint8_t base_x = mid_x - x_scale * 30;
  // Bar width goes from 1 to 60 (instead of 0 to 59)
  uint8_t bar_left, bar_width;
  if ((tm.Minute & 1) == 0) {
    // bar growing from left
    bar_left = base_x;
    bar_width = tm.Second * x_scale;
  } else {
    // bar shrinking to right
    bar_left = base_x + tm.Second * x_scale;
    bar_width = (60 - tm.Second) * x_scale;
  }

  // Logging plot setup
  const int log_x = 0;
  const int log_y = base_y + 8;
  const int log_w = 128;
  const int log_h = 64 - log_y;

  u8g2.firstPage();
  do {
    // Logging first, since it changes the font.
    draw_log_output(log_x, log_y, log_w, log_h);
    // Date.
    u8g2.setFont(SMALLFONT);
    u8g2.drawStr(date_x, 0, datestr);
    // Large digits time.
    u8g2.setFont(BIGFONT);
    u8g2.drawStr(hour_x, time_y, hr_string);
    u8g2.drawStr(minute_x, time_y, min_string);
    if (colon_visible) {
      u8g2.drawStr(colon_x, time_y, ":");
    }
    // Seconds progress bar.
    // Box around the second progress bar, 1 pixel separated.
    u8g2.drawFrame(base_x - 2, base_y - 2, 60 * x_scale + 4, 8);
    // Draw/undraw transition occurs on second 1.  Second 0 is drawing the 60th tick on the line.
    u8g2.drawBox(bar_left, base_y, bar_width, height);
  } while (u8g2.nextPage());
  //u8g2.sendBuffer();  // for _F_ initializer only

  return mins_within_day;
}

// ---------------------------------
// Backlight
// ---------------------------------

// Config for backlight day/night mode.
const int light_low = 32;
const int light_high = 255;
const int hour_up = 7;
const int hour_down = 22;

void setup_backlight(void) {
  // Backlight
  pinMode(backlightPin, OUTPUT);  // sets the pin as output
  analogWrite(backlightPin, light_low);
}

int brightness = 0;
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
      analogWrite(backlightPin, brightness);
      Serial.print("brightness=");
      Serial.println(brightness);
    }
  }
}

// ---------------------------------
// Main
// ---------------------------------

#define SECS_PER_DAY (24 * 60 * 60)

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("u8g2_clock_logger");

  setup_RTC();
  setup_interrupts();
  setup_display();
  setup_backlight();
  setup_logger();
}

time_t current_now = 0;

void loop()
{
  if (update_RTC()) {
    current_now = now_local();
    int mins_within_day = update_display(current_now);
    update_logger(rtc_temp_F(), mins_within_day);
  }
  update_backlight(hour(current_now));
}
