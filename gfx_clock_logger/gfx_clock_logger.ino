/*
 *  gfx_clock_logger based on u8g2_clock_logger.
 *  This version includes datalogging
 *  
 * Wiring:
 *   Designed to run on Adafruit ESP32-S3 TFT with built-in ST7789
 *   DS3231 StemmaQT on built-in I2C
 * 
 */

#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

// Enable serial monitor?
// When enabled, boot will hang if we *don't* have a computer attached (i.e., just USB power)
#define USE_SERIAL
bool serial_available = false;
// Fake rapid cycling of time.
//#define DEBUG

// Which temperature sensor?
//#define TEMP_SHT4x
#define TEMP_DS3231

const int MINS_PER_DAY = 1440;
const int LOG_DATA_LEN = 120;     // One value per pixel, roughly.
const int LOG_INTERVAL_SECS = 12 * 60;  // Minutes between each logged value. 12 min x 120 vals = 1440 mins (24 h).
const int LOG_MAX_TIME_PERIOD = MINS_PER_DAY * 60;  // Make sure we roll-over correctly.
const int SUBDIV_MINS = (30 * LOG_INTERVAL_SECS);  // Where the vertical lines occur

//#include <Arduino.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>

#ifdef ARDUINO_ARCH_RP2040
  #ifdef PIN_NEOPIXEL  // i.e., this is a Feather RP2040
    #define FEATHER_RP2040
    #define FEATHER_OLED
    #warning FEATHER_RP2040
  #else
    #define MY_PICO_RP2040
    #define MY_PICO_RP2040_LCD  // Alternate RP2040 pinout for 3" LCD on SPI
    #warning PICO_RP2040
  #endif
  #ifdef MY_PICO_RP2040_LCD
    #define DISPLAY_ST7920  // 128x64 green-yellow LCD matrix
  #else  // FEATHER_RP2040
    #define DISPLAY_SH1107  // 128x(64,128) mono OLED in Feather stack
  #endif
  //const int int_sda_pin = 2;
  //const int int_scl_pin = 3;
  const uint8_t sqwPin = 29;  // (A3)
  const int ext_sda_pin = 24;
  const int ext_scl_pin = 25;
  #define INT_I2C Wire
  #define EXT_I2C Wire1
#else
  // ESP32-S3
  #define INT_I2C Wire
  #define EXT_I2C Wire1
  #define DISPLAY_ST7789  // Built-in display on ESP32-S3 TFT
  //#define DISPLAY_SSD1351  // Exernal 128x128 RGB TFT
  // ESP32-S3 TFT - Expect SQWV input on 2 (external).
  const uint8_t sqwPin = A3;
  const int ext_sda_pin = A4;
  const int ext_scl_pin = A5;
#endif

// Use dedicated hardware SPI pins
#ifdef DISPLAY_ST7789
  #warning DISPLAY_ST7789
  #include <Adafruit_ST7789.h>
  Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
  // Ue the backlight.
  #define BACKLIGHT
  const int backlightPin = TFT_BACKLITE;  // PWM output to drive dimmable backlight (NOTUSED)
  #define BGCOLOR 0x0847E6  // yellow-green
  #define FGCOLOR 0
  #define SLEEP_TIMEOUT_SECS 0  // no timeout for LCD.
#endif
#ifdef DISPLAY_SH1107
  #warning DISPLAY_SH1107
  #include <Adafruit_SH110X.h>
  #ifdef FEATHER_OLED
    const int display_address = 0x3C;
    #define SCREEN_HEIGHT 64
  #else  // standalone OLED
    const int display_address = 0x3D;
    #define SCREEN_HEIGHT 128
  #endif
  #define SCREEN_WIDTH 128
  Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, &INT_I2C);
  // SH1107 needs display.display() after drawing
  #define DISPLAY_DISPLAY_CMD
  #define BGCOLOR SH110X_BLACK
  #define FGCOLOR SH110X_WHITE
  #define SLEEP_TIMEOUT_SECS 300
#endif

// Predeclare display timeout val.
uint32_t display_sleep_timeout_secs = SLEEP_TIMEOUT_SECS;

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

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

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

//#include "RTClib.h"
//RTC_DS3231 ds3231;
#include <DS3231.h>
#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
DS3231 ds3231(EXT_I2C);

RTClib RTC;

time_t last_time = 0;

time_t RTC_utc_get(void) {
  return RTC.now().unixtime();
}

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  //return myTZ.toLocal(now());
  return myTZ.toLocal(ds3231_now());
}

void RTC_set_time(const tmElements_t& tm) {
  // Set the DS3231 time.
  if (serial_available) Serial.print("Set RTC: ");
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

char *itoa2(int num, char *s, int base=10) {
#define DTOA(d) ((d < 10) ? ('0' + d) : ('A' + d - 10))
  char *my_s = s;
  *my_s++ = DTOA(num / base);
  *my_s++ = DTOA(num % base);
  *my_s++ = '\0';
  return s;  // Is this the return?
}

char *format_tm(const tmElements_t &tm, char *s) {
  char *s_in = s;
  itoa(1970 + tm.Year, s, 10);
  s += strlen(s);
  *s++ = '-';
  itoa2(tm.Month, s);
  s += 2;
  *s++ = '-';
  itoa2(tm.Day, s);
  s += 2;
  *s++ = ' ';
  itoa2(tm.Hour, s);
  s += 2;
  *s++ = ':';
  itoa2(tm.Minute, s);
  s += 2;
  *s++ = ':';
  itoa2(tm.Second, s);
  return s_in;
}

char *format_time(time_t t, char *s) {
  tmElements_t tm;
  breakTime(t, tm);
  return format_tm(tm, s);
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
    if(serial_available) {
      Serial.println("Unable to sync with the RTC");
      Serial.flush();
    }
    abort();
  }
  if (serial_available) {
    Serial.println("RTC has set the system time");
    char s[22];
    Serial.println(format_time(now_local(), s));
    Serial.println("RTC OK");
  }
  last_time = now_local();
}

time_t ds3231_now(void) {
  // Unix time corresponding to current DS3231 state.
  // Assume called soon after a tick interrupt, so can assume values are stable.
  tmElements_t tm;
  tm.Second = ds3231.getSecond();
  tm.Minute = ds3231.getMinute();
  bool h12, pmtime, century;
  tm.Hour = ds3231.getHour(h12, pmtime);
  tm.Day = ds3231.getDate();
  tm.Month = ds3231.getMonth(century);
  tm.Year = ds3231.getYear() + (2000 - 1970);  // tm.Year is years since 1970 in 8 bits.
  return makeTime(tm);
}

#define INTERRUPT_DRIVEN

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
  if (serial_available) Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  if (serial_available) {
    Serial.println("Found SHT4x sensor");
    Serial.print("Serial number 0x");
    Serial.println(sht4.readSerial(), HEX);
  }
#endif // TEMP_SHT4x
}

#ifdef TEMP_SHT4x
int read_temp_F(void) {
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  if (serial_avaliable) {
    //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    //Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  }
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
  if (serial_available) {
    char s[22];
    Serial.println(format_tm(tm, s));
  }
}

// -------------------
// Logger
// -------------------

uint8_t log_data[LOG_DATA_LEN];
time_t log_times[LOG_DATA_LEN];
int data_min = 70;
int data_max = 75;

#define INVALID_TIME (-1)

void init_data(int init_val, time_t init_time) {
  for (int i = 0; i < LOG_DATA_LEN; ++i) {
    log_data[i] = init_val;
    log_times[i] = init_time;
  }
  set_min_max(log_data, log_times, LOG_DATA_LEN, &data_min, &data_max);
}

void set_min_max(uint8_t *log_data, time_t *data_valid, int log_data_len, int *pdata_min, int *pdata_max) {
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

void push_data(uint8_t new_data, time_t new_time) {
  // Move forward data
  if (serial_available) {
     Serial.print("new_data: ");
     Serial.println(new_data);
  }
  for (int i = 0; i < LOG_DATA_LEN - 1; ++i) {
    log_data[i] = log_data[i + 1];
    log_times[i] = log_times[i + 1];
  }
  log_data[LOG_DATA_LEN - 1] = new_data;
  log_times[LOG_DATA_LEN - 1] = new_time;
  // Reset the min/max limits based on current data.
  set_min_max(log_data, log_times, LOG_DATA_LEN, &data_min, &data_max);
}

time_t last_log_time = 0;

void update_logger(int data_val, time_t data_time) {
  if ( (data_time - last_log_time + LOG_MAX_TIME_PERIOD) % LOG_MAX_TIME_PERIOD >= LOG_INTERVAL_SECS) {
    // Time to log a new value.
    last_log_time = data_time;
    // Record new temperature every minute.
    push_data(data_val, data_time);
  } else {
    // Update the most recent data so that logger shows current temp.
    //update_most_recent_data(data_val);
    // Skip this because plotting outside the current min/max range leaves permanent cruft on GFX display.
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
#if SCREEN_WIDTH == 240
const int16_t overall_top_y = 0;
const int16_t date_midline_y = overall_top_y + 16;
const int16_t time_midline_y = overall_top_y + 52;
const int16_t seconds_midline_y = overall_top_y + 80;
const int16_t log_top_y = overall_top_y + 102;
const int16_t log_width = 192;
const int16_t log_height = 32;
const int16_t display_mid_x = 120;
const int8_t secs_x_scale = 3;
const int8_t secs_height = 8;
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/CalBlk36.h>
#define SMALLFONT &FreeSans12pt7b
#define SMALL_VSHIFT  (0)
#define MICROFONT
#define MICROFONT_H  (8)
#define MICROFONT_W  (6)
//#define CalBlk36 &FreeSansBold24pt7b
#define CalBlk36 &CalBlk3612pt7b
#endif
#if SCREEN_WIDTH == 128
const int16_t overall_top_y = 0;
const int16_t date_midline_y = overall_top_y + 3;
const int16_t time_midline_y = overall_top_y + 25;
const int16_t seconds_midline_y = overall_top_y + 36;
const int16_t log_top_y = overall_top_y + 43;
const int16_t log_width = 128;
const int16_t log_height = 21;
const int16_t display_mid_x = 64;
const int8_t secs_x_scale = 2;
const int8_t secs_height = 2;
#include <Fonts/CalBlk36.h>
#include <Fonts/TomThumb.h>
#define SMALLFONT 
#define SMALL_VSHIFT  (-6)
#define MICROFONT &TomThumb
#define MICROFONT_W (4)
#define MICROFONT_H (6)
#define CalBlk36 &CalBlk3612pt7b
#endif


uint16_t big_colon_width = 0;
uint16_t big_colon_height = 0;
uint16_t digits_width = 0;
uint16_t digits_height = 0;
uint16_t digits_baseline_shift = 0;
uint16_t date_width = 0;
const char *datefmt = "DDD YYYY-MM-DD";

// 565 RGB 16-bit colors
int fgcolor = FGCOLOR; // ST77XX_WHITE;
int bgcolor = BGCOLOR; // Bright blueish-green // ST77XX_BLACK;

enum text_alignment {
  TOP,
  MIDDLE,
  BOTTOM,
  LEFT,
  RIGHT,
};

void print_text(char *s, int16_t x, int16_t y, int8_t x_align = MIDDLE, int8_t y_align = MIDDLE,
                int16_t clear_width=0, int16_t clear_height=0, bool debug=false, int16_t font_hshift=0) {
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(s, x, y, &x1, &y1, &w, &h);
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
  if (x_align == MIDDLE) {x1 -= w / 2;}
  else if (x_align == RIGHT) {x1 -= w;}
  if (y_align == MIDDLE) {y1 += h / 2;}
  else if (y_align == TOP) {y1 += h;}
  if (clear_width) {
    if (x_align == RIGHT) {
      // If clear_width != w, make the right edges line up (not the left).
      // Need to stretch right edge to cover 11->12 transition.
      display.fillRect(x - clear_width, y1 - h + 1, clear_width + 4, clear_height, bgcolor);
      //display.drawRect(x - clear_width, y1 - h + 1, clear_width + 4, clear_height, fgcolor);
    } else {
      display.fillRect(x - 1, y1 - h + 1, clear_width + 2, clear_height, bgcolor);
      //display.drawRect(x - 1, y1 - h + 1, clear_width + 2, clear_height, fgcolor);
    }
  }
  display.setCursor(x1, y1 + font_hshift);
  display.print(s);
}

//int backlight_val = 32;

void setup_display(void) {

  // turn on backlite
  //pinMode(TFT_BACKLITE, OUTPUT);
  //analogWrite(TFT_BACKLITE, backlight_val);

  // initialize TFT
#ifdef DISPLAY_ST7789
  display.init(135, 240); // Init ST7789 240x135
#endif
#ifdef DISPLAY_SH1107
  display.begin(display_address, true);
  //display.display();  // Splashscreen
  //delay(1000);
  display.clearDisplay();
  display.display();
  Serial.println("SH1107 started");
#endif
  display.setRotation(1);
  display.fillScreen(bgcolor);

  display.setFont(CalBlk36);
  //display.setTextSize(2);
  int16_t  x1, y1;
  uint16_t w, h;
  display.getTextBounds(":", (int16_t)0, (int16_t)0, &x1, &y1, &big_colon_width, &big_colon_height);
  big_colon_width += 1;
  big_colon_height += 1;
  display.getTextBounds("00", (int16_t)0, (int16_t)0, &x1, &y1, &digits_width, &digits_height);
  // Need to clear a little further.
  digits_width += 1;
  digits_height += 1;
#if SCREEN_WIDTH == 128
  big_colon_height -= 11;
  digits_height -= 11;
  digits_baseline_shift = -10;
#endif

  display.setTextColor(fgcolor);

  display.setFont(MICROFONT);
  display.setCursor(SCREEN_WIDTH - MICROFONT_W, MICROFONT_H);
  display.print("S");
  if (serial_available) {
    Serial.println(F("Initialized"));
  } else {
    display.setCursor(SCREEN_WIDTH - MICROFONT_W, MICROFONT_H);
    display.print("#");    
  }

  // Seconds progress bar frame.
  // Seconds bar dimensions
  const uint8_t base_y = seconds_midline_y + secs_height / 2;
  const uint8_t base_x = display_mid_x - secs_x_scale * 30;
  // Box around the second progress bar, 1 pixel separated.
  display.drawRect(base_x - 2, base_y - 2, 60 * secs_x_scale + 4, secs_height + 4, fgcolor);
#ifdef DISPLAY_DISPLAY_CMD
  display.display();
#endif
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

int y_offset_per_data(int data_y, int mid_y) {
    if (data_y >= mid_y) {
    return -1;  // Line is in lower half; label above final point.
  } else {
    return 1 + MICROFONT_H;   // Line is in upper half; label below final point.
  }
}

void draw_time(int x, int y, int day_mins, bool show_minutes=true) {
  // Plot a time as HH:MM (or just HH if not show_minutes) using tiny font.
  // x, y is bottom-left
  char str[3];
  *(sprint_int2(str, day_mins / 60)) = '\0';
  display.setCursor(x, y);
  display.print(str);
  if (show_minutes) {
    *(sprint_int2(str, day_mins % 60)) = '\0';
    display.setCursor(x + 2 * MICROFONT_W + 2, y);
    display.print(str);   // Over by 2 chr + 2 px for colon.
    // Add colon.
    display.fillRect(x + 2 * MICROFONT_W, y - 3, 1, 1, fgcolor);
    display.fillRect(x + 2 * MICROFONT_W, y - 1, 1, 1, fgcolor);
  }
}

void draw_log_output(int x, int y, int w, int h) {
  // Clear canvas
  display.fillRect(x, y - 1, w, h + 2, bgcolor);
  //display.setTextSize(1);
  // Figure scaling
  display.setFont(MICROFONT);
  const int legend_w = 3 * MICROFONT_W; // for legends up to 3 digits.
  uint8_t data_x = x + legend_w;
  uint8_t data_w = w - legend_w;
  int data_scale = (h - 1) * 256 / (data_max - data_min);
  uint8_t last_x;
  bool last_x_valid = false;
  uint8_t last_y = y;
  uint8_t last_data = 0;
  uint8_t first_x = 0;
  uint8_t first_y = 0;
  time_t first_time_mins = INVALID_TIME;
  int8_t last_quarter_day = -1;
  for (int i = 0; i < LOG_DATA_LEN; ++i) {
    last_data = log_data[i];
    if (log_times[i] != INVALID_TIME) {
      int new_y = y + (h - 1) - ((data_scale * (last_data - data_min)) >> 8);
      int new_x = data_x + (i * (data_w - 1) / (LOG_DATA_LEN - 1));
      if (last_x_valid) {
        display.drawLine(last_x, last_y, new_x, new_y, fgcolor);
      }
      // Add vertical lines every 6h transition.
      int8_t quarter_day = ((log_times[i] / 60) / SUBDIV_MINS) % 128;
      if (quarter_day != last_quarter_day) {
        if (last_quarter_day >= 0) {
          display.drawLine(new_x, y, new_x, y + h, fgcolor);
          // Label it with 2 digits to the left.
          int vert_line_legend_x = new_x - (2 * MICROFONT_W);
          // Don't draw if it's going to splay off the left.
          if (vert_line_legend_x >= data_x) {
            draw_time(vert_line_legend_x, y - 1 + MICROFONT_H, quarter_day * SUBDIV_MINS, /* show_minutes= */ false);
          }
        }
        last_quarter_day = quarter_day;
      }
      last_x = new_x;
      last_y = new_y;
      last_x_valid = true;
      if (first_time_mins == INVALID_TIME) {
        first_time_mins = (log_times[i] / 60) % MINS_PER_DAY;
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
    int first_time_x = first_x - (4 * MICROFONT_W + 1);
    if (first_time_x >= data_x) {
      draw_time(first_time_x, first_y + y_offset, first_time_mins);
    }
    // Label current value.
    y_offset = y_offset_per_data(last_y, mid_y);
    *(sprint_int(legend_str, last_data)) = '\0';
    display.setCursor(last_x - MICROFONT_W * strlen(legend_str) + 2, last_y + y_offset);
    display.print(legend_str);
  }
  // Add legend at left.
  *(sprint_int(legend_str, data_max)) = '\0';
  display.setCursor(x, y + MICROFONT_H);
  display.print(legend_str);
  *(sprint_int(legend_str, data_min)) = '\0';
  display.setCursor(x, y + h);
  display.print(legend_str);  // Font is 6 pixels high.
  //display.setTextSize(2);
}

int8_t last_day = 0, last_hour = -1, last_minute = -1;
uint8_t colon_visible = true;

char dow_names[] = "SunMonTueWedThuFriSat";

void sprint_date(class DateTime& dt, char* datestr) {
  // Fake dt.toString for our format.  Pad each end with spaces to get proper clearing of previous.
  int dow = (dt.dayOfTheWeek() - 1) % 7;  // WTF???
  char *s = datestr;
  *s++ = ' ';
  for (int i = 0; i < 3; ++i) {
    *s++ = dow_names[3 * dow + i];
  }
  *s++ = ' ';
  s = sprint_int(s, dt.year());
  *s++ = '-';
  s = sprint_int2(s, dt.month());
  *s++ = '-';
  s = sprint_int2(s, dt.day()); 
  *s++ = ' ';
  *s++ = '\0'; 
}

int update_display(time_t t, uint8_t redraw=false) {
  // Returns minutes within day (0..1440).
  //u8g2.clearBuffer();   // for _F_ initializer only
  //display.fillScreen(bgcolor);
  int mid_x = display_mid_x;

  tmElements_t tm;
  breakTime(t, tm);
  serial_print_tm(tm);
  int mins_within_day = 60 * tm.Hour + tm.Minute;
  if (redraw || tm.Day != last_day) {
    last_day = tm.Day;
    char datestr[24];
    strcpy(datestr, datefmt);
    DateTime dt(t);
    //dt.toString(datestr);
    sprint_date(dt, datestr);    
    // Date.
    display.setFont(SMALLFONT);
    int16_t  x1, y1;
    uint16_t w, h;
    display.getTextBounds(datestr, (int16_t)0, (int16_t)0, &x1, &y1, &w, &h);
    //display.drawRect(mid_x - w/2 - 8, date_midline_y - h/2 - 1, w + 16, h + 2, fgcolor);      
    print_text(datestr, mid_x - w/2, date_midline_y - h / 2 - 1, LEFT, TOP, w + 16, h, true, SMALL_VSHIFT);
    //print_text(datestr, 0, 0, LEFT, TOP, w + 16, h, true, SMALL_VSHIFT);
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
  if (redraw || (prev_minute & 1) == 0) {
    // bar growing from left
    display.fillRect(base_x, base_y, bar_width, secs_height, fgcolor);
  } else {
    // bar shrinking to right
    display.fillRect(base_x, base_y, bar_width, secs_height, bgcolor);
  }

  // Logging plot setup
  const int log_x = mid_x - (log_width >> 1);
  const int log_y = log_top_y;
  const int log_w = log_width;
  const int log_h = log_height;

  // Large digits time.
  display.setFont(CalBlk36);
  char digit_string[3];
  //mid_x -= big_colon_width;
  if (redraw || tm.Hour != last_hour) {
    last_hour = tm.Hour;
    sprint_int2(digit_string, tm.Hour);
    digit_string[2] = '\0';
    print_text(digit_string, mid_x - (big_colon_width/2) - 3, time_midline_y, 
               RIGHT, MIDDLE, digits_width, digits_height, false,  digits_baseline_shift);
  }
  if (redraw || tm.Minute != last_minute) {
    last_minute = tm.Minute;
    sprint_int2(digit_string, tm.Minute);
    digit_string[2] = '\0';
    print_text(digit_string, mid_x + (big_colon_width/2) + 2, time_midline_y,
               LEFT, MIDDLE, digits_width, digits_height, false, digits_baseline_shift);
    // Update log when minutes change.
    draw_log_output(log_x, log_y, log_w, log_h);
  }
  if (colon_visible) {
    print_text(":", mid_x, time_midline_y + digits_baseline_shift);
  } else {
    display.fillRect(mid_x - 1 - 4, time_midline_y - (big_colon_height >> 1) - 4,
                 big_colon_width, big_colon_height, bgcolor);
  }
#ifdef DISPLAY_DISPLAY_CMD
  display.display();
#endif

  return mins_within_day;
}

// ------------- Display sleep (screensaver) -----------

// Moved up for CLI access
bool display_on = true;

int backlight_brightness = 0;

void wake_up_display(void) {
  Serial.println("wake_display");
#ifdef DISPLAY_BACKLIGHT
  // turn on backlite
  analogWrite(backlightPin, backlight_brightness);
#endif
  display_on = true;
  setup_display();
  update_display(now_local(), /* redraw= */ true);
}

void sleep_display(void) {
  Serial.println("sleep_display");
#ifdef DISPLAY_DISPLAY_CMD
  display.clearDisplay();
  display.display();
#else
  display.fillScreen(BLACK);
#endif
#ifdef DISPLAY_BACKLIGHT
  // turn off backlite
  analogWrite(backlightPin, 0);
#endif
  display_on = false;
}

// Sleep display after 5 mins.
//uint32_t display_sleep_timeout_secs = 300;
// forward-declared above serial command block.
uint32_t millis_last_action = 0;

void sleep_update(void) {
  // Check the time and sleep display if we've been idle.
  // We use millis since ds3231 time may be changing.
  if (millis_last_action && display_sleep_timeout_secs 
      && (millis() - millis_last_action) > 1000 * display_sleep_timeout_secs) {
    if (display_on) {
      sleep_display();
    }
  }
}

void sleep_tickle(void) {
  // Reset display sleep countdown.
  //Serial.print("tickle: display_on=");
  //Serial.println(display_on);
  millis_last_action = millis();
  if (!display_on) {
    wake_up_display();
  }
}

// ======================================================
// =========== Button management ==============
// ======================================================

#define NUM_BUTTONS 3  // on D9, D6, D5 on feather OLED wing are GPIO 9, 8, 7 on RP2040 Feather
#ifdef ARDUINO_ARCH_RP2040
#ifdef FEATHER_RP2040  // i.e., this is a Feather RP2040
    int button_pins[NUM_BUTTONS] = {9, 8, 7};
#warning "Feather RP2040"
#else  // RP2040 Pico
  #ifdef MY_PICO_RP2040_LCD
    int button_pins[NUM_BUTTONS] = {20, 21, 22};
  #else
    int button_pins[NUM_BUTTONS] = {18, 19, 20};
  #endif
#endif
#else
  int button_pins[NUM_BUTTONS] = {9, 6, 5};
#endif
int button_state[NUM_BUTTONS] = {0, 0, 0};
unsigned long button_last_change_time[NUM_BUTTONS] = {0, 0, 0};
// For distinguishing short/long press
int button_down_time = 0;

// Keeping track of user interaction to control display sleep.
long secs_last_action = 0;

#define DEBOUNCE_MILLIS 50
#define MILLIS_LONG_PRESS 500

void buttons_setup(void) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    pinMode(button_pins[button], INPUT_PULLUP);
  }
  sleep_tickle();
}

void buttons_update(void) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    // Buttons are pulled up, so read as 1 when open, 0 when pressed.
    int new_state = 1 - digitalRead(button_pins[button]);
    if (button_state[button] == new_state) {
      continue;
    }
    // State has changed.
    unsigned long millis_now = millis();
    unsigned long millis_since_last_change = millis_now - button_last_change_time[button];
    button_last_change_time[button] = millis_now;
    if (millis_since_last_change <= DEBOUNCE_MILLIS) {
      continue;
    }
    // Debounced state change
    button_state[button] = new_state;
    if (new_state) {
      // State changed to "pressed"
      button_down_time = millis_now;
      continue;
    }
    // State change was to "released".
    bool long_press = (millis_now - button_down_time) > MILLIS_LONG_PRESS;
    Serial.print("Button ");
    Serial.print(button);
    if (long_press) {
      Serial.println(" long press");
    } else {
      Serial.println(" short press");
    }
    // Action - tickle the screensaver.
    if (!display_on) {
      sleep_tickle();  // also wakes the display.
      // But otherwise ignore the press.
      return;
    }
    sleep_tickle();
    switch(button) {
      case 0:
        if (long_press) {
          sleep_display();
        } else {
          //display_detail = !display_detail;
        }
        break;
      case 1:
        // Increase aging register
        if (long_press) {
          //dac_save_to_eeprom();
        } else {
          //ds3231_delta_aging(1);
        }
        break;
      case 2:
        if (long_press) {
          // Long press syncs to GPS
          //request_RTC_sync = true;
          //Serial.print("gps_micros=");
          //Serial.print(gps_micros);
          //Serial.print(" last_gps_micros=");
          //Serial.println(last_gps_micros);
        } else {
          //ds3231_delta_aging(-1);
        }
        break;
    }
  }
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
    if (serial_available) Serial.println("Error: cmd arg string is not 4 chrs.");
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
    if (serial_available) Serial.println("Warn: Year does not start with 20...");
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
          case 'A':
            // Set aging offset.
            if (strlen(cmd_buffer + 1)) {
              value = atoi(cmd_buffer + 1);
              ds3231.setAgingOffset(value);
            }
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
            backlight_brightness = htoi(cmd_buffer + 1);
            //set_backlight_color(backlight_brightness);
#ifdef BACKLIGHT
            analogWrite(backlightPin, backlight_brightness);
#endif
            Serial.print("new color=");
            Serial.println(backlight_brightness);
            break;
          case 'X':
            // Set/read display sleep timeout in secs.
            if (cmd_buffer[1]) {
              display_sleep_timeout_secs = atoi(cmd_buffer + 1);
            }
            Serial.print("Display sleep (sec, 0=disabled)=");
            Serial.println(display_sleep_timeout_secs);
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
const int light_low = 8;
const int light_high = 64;
const int hour_up = 7;
const int hour_down = 22;

//int backlight_brightness = 0;

void setup_backlight(void) {
  // Backlight
#ifdef BACKLIGHT
  pinMode(backlightPin, OUTPUT);  // sets the pin as output
  analogWrite(backlightPin, backlight_brightness);
#endif
}

int bright_tick = 0;
const int ticks_per_step = 1; // 1ms delay on polling, was 4096;  

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
    int bright_delta = target_brightness - backlight_brightness;
    if (bright_delta) {
      backlight_brightness += (bright_delta >> 6) + sgn(bright_delta);
#ifdef BACKLIGHT
      analogWrite(backlightPin, backlight_brightness);
      if (serial_available) {
        //Serial.print("brightness=");
        //Serial.println(backlight_brightness);
      }
#endif // BACKLIGHT
    }
  }
}

// ---------------------------------
// Main
// ---------------------------------

#define SECS_PER_DAY (24 * 60 * 60)

//bool serial_available = false;

#define MAXWAIT_SERIAL 1000  // 200 = 2 seconds.

void open_serial(int baudrate=9600) {
  Serial.begin(baudrate);
  // Wait for Serial port to open
  int i = 0;
  while (!Serial) {
    delay(10);
    ++i;
    if (i > MAXWAIT_SERIAL) break;
  }
  if (i <= MAXWAIT_SERIAL) {
    serial_available = true;
  }
  delay(1000);
}

const int ledPin = 13; // On-board LED
uint8_t ledState = false;

void setup()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  open_serial();

  if (serial_available) {
    Serial.print(F("gfx_clock_logger "));
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);
  }

#  //pinMode(ledPin, OUTPUT);
  //digitalWrite(ledPin, HIGH);
#ifdef ARDUINO_ARCH_RP2040
  // Configure Pico RP2040 I2C
  // Internal I2C is used to communicate with I2C peripherals.
  //Serial.println("Setting INT_I2C pins...");
  // Feather RP2040 hangs if you try to set int_sda pins (2/3).
  //INT_I2C.setSDA(int_sda_pin);
  //INT_I2C.setSCL(int_scl_pin);
  // Wire is initialized insid OLED display.
  //Serial.println("INT_I2C pins set.");
  Serial.println("Configuring EXT_I2C...");
  EXT_I2C.setSDA(ext_sda_pin);
  EXT_I2C.setSCL(ext_scl_pin);
  Serial.println("EXT_I2C pins set.");
  EXT_I2C.begin();
#else
  //EXT_I2C.begin(ext_sda_pin, ext_scl_pin);
  //INT_I2C.begin();
#endif
  Serial.println("I2C configured.");

#ifdef DISPLAY_ST7789
  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);
#endif

  setup_RTC();
  Serial.println("RTC set up.");
  setup_interrupts();
  Serial.println("Interrupts set up.");
  setup_display();
  setup_backlight();
  Serial.println("done backlight");
  setup_temp_F();
  Serial.println("done temp");
  setup_logger();
  Serial.println("done logger");
  cmd_setup();
  Serial.println("done cmd");

  buttons_setup();
}

//time_t current_now = 60 * (9 * 60 + 50);
time_t current_now = 0; // 60 * (23 * 60 + 50);

void loop()
{
#ifdef DEBUG
  // Quick cycling of time
  if (current_now == 0) { current_now = now_local(); }
  ++current_now;
  int mins_within_day = update_display(current_now);
  update_logger(read_temp_F(), mins_within_day);
#else
  if (update_RTC()) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    current_now = now_local();
    if (display_on)
        update_display(current_now);
    update_logger(read_temp_F(), current_now);
  }
#endif
  cmd_update();
  buttons_update();
  update_backlight(hour(current_now));
  delay(50);
}
