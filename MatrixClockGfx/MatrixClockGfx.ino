/*
 * MatrixClockGfx
 * 
 * Simulating the 64x64 SmartMatrix clock on a Adafruit GFX LCD
 * 
 * Code based on MatrixClockDpwe
 * 
 * This one runs on an ESP32-Feather-TFT, and gets time from NTP.
 * 
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


#include <Wire.h>
#include <TimeLib.h>

const int defaultBrightness = 240;
//const int defaultBrightness = (2*255)/100;     // dim: 2% brightness

//const SM_RGB clockColor = {0xc0, 0xc0, 0xc0};
// note: clockColor is now set in set_brightness.
const uint8_t clockIntensity = 0xc0;
//const SM_RGB clockColor = {0x10, 0x40, 0xc0};
//const SM_RGB clockColor2 = {0x50, 0x60, 0x80};

#define RGB565(r, g, b) (((r & 0xF8)<<8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

// 565 RGB 16-bit colors
int fgColor = RGB565(0xc0, 0xe0, 0xff); // 0x0847E6; // Bright blueish-green // ST77XX_BLACK;
int bgColor = 0; // ST77XX_WHITE;

//const SM_RGB dotColor = {0xff, 0x0, 0x0};
int dotColor = RGB565(0xFF, 0, 0);


short int rtc_tick = 0;

// from https://forum.pjrc.com/threads/32254-Teensy-3-x-RTC-Seconds-Interrupt
void rtc_seconds_isr(void)
{
    static int state=0;

    state = state ^ 1;
    pinMode(13, OUTPUT);
    digitalWrite(13, state);

    rtc_tick = 1;
}

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
  struct tm timeinfo;
  timeinfo = *localtime (&tv->tv_sec);
  Serial.print("Got time adjustment from NTP: ");
  //Serial.println(timeinfo, "%A, %B %d %Y %H:%M:%S");
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
    Serial.print(timeinfo.tm_hour);
    Serial.print(":");
    Serial.print(timeinfo.tm_min);
    Serial.print(":");
    Serial.println(timeinfo.tm_sec);
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


// Excised from BurstClockSerial, so we can support the same clock set syntax.

byte atoi2(char *s) {
  // Convert two ascii digits to a uint8.
  return (s[1] - '0') + 10 * (s[0] - '0');
}

struct tm parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  struct tm tm;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  tm.tm_year = y2kYearToTm(atoi2(time_string + 2));
  tm.tm_mon = atoi2(time_string + 4) - 1;
  tm.tm_mday = atoi2(time_string + 6);
  tm.tm_hour = atoi2(time_string + 8);
  tm.tm_min = atoi2(time_string + 10);
  tm.tm_sec = atoi2(time_string + 12);
  return tm;
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(void) {
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            if (strlen(cmd_buffer) != 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              //Teensy3Clock.set(makeTime(parse_time_string(cmd_buffer + 1)));
              //resync_time();
              Serial.print("Set time - ");
              Serial.print(hour());
              Serial.print(":");
              Serial.println(minute());
            }
            break;
          case 'B':
            int brightness = atoi(cmd_buffer + 1);
            set_brightness(brightness);
            Serial.print("Brightness: ");
            Serial.println(brightness);
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

//------------------------
// Display
//------------------------

uint16_t HsvToRgb(uint8_t h, uint8_t s, uint8_t v)
{
    uint8_t r, g, b;  // rgb;  // 0x00RRGGBB
    uint8_t region, remainder, p, q, t;
    
    if (s == 0)
    {
        r = v;
        g = v;
        b = v;
        return RGB565(r, g, b);
    }
    
    region = h / 43;
    remainder = (h - (region * 43)) * 6; 
    
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
    
    switch (region)
    {
        case 0:
            r = v; g = t; b = p;
            break;
        case 1:
            r = q; g = v; b = p;
            break;
        case 2:
            r = p; g = v; b = t;
            break;
        case 3:
            r = p; g = q; b = v;
            break;
        case 4:
            r = t; g = p; b = v;
            break;
        default:
            r = v; g = p; b = q;
            break;
    }
    
    return RGB565(r, g, b);
}

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
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#define SMALLFONT &FreeSans12pt7b
#define MICROFONT 0
#define CalBlk36 &FreeSansBold24pt7b

#define TIMEFONT &FreeSansBold18pt7b
#define DATEFONT &FreeSans12pt7b

uint16_t big_colon_width = 0;
uint16_t big_colon_height = 0;
uint16_t time_width = 0;
uint16_t time_height = 0;
uint16_t date_width = 0;
uint16_t date_height = 0;
const char *datefmt = "DDD YYYY-MM-DD";

enum text_alignment {
  TOP,
  MIDDLE,
  BOTTOM,
  LEFT,
  RIGHT,
};

//int backlight_val = 32;
const int MHEIGHT = 135;
const int MWIDTH = 240;
const uint16_t kMatrixWidth = 128;       // Set to the width of your display, must be a multiple of 8
const uint16_t kMatrixHeight = 128;      // Set to the height of your display
const int MTOP = ((MHEIGHT - kMatrixHeight)>>1);
const int MLEFT = ((MWIDTH - kMatrixWidth)>>1);

void print_text(const char *s, int16_t x, int16_t y,
                int8_t x_align = MIDDLE, int8_t y_align = MIDDLE,
                int16_t clear_width = 0, int16_t clear_height = 0, bool debug = false) {
  // Set clear_width to -1 to skip clear-before-write.
  x += MLEFT;
  y += MTOP;
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
      tft.fillRect(x - clear_width, y1 - h + 1, clear_width + 4, clear_height, bgColor);
    } else {
      tft.fillRect(x1 - 1, y1 - h + 1, clear_width + 2, clear_height, bgColor);
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

void setup_display(void) {

  // turn on backlite
  //pinMode(TFT_BACKLITE, OUTPUT);
  //analogWrite(TFT_BACKLITE, backlight_val);

  // initialize TFT
  tft.init(MHEIGHT, MWIDTH); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(bgColor);

  tft.setTextColor(fgColor);

  // Serial availability status.
  tft.setFont(MICROFONT);
  tft.setCursor(MWIDTH - 8, 0);
  tft.print("S");
  if (!serial_available) {
    tft.setCursor(MWIDTH - 8, 0);
    tft.print("X");
  }

  // Preset box widths.
  tft.setFont(TIMEFONT);
  int16_t x1, y1;
  tft.getTextBounds("00:00", (int16_t)0, (int16_t)0, &x1, &y1, &time_width, &time_height);
  time_width += 1;
  time_height += 1;
  tft.setFont(DATEFONT);
  tft.getTextBounds("00-00", (int16_t)0, (int16_t)0, &x1, &y1, &date_width, &date_height);
  date_width += 1;
  date_height += 1;

  // Seconds progress bar frame.
  // Seconds bar dimensions
  //const uint8_t base_y = seconds_midline_y + secs_height / 2;
  //const uint8_t base_x = display_mid_x - secs_x_scale * 30;
  // Box around the second progress bar, 1 pixel separated.
  //tft.drawRect(base_x - 2, base_y - 2, 60 * secs_x_scale + 4, secs_height + 4, fgColor);

  Serial.println("setup_display done.");
}

void show_ntp_update(struct tm *timeinfo)
{
  // Update display.
  char msg[40];
  char *s = msg;
  strcpy(s, " Last update: ");
  s += strlen(s);
  //sprint_tm(timeinfo, s);
  s += strlen(s);
  *s++ = ' ';
  *s++ = '\0';
  tft.setFont(MICROFONT);
  print_text(msg, display_mid_x, ntpstat_mid_y);
}

uint8_t colon_visible = true;
int last_second = -1;
int hour_ = 0;
int minute_ = 0;
int second_ = 0;
int loop_count = 0;

unsigned long int millisLastTick = -1;

unsigned long int millisSinceTick(void) {
  return millis() - millisLastTick;
}

void draw_fat_string(int x, int y, char *string) {
  // Convolve string with a 2x2 rectangle to make it fat.
  tft.setCursor(x + 1, y);  tft.print(string);
  tft.setCursor(x+1, y);  tft.print(string);
  tft.setCursor(x, y + 1);  tft.print(string);
  tft.setCursor(x+1, y + 1);  tft.print(string);
}

void drawPixel(int x, int y, uint16_t c) {
  tft.drawPixel(MLEFT + x, MTOP + y, c);
}

void update_display(struct tm& tm) {

  update_time_date(tm);
  update_sec_hand(tm);
  update_sec_bar(tm);

  //backgroundLayer.swapBuffers();
}

void update_time_date(struct tm& tm) {
  second_ = tm.tm_sec;
  if (second_ != last_second) {
    // first loop since clock ticked.
    last_second = second_;
    millisLastTick = millis();
  }
  if (minute_ != tm.tm_min) {
    // Update the time variables once per min.
    hour_ = tm.tm_hour;
    minute_ = tm.tm_min;

    // Update the time digits.
    char timeBuffer[12];

    // Clear screen before writing new text.
    //tft.fillRect(MLEFT, MTOP, kMatrixWidth, kMatrixHeight, bgColor);

    // Time in top half.
    int x = kMatrixWidth / 2;
    int y = kMatrixHeight / 2 - (time_height >> 2) - 2;
    sprintf(timeBuffer, "%02d:%02d", hour_, minute_);
    // indexedLayer.setIndexedColor(1, clockColor);
    //draw_fat_string(x, y, timeBuffer);
    tft.setFont(TIMEFONT);
    print_text(timeBuffer, x, y, MIDDLE, BOTTOM, time_width, time_height);

    // Date in bottom half.
    x = kMatrixWidth / 2;
    y = kMatrixHeight / 2 + (time_height >> 2);
    sprintf(timeBuffer, "%02d-%02d", tm.tm_mon + 1, tm.tm_mday);
    // indexedLayer.setIndexedColor(1, clockColor2);
    //draw_fat_string(x, y, timeBuffer);
    tft.setFont(DATEFONT);
    print_text(timeBuffer, x, y, MIDDLE, TOP, date_width, date_height);

    // Track # millis between each tick.
    //sprintf(timeBuffer, "%4d", cached_millisSinceTick);
    //indexedLayer.setFont(font5x7);
    //indexedLayer.drawString(44, 55, 1, timeBuffer);
  
    //indexedLayer.swapBuffers();
  }
}

//const int dot_halfwidth = 3;
//const float dot_sq_radius = 5.0;

const int dot_halfwidth = 5;
const float dot_sq_radius = 20.0;

void update_sec_hand(struct tm& tm) {
  // Moving second hand.
  float angle = 2.0 * 3.1415926 * (float(second_) + float(millisSinceTick())/1000) / 60.0;
  float r = (kMatrixWidth / 2.0) - 4.0; //28.0;
  float x_center = kMatrixWidth / 2.0, y_center = kMatrixHeight / 2.0;
  float x_point = x_center + r * sin(angle);
  float y_point = y_center - r * cos(angle);
  //backgroundLayer.fillCircle((int16_t)x_point, (int16_t)y_point, /* r= */ 3, dotColor);
  for (int x = (int16_t)x_point - dot_halfwidth; x <= (int16_t)x_point + dot_halfwidth; ++x) {
    for (int y = (int16_t)y_point - dot_halfwidth; y <= (int16_t)y_point + dot_halfwidth; ++y) {
      float x_rel = float(x) - x_point;
      float y_rel = float(y) - y_point;
      // Pixel intensity declines as distance-from-center squared out to sqrt(5) pixels.
      int value = int(255.0 * max(0.0, (1.0 - ((x_rel * x_rel + y_rel * y_rel) / dot_sq_radius))));
      uint16_t color = HsvToRgb(140, 128, value);
      drawPixel(x, y, color);
    }
  }

  // 5-sec dots.
  for (int tick_hour = 0; tick_hour < 12; ++tick_hour) {
    angle = 2.0 * 3.1415926 * float(tick_hour) / 12.0;
    int x = (int16_t)round(x_center + r * sin(angle));
    int y = (int16_t)round(y_center - r * cos(angle));
    uint16_t color = HsvToRgb(50, 128, 255);
    drawPixel(x, y, color);
  }
}

void update_sec_bar(struct tm& tm) {
  // Rainbow seconds-bar.
  loop_count = (loop_count + 1) % 256;
  int pixels_per_col = kMatrixWidth / 60;
  // Precalculate intensity by row.
  //    int h = 5;
  //    int value = 25 * (10 - (abs(row - 2) * abs(row - 2)));
  //int row_values[] = {150, 225, 250, 225, 150};
  int row_values[] = {150, 180, 225, 240, 250, 240, 225, 180, 150};
  int num_rows = sizeof(row_values) / sizeof(int);

  int first_col, last_col;
  if ((minute_ & 1) == 0) {
    first_col = 0;
    last_col = second_;
  } else {
    first_col = second_;
    last_col = 60;
  }
  
  for (int col = first_col; col < last_col; ++col) {
    // Skip one pixel every 15 sec, and another at 30 sec.
    int x = pixels_per_col * (col + 0 + (col / 15) + (col / 30));
    // indexedLayer.drawPixel(pixel + 2, kMatrixHeight / 2 + 3, 1 + (pixel & 1));
    for (int row = 0; row < num_rows; ++row) {
      int y = kMatrixHeight / 2 - (num_rows / 2) + row;
      int hue = ((col + row) * 4 + loop_count) % 256;
      int saturation = 255;
      int value = row_values[row];
      uint16_t color = HsvToRgb(hue, saturation, value);
      for (int offset = 0; offset < pixels_per_col; ++offset) {
         drawPixel(x + offset, y, color);
      }
    }
  }
  // Maybe undraw the preceding second.
  if ( ((minute_ & 1) && (second_ > 0)) || (((minute_ &1) == 0) && (second_ == 0))) {
    // When moving left-hand edge, we also need to undraw the just-left second.
    int col = (first_col + 59) % 60;
    int x = pixels_per_col * (col + 0 + (col / 15) + (col / 30));
    for (int row = 0; row < num_rows; ++row) {
      int y = kMatrixHeight / 2 - (num_rows / 2) + row;
      for (int offset = 0; offset < pixels_per_col; ++offset) {
        drawPixel(x + offset, y, 0);
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

int brightness = 0;

void set_brightness(uint8_t brightness) {
    analogWrite(backlightPin, brightness);
}

void setup_backlight(void) {
  // Backlight
#ifdef BACKLIGHT
  pinMode(backlightPin, OUTPUT);  // sets the pin as output
  set_brightness(brightness);
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
      set_brightness(brightness);
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

void setup() {
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
  setup_backlight();

}


void loop() {

  cmd_update();

  // Time including DST.
  time_t tim;
  struct tm tm;
  time(&tim);
  tm = *localtime(&tim);
  update_backlight(tm.tm_hour);

  update_display(tm);

  // Start next loop at what we expect to be 10ms after the next tick.
  //delay(1010 - millisSinceTick());
  delay(50);
  
}
