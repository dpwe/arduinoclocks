/*
 *  u8g2_clock based on glcd_clock.
 *  
 * Wiring:
 * 
 *  Arduino Nano : 128x64 ST7920 display
 *    D13 (SCK)  : CLK 
 *    D11 (MOSI) : DAT
 *    D10        : CS
 *    D8 (PWM)   : LEDA (via 220 ohm resistor)
 *    GND        : LEDK
 *    GND        : PSB (to select serial i/f)
 *    5V         : VCC
 *    GND        : GND
 *    
 *  Arduino Nano : DS3231
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
  bool handled_interrupt = false;
  if (pending_RTC_interrupt) {
    pending_RTC_interrupt = false;
    handled_interrupt = true;
  }
  return handled_interrupt;
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

//------------------------
// Display
//------------------------

//#define YWROBOT_12864
#ifdef YWROBOT_12864
#pragma message("Using YwRobot")
// YwRobot 128x64
//U8G2_UC1701_MINI12864_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
U8G2_ST7565_ERC12864_ALT_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);  // contrast improved version for ERC12864
#else
// SPI Green LCD panel
U8G2_ST7920_128X64_2_HW_SPI u8g2(U8G2_R0, /* CS=*/ 10, /* reset=*/ 8); // Feather HUZZAH ESP8266, E=clock=14, RW=data=13, RS=CS
#endif
// RP2040 feather
//#define SPI SPI0

//#define SMALLFONT u8g2_font_profont11_tr
#define SMALLFONT System5x7

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
}

char *sprint_int2(char *s, uint8_t n)
{  // Always 2 digits, assume n nonnegative, < 99.
  *s++ = '0' + (n / 10);
  *s++ = '0' + (n % 10);
  return s;
}

uint8_t last_day = 0;
uint8_t colon_visible = true;

void update_display(time_t t) {
  //u8g2.clearBuffer();   // for _F_ initializer only

  // Set text area to be more centered.
  //GLCD.DefineArea(14, 0, 127, 38);

  tmElements_t tm;
  breakTime(t, tm);
  serial_print_tm(tm);
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

  const int time_y = 14;

  int colon_x = mid_x - (big_colon_width >> 1);
  int minute_x = colon_x + big_colon_width;
  int hour_x = colon_x - u8g2.getStrWidth(hr_string);
  // Manual adjustment
  colon_x += 2;

  // Big digits dimensions
  // Seconds bar dimensions
  uint8_t x_scale = 2;
  uint8_t height = 4;
  uint8_t base_y = 58;
  uint8_t base_x = 64 - x_scale * 30;
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

  u8g2.firstPage();
  do {
    u8g2.setFont(SMALLFONT);
    u8g2.drawStr(date_x, 0, datestr);
    //u8g2.setFont(u8g2_font_logisoso34_tn);
    u8g2.setFont(CalBlk36);
    u8g2.drawStr(hour_x, time_y, hr_string);
    u8g2.drawStr(minute_x, time_y, min_string);
    if (colon_visible) {
      u8g2.drawStr(colon_x, time_y, ":");
    }
    //  Seconds progress bar.
    // Box around the second progress bar, 1 pixel separated.
    u8g2.drawFrame(2, 56, 124, 8);
    // Draw/undraw transition occurs on second 1.  Second 0 is drawing the 60th tick on the line.
    u8g2.drawBox(bar_left, base_y, bar_width, height);
  } while (u8g2.nextPage());
  //u8g2.sendBuffer();  // for _F_ initializer only
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

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("u8g2_clock");

  setup_RTC();
  setup_interrupts();
  setup_display();
  setup_backlight();
}

time_t current_now = 0;

void loop()
{
  if (update_RTC()) {
    current_now = now_local();
    update_display(current_now);
  }
  update_backlight(hour(current_now));
}
