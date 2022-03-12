/*
 *  glcd_clock (based on openGLCD Library)
 *  
 *  ks0108 128x64 LCD dot-matrix display is connected on *a lot* of pins, 
 *  as specified in openGLCD/config/ks0108/PinConfig_ks0108-Uno.h.
 *  
 *  This differs from the default openglcd because:
 *   - The data pins are in order (LCD D0 = Arduino D4 .. LCD D7 = Arduin D11) rather than nybble-swapped.
 *   - LCD E is moved to D12, freeing D3 ..
 *   - Backlight is on D3, which is PWM enabled
 *   
 *   For the clock RTC...
 *    - DS3231 SDA on A4
 *    - DS3231 SCL on A5
 *    - DS3231 SQWV on D2 (interrupt-capable).
 */

typedef unsigned long time_t;
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire
#include "RTClib.h"

// no font headers have to be included
#include <openGLCD.h>

// --------------------
// 1PPS interrupt
// --------------------

const int sqwPin = 2; // The number of the pin for DS3231 1PPS SQWV output

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
#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

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

//------------------------
// Display
//------------------------

void setup_display(void) {
  // Initialize the GLCD 
  GLCD.Init();

  // Set text area to be more centered.
  GLCD.DefineArea(14, 0, 127, 38);

  // Box around the second progress bar, 1 pixel separated.
  GLCD.DrawRect(2, 56, 124, 8);

  //GLCD.SelectFont(Arial_bold_14);
  //GLCD.SelectFont(TimesNewRoman16_bold);
  //GLCD.SelectFont(CalBlk36);
}

const char *fmt = "hh mm ";
uint8_t colon_visible = true;
uint8_t last_day = 0;

void update_display(const DateTime& dt) {

   // Day/Date at top.
   if (dt.day() != last_day) {
      last_day = dt.day();
      GLCD.SelectFont(System5x7);
      GLCD.CursorToXY(0, 0);
      char datefmt[] = "DDD YYYY-MM-DD";
      GLCD.print(dt.toString(datefmt));
      // Switch font back.
      GLCD.SelectFont(CalBlk36);
    }
    
    // Main time digits
    //GLCD.ClearScreen();
    GLCD.CursorToXY(0, 14);
    char buf[8];
    strcpy(buf, fmt);
    GLCD.print(dt.toString(buf));
    if (colon_visible) {
      buf[2] = '\0';
      GLCD.CursorToXY(GLCD.StringWidth(buf), 14);
      GLCD.print(":");
    }
    colon_visible = !colon_visible;

    // Seconds progress bar.
    uint8_t x_scale = 2;
    uint8_t height = 4;
    uint8_t base_y = 58;
    uint8_t base_x = 64 - x_scale * 30;
    // Bar width goes from 1 to 60 (instead of 0 to 59), so we have to work with one second ago.
    int prev_minute = dt.minute();
    int prev_second = dt.second() - 1;
    if (prev_second < 0) {
      prev_minute = (prev_minute + 59) % 60;
      prev_second += 60;
    }
    uint8_t width_l = x_scale * (1 + prev_second);
    // Draw/undraw transition occurs on second 1.  Second 0 is drawing the 60th tick on the line.
    if (prev_minute & 1) {
      // undraw bar.
      GLCD.FillRect(base_x, base_y, width_l, height, PIXEL_OFF);
    } else {
      // draw bar.
      GLCD.FillRect(base_x, base_y, width_l, height);
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

const int backlightPin = 3;

void setup_backlight(void) {
  // Backlight
  pinMode(backlightPin, OUTPUT);  // sets the pin as output
  analogWrite(backlightPin, light_low);
}

int target_brightness = 0;

void update_backlight_time(int hour) {
  target_brightness = light_low;
  if (hour >= hour_up && hour < hour_down) {
    target_brightness = light_high;
  }
}

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

int brightness = 0;
int brightness_tick = 0;
const int ticks_per_step = 1;

void update_backlight() {
  int bright_delta = target_brightness - brightness;
  if (bright_delta) {
    if (++brightness_tick >= ticks_per_step) {
      brightness_tick = 0;
      brightness += (bright_delta >> 6) + sgn(bright_delta);
      analogWrite(backlightPin, brightness);
      //Serial.print("brightness=");
      //Serial.println(brightness);
    }
  }
}

// -------------------------
// Main
// -------------------------

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("glcd_clock");

  setup_RTC();
  setup_interrupts();
  setup_backlight();

  setup_display();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  if (pending_RTC_interrupt) {
    pending_RTC_interrupt = false;
    time_t now_now = now_local();
    DateTime dt(now_now);
    update_backlight_time(dt.hour());
    update_display(dt);
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // Backlight update is at full-speed, not 1 Hz.
  update_backlight();
  delay(10);
}
