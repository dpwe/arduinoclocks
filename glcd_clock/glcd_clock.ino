/*
 *  glcd_clock based on openGLCD Library - Hello World
 * 
 */

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

const int sqwPin = 2; // The number of the pin for monitor alarm status on DS3231

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

unsigned long last_rtc_micros = 0;

void sync_time_from_RTC(void) {
  // This is called soon after an A0 transition is detected, so RTC unixtime is still current.
  //rtc_clock.sync(rtc.now().unixtime(), rtc_micros);
  last_rtc_micros = rtc_micros;
}

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

void update_RTC(void) {
  if (pending_RTC_interrupt) {
    sync_time_from_RTC();
    pending_RTC_interrupt = false;
    Serial.println("RTC IRQ");
  }
}

time_t RTC_utc_get(void) {
  return rtc.now().unixtime();
}


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


void RTC_setup(void) {
  // Have the now() function return UTC.
  setSyncProvider(RTC_utc_get);   // the function to get the time from the RTC
  if(timeStatus() != timeSet)
     Serial.println("Unable to sync with the RTC");
  else {
     Serial.println("RTC has set the system time");
     tmElements_t tm;
     breakTime(now_local(), tm);
     serial_print_tm(tm);
  }
}

//------------------------
// Display
//------------------------

// include the library header
// no font headers have to be included
#include <openGLCD.h>

uint8_t last_day = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("glcd_clock");

  setup_RTC();
  RTC_setup();
  setup_interrupts();
  
  // Initialize the GLCD 
  GLCD.Init();

  // Set text area to be more centered.
  GLCD.DefineArea(14, 0, 127, 38);

  // Box around the second progress bar, 1 pixel separated.
  GLCD.DrawRect(2, 56, 124, 8);

  //GLCD.SelectFont(Arial_bold_14);
  //GLCD.SelectFont(TimesNewRoman16_bold);
  //GLCD.SelectFont(CalBlk36);
  

//  GLCD.print(F("hello, world!")); // keep string in flash on AVR boards with IDE 1.x
//  GLCD.Puts(F("hello, world!")); // Puts() supports F() with any version of IDE

  // print() below uses RAM on AVR boards but works
  // on any version of IDE with any processor
  // note: Same is true for Puts()
  //GLCD.print("hello, world!"); 
}

const char *fmt = "hh mm ";

uint8_t colon_visible = true;

void loop()
{
  if (pending_RTC_interrupt) {
    pending_RTC_interrupt = false;
    time_t now_now = now_local();
    DateTime dt(now_now);
    if (dt.day() != last_day) {
      last_day = dt.day();
      GLCD.SelectFont(System5x7);
      GLCD.CursorToXY(0, 0);
      char datefmt[] = "DDD YYYY-MM-DD";
      GLCD.print(dt.toString(datefmt));
      // Switch font back.
      GLCD.SelectFont(CalBlk36);
    }
    //GLCD.ClearScreen();
    GLCD.CursorToXY(0, 14);

    char buf[20];
    strcpy(buf, fmt);
    GLCD.print(dt.toString(buf));
    if (colon_visible) {
      buf[2] = '\0';
      GLCD.CursorToXY(GLCD.StringWidth(buf), 14);
      GLCD.print(":");
    }
    colon_visible = !colon_visible;

    //  Second progress bar.
    uint8_t x_scale = 2;
    uint8_t height = 4;
    uint8_t base_y = 58;
    uint8_t base_x = 64 - x_scale * 30;
    // Bar width goes from 1 to 60 (instead of 0 to 59)
    uint8_t width_l = x_scale * (1 + second(now_now - 1));
    // Draw/undraw transition occurs on second 1.  Second 0 is drawing the 60th tick on the line.
    if (minute(now_now - 1) & 1) {
      // undraw bar.
      GLCD.FillRect(base_x, base_y, width_l, height, PIXEL_OFF);
    } else {
      // draw bar.
      GLCD.FillRect(base_x, base_y, width_l, height);
    }
  }
}
