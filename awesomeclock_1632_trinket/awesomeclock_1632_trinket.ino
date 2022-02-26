
#include <avr/power.h>  // for clock adjustment

#define HT1632_NUMBERS_ONLY  // Actually have to set directly in HT1632.cpp.
#include "HT1632.h"

// See https://learn.adafruit.com/16x24-led-matrix/wiring
// Pin 0 is I2C DAT
#define CS   1  // White
// Pin 2 is I2C CLK
#define WR   3  // Yellow
#define DATA 4  // Orange
//#define CS2  5

// use this line for single matrix
HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS);
// use this line for two matrices!
//HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS, CS2);


// ---------------------------------
// RTC clock
// ---------------------------------

#include <Wire.h>

// Super-minimal DS3231 interface, from http://www.RinkyDinkElectronics.com/

typedef uint32_t time_t;

class Time
{
public:
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   date;
  uint8_t   month;
  uint16_t  year;
  uint8_t   dow;

  Time(uint16_t year=2014, uint8_t mon=1, uint8_t date=1, uint8_t hour=0, uint8_t min=0, uint8_t sec=0);
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

class DS3231
{
  public:
    bool begin(void);
    Time  now();

  private:
    static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
};

boolean DS3231::begin(void) {
  Wire.begin();
  return true;
}

#define CLOCK_ADDRESS 0x68
Time DS3231::now()
{
  // Directly read the first 8 values from the RTC chip
  uint8_t buffer[8];
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(0x0);
  Wire.endTransmission();
  Wire.requestFrom(CLOCK_ADDRESS, 8);
  for(int i = 0; i < 8; ++i) {
     buffer[i] = Wire.read();
  }
  // Decode/format time.
  return Time(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5] & 0x7F),
              bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
              bcd2bin(buffer[0] & 0x7F));
}

#define DO_TIMEZONE

#ifdef DO_TIMEZONE
#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
#endif 

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
  uint8_t second_sunday_date = 14 - ((march_first_dow - 1) % 7);
  dst_start_day = 31 + 28 + ((year % 4) == 0) + second_sunday_date - 1;
  // March and November are 245 days == 35.0 weeks apart, so 2nd sunday in Nov is the same DOW
  dst_end_day = dst_start_day + 245;
  dst_cache_year = year;
}

// Only works for UTC-3 to UTC-21 (so that 2am local is still the same date in UTC).
#define STANDARD_TIME_DIFF_HOURS -5

void make_localtime(Time &now) {
  if (dst_cache_year != now.year)  calc_timechange_days(now.year);
  // 2am local in North America is 2am + 4/5 in UTC
  int DoY = day_of_year(now.year - 2000, now.month, now.date);
  bool sprung_forward = (DoY > dst_start_day) ||
                        ((DoY == dst_start_day) && (now.hour >= 2 - STANDARD_TIME_DIFF_HOURS));
  bool fallen_back = (DoY > dst_end_day) ||
                      ((DoY == dst_end_day) && (now.hour >= 2 - STANDARD_TIME_DIFF_HOURS));
  bool is_dst = sprung_forward - fallen_back;
  int signed_hour = now.hour;
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

void endless_flash(int period_ms, int count=-1) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (count--) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(period_ms);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(period_ms);                       // wait for a second
  }
}

DS3231 rtc;

void rtc_setup() {
  if (! rtc.begin()) {
    //Serial.println("Couldn't find RTC");
    //Serial.flush();
    // Rapid flash to indicate "no RTC" state.
    endless_flash(100);
  }
}

// ---------------------------------
// Matrix display
// ---------------------------------

void matrix_setup() {
  //Serial.begin(9600);
  //Serial.println("** awesomeclock_1632_trinket **");
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
        // undraw old character
        //matrix.setCursor(x, 0);
        //matrix.setTextColor(0);   // 'off' LEDs
        //matrix.write(state[i]);   
        // draw new character   
        //matrix.setCursor(x, 0);
        //matrix.setTextColor(1);   // 'on' LEDs
        //matrix.write(new_state[i]);      
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

#define BACKLIGHT
#ifdef BACKLIGHT
// Config for backlight day/night mode.
const int light_low = 1;
const int light_high = 15;
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

#endif // BACKLIGHT

// ---------------------------------
// Main
// ---------------------------------

Time start;
unsigned long start_ms = 0;

void setup() {
  // Run the Trinket at 16 MHz
  //if (F_CPU == 16000000) clock_prescale_set(clock_div_1);

  rtc_setup();
  start = rtc.now();
  start_ms = millis();
  //endless_flash(100, start.sec);

  pinMode(CS, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(DATA, OUTPUT);
  matrix_setup();
#ifdef BACKLIGHT
  backlight_setup();
#endif // BACKLIGHT
}

void wind_clock(Time &start) {
  // Advance <start> one sec.
  start.sec = (start.sec + 1) % 60;
  if (start.sec) return;
  start.min = (start.min + 1) % 60;
  if (start.min) return;
  start.hour = (start.hour + 1) % 24;
  // Don't bother updating Date for now.
}

uint8_t last_sec = 0;

#define USE_RTC

Time wait_for_next_sec(void) {
#ifdef USE_RTC
  while(rtc.now().sec == last_sec) {
    delay(20);
  }
  return rtc.now();
#else
  while((millis() - start_ms) < 1000) {
    delay(20);
  }
  start_ms += 1000;
  wind_clock(start);
  return start;
#endif
}

void loop() {
  Time now = wait_for_next_sec();
  make_localtime(now);
  last_sec = now.sec;
  int millis_start = millis();

  matrix_update(now.hour, now.min, now.sec);
  while(transition_state != 0) {
    delay(20);  
    matrix_update(now.hour, now.min, now.sec);
  }
#ifdef BACKLIGHT
  backlight_update(now.hour);
#endif // BACKLIGHT
  
  int millis_to_wait = 950 - (millis() - millis_start);
  //Serial.print("millis_to_wait=");
  //Serial.println(millis_to_wait);
  delay(millis_to_wait);
}
