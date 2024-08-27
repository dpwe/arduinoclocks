// awesomeclock_1632_atmega
//
// Clock for 16x24 LED matrix using HT1632 driver
// and DS3231 (programmed to UTC) for time.
//
// This version is setup to use a regular Arduino (not an ATTiny).

// Wiring for 16x24 LED matrix connected to ATMEGA328P board 
// (Adafruit Boarduino + FTDI cable; Program as "Duemillenova with 328P")
#define DATA 3  // Orange
#define CS   5  // White
#define WR   4  // Yellow

// Chronodot I2C
// SDA -> A4  (Blue)
// SCL -> A5  (Yellow)
// SQWV -> D2 (Green; not actually used in this script because ATTiny can't use it.)

#define HT1632_NUMBERS_ONLY  // Actually have to set directly in HT1632.cpp.
#include "HT1632.h"
// Single matrix
HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS);

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

class DS3231
{
  public:
    bool begin(void);
    Time now();
    void set(Time t);

  private:
    static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
};

boolean DS3231::begin(void) {
  Wire.begin();
  return true;
}

#define CLOCK_ADDRESS 0x68

#define DS3231_ADDRESS 0x68   ///< I2C address for DS3231
#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_AGING 0x10     ///< Aging offset register
#define DS3231_TEMPERATUREREG                                                  \
  0x11 ///< Temperature register (high byte - low byte is at 0x12), 10-bit
       ///< temperature value

void i2c_dev_write(uint8_t *buf, int num) {
  Wire.beginTransmission(CLOCK_ADDRESS);
  for (int i = 0; i < num; ++i) {
    Wire.write(buf[i]);
  }
  Wire.endTransmission();
}

void i2c_dev_read(uint8_t *buf, int num) {
  Wire.requestFrom(CLOCK_ADDRESS, num);
  for(int i = 0; i < num; ++i) {
     buf[i] = Wire.read();
  }
}

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

uint8_t read_register(uint8_t reg) {
  uint8_t buffer[1];
  i2c_dev_write(&reg, 1);
    i2c_dev_read(buffer, 1);
  return buffer[0];
}

void write_register(uint8_t reg, uint8_t val) {
  uint8_t buffer[2] = {reg, val};
  i2c_dev_write(buffer, 2);
}

void read_registers(uint8_t reg, uint8_t* buffer, uint8_t num) {
  // Put register index in first byte of buffer, overwritten on read.
  //buffer[0] = reg;
  //i2c_dev->write_then_read(buffer, 1, buffer, num);
  i2c_dev_write(&reg, 1);
  i2c_dev_read(buffer, num);
}

void write_registers(uint8_t reg, const uint8_t* buffer, uint8_t num) {
  uint8_t my_buffer[20];
  //assert(num <= 19);
  // We need to assemble a single buffer with reg followed by payload.
  my_buffer[0] = reg;
  for (int i = 0; i < num; ++i) {
    my_buffer[1 + i] = buffer[i];
  }
  i2c_dev_write(my_buffer, num + 1);
}

void DS3231::set(Time t) {
  uint8_t buffer[7] = {bin2bcd(t.sec),
                       bin2bcd(t.min),
                       bin2bcd(t.hour),
                       0, // bin2bcd(dowToDS3231(dt.dayOfTheWeek())),
                       bin2bcd(t.date),
                       bin2bcd(t.month),
                       bin2bcd(t.year - 2000U)};
  write_registers(DS3231_TIME, buffer, 7);
  
  uint8_t statreg = read_register(DS3231_STATUSREG);
  statreg &= ~0x80; // flip OSF bit
  write_register(DS3231_STATUSREG, statreg);
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
  uint8_t second_sunday_date = 14 - ((march_first_dow - 1) % 7);
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

// Error state, but there's no Serial on ATTiny, so do rapid flash of LED.
void endless_flash(int period_ms=100, int count=-1) {
  // Default "endless" is really only 65k flashes.
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

Time parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  Time t;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  t.year = 2000 + atoi2(time_string + 2);
  t.month = atoi2(time_string + 4);
  t.date = atoi2(time_string + 6);
  t.hour = atoi2(time_string + 8);
  t.min = atoi2(time_string + 10);
  t.sec = atoi2(time_string + 12);
  return t;
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

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

bool enable_rtc_updates = true;

void cmd_update(void) {
  int value;
  while (Serial.available() > 0) {
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
            //ds3231.setAgingOffset(value);
            Serial.print("Aging offset=");
            //Serial.println((int8_t)ds3231.getAgingOffset());
            break;
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            Serial.println("Z command");
            if (strlen(cmd_buffer) < 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              Time t = parse_time_string(cmd_buffer + 1); 
              rtc.set(t);
              Serial.println("Time set.");
            }
            serial_print_tm(rtc.now());
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
// Main
// ---------------------------------

Time start;
unsigned long start_ms = 0;

void setup() {
  // Run the Trinket at 16 MHz
  //if (F_CPU == 16000000) clock_prescale_set(clock_div_1);

  // initialize serial communication at 9600 bits per second.
  Serial.begin(9600);

  Serial.print("awesomeclock_1632_atmega ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.println("Post-reset settling...");
  delay(200);

  rtc_setup();
  start = rtc.now();
  serial_print_tm(start);
  start_ms = millis();
  //endless_flash(100, start.sec);

  pinMode(CS, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(DATA, OUTPUT);
  matrix_setup();
  backlight_setup();
  cmd_setup();
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
  int millis_start = millis();
  make_localtime(now);
  last_sec = now.sec;

  matrix_update(now.hour, now.min, now.sec);
  while(transition_state != 0) {
    delay(20);  
    matrix_update(now.hour, now.min, now.sec);
  }
  backlight_update(now.hour);

  cmd_update();
  
  int millis_to_wait = 950 - (millis() - millis_start);
  delay(millis_to_wait);
}
