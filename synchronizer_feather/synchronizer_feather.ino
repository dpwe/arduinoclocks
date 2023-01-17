// Synchronizer_feather
//
// Get time from RTC chip and GPS.
// Display relative time alignment of each (relative to millis())
// Allow synchronization to each
// Emit time sync messages over serial on demand.

// This version runs on the Adafruit feather RP2040
// with a 128x64 OLED display and 3 input buttons
// or an ESP32-S2 TFT with built-in display,

#include <Wire.h>

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

// Wiring:
//   GPS tx out                      -> D0 (for Serial1 RX)
//   int DS3231 SQWV out (pale blue) -> A1 (GP27)
//   GPS 1PPS out    (green)         -> A2 (GP28)
//   ext DS3231 SQWV out             -> A3 (GP29)
const int ledPin = 13; // On-board LED

// RP2040 Feather pins - default I2C
// Default I2C ("Wire") is actually Wire1 on RP2040 Feather
// But let's use Wire (I2C0) to communicate with the external DS3231
// You can put it on so many pins, but most have problems;
// 0/1 - 1 is used for the UART RX from the GPS
// 4/5 - 5 is used for button 0 from the display board
// 8/9 - 8 and 9 are used for buttons 1 and 2 on display board
// 12/13 - 13 is the on-board LED used to echo SQWV
// 16/17 - are not brought to pins on Feather RP2040
// 20/21 - 21 is not brought to pin on Feather RP2040
// 24/25 - were originally my interrupt inputs, but YAY we have a winner.  Interrupts move to A2/A3
//#ifdef DS3231_ON_EXTERNAL_I2C
#ifdef ARDUINO_ARCH_RP2040
const int ext_sda_pin = 24;   // Same physical pins on Feather, but different names.
const int ext_scl_pin = 25;
const int ext_sqwPin = 29; // (A3) SQWV output from external DS3231
const int int_sqwPin = 27; // (A1) SQWV output from Feather (internal) DS3231
const int ppsPin = 28; // (A2) PPS output from GPS board
#else 
const int ext_sda_pin = A4;
const int ext_scl_pin = A5;
const int ext_sqwPin = A3; // (A3) SQWV output from external DS3231
const int int_sqwPin = A1; // (A1) SQWV output from Feather (internal) DS3231
const int ppsPin = A2; // (A2) PPS output from GPS board
#endif
//#else
// If using standard Feather I2C...
//#define Wire Wire1
//#endif // DS3231_ON_EXTERNAL_I2C


// UI:
//  Top button:     Short: Swap focus between internal and external DS3231
//                  Long: Sleep display.
//  Middle button:  Short: Increase loading via Offset Register
//                  Long: Emit sync command on serial
//  Bottom button:  Short: Decrease loading via Offset Register
//                  Long: Sync DS3231 to time from GPS

// =============================================================
// PPS change time recording.
// =============================================================

volatile unsigned long int_rtc_micros = 0;
volatile unsigned long ext_rtc_micros = 0;
volatile unsigned long gps_micros = 0;

void int_rtc_mark_isr(void)
{
  int_rtc_micros = micros();
}
void ext_rtc_mark_isr(void)
{
  ext_rtc_micros = micros();
}
void gps_mark_isr(void)
{
  gps_micros = micros();
}

void setup_interrupts() {
  // DS3231 is on falling edge.
  attachInterrupt(digitalPinToInterrupt(int_sqwPin), int_rtc_mark_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(ext_sqwPin), ext_rtc_mark_isr, FALLING);
  // GPS mark is on rising edge.
  attachInterrupt(digitalPinToInterrupt(ppsPin), gps_mark_isr, RISING);
} 

// ======================================================
// Base Clock class tracks unixtime rel to millis/micros.
// ======================================================

// Ignore the first few syncs for RTC and GPS, to avoid noisy initial pulses.
#define MIN_SYNC_COUNT 10
// How many secs back to let the oldest sync go
#define CLOCK_SYNC_INTERVAL_SECS 30

class Clock {
 public:
  const char *name_ = 0;
  // base_unixtime_ corresponds to base_micros_.
  time_t base_unixtime_ = 0;
  unsigned long base_micros_ = 0;
  bool synced_ = false;
  // Calibrated system ticks per Clock's second.  
  // This is stored in tenths of microseconds to allow fractional microseconds.
  long decimicros_per_sec_ = 10000000;   // 10^7
  long nanoseconds_error_per_sec_ = 0;
  time_t measurement_period_secs_ = 0;
  bool ppm_tracking_ = false;  // is nanoseconds_error meaningful?
  // for time since the first sync seen.
  time_t oldest_sync_unixtime_ = 0;
  unsigned long oldest_sync_micros_ = 0;
  // for time since older sync.
  time_t older_sync_unixtime_ = 0;
  unsigned long older_sync_micros_ = 0;
  // Store one intermediate value so we can hop.
  time_t next_older_sync_unixtime_ = 0;
  unsigned long next_older_sync_micros_ = 0;
  // Count how many syncs we've received.
  int sync_count_ = 0;
  int min_sync_count_ = 0;

  Clock(const char *name, int min_sync_count=MIN_SYNC_COUNT) 
    : name_(name), min_sync_count_(min_sync_count) {}

  long skew_micros(Clock& ref) {
    // Report the delay in *system micros* between ref clock reaching a time and this clock reaching the same time.
    // A positive skew means this clock is behind the ref clock.
    // This only compares the last sync times of each clock, not actual "current" time.
    // If this clock is slow relative to ref clock, skew_micros will increas with time.
    // Strictly, value depends on whether we're evaluating it at this.base_unixtime_, or ref.base_unixtime_.  
    // Use ref, because ref.micros_per_sec is less likely to be wildly off.
    long micros_per_sec = (ref.decimicros_per_sec_ + 5) / 10;  // Apply rounding.
    long skew_us = ((long)(base_micros_ - ref.base_micros_)) - micros_per_sec * ((long)(base_unixtime_ - ref.base_unixtime_));
    if (skew_us < -900) {
      // Tracking bug
      Serial.print("this base_micros=");
      Serial.print(base_micros_);
      Serial.print(" unix_sec=");
      Serial.println(base_unixtime_ % 3600);
      Serial.print("ref  base_micros=");
      Serial.print(ref.base_micros_);
      Serial.print(" unix_sec=");
      Serial.println(ref.base_unixtime_ % 3600);
    }
    return skew_us;
  }

  long unixtime(void) {
    // Return current time in seconds + milliseconds.
    unsigned long micros_since_base = micros() - base_micros_;
    long seconds_since_base = (10 * micros_since_base) / decimicros_per_sec_;
    return base_unixtime_ + seconds_since_base;
  }

  void sync(time_t sync_unixtime, unsigned long sync_micros=0) {
    // Sync the clock to sync_unixtime exactly now (or at sync_micros).
    ++sync_count_;
    if (sync_micros == 0) {
      sync_micros = micros();
    }
    if (sync_count_ > min_sync_count_) {
      if (oldest_sync_micros_ == 0) {
        Serial.println("oldest_sync_micros_ 0");
        oldest_sync_micros_ = sync_micros;
        oldest_sync_unixtime_ = sync_unixtime;
        older_sync_micros_ = sync_micros;
        older_sync_unixtime_ = sync_unixtime;
        next_older_sync_micros_ = sync_micros;
        next_older_sync_unixtime_ = sync_unixtime;
      } else {
        if ((sync_unixtime - next_older_sync_unixtime_) > CLOCK_SYNC_INTERVAL_SECS) {
          // Push a (time, micros) sync pair onto the (short) queue every 2 minutes.
          older_sync_micros_ = next_older_sync_micros_;
          older_sync_unixtime_ = next_older_sync_unixtime_;
          next_older_sync_micros_ = sync_micros;
          next_older_sync_unixtime_ = sync_unixtime;
        }
        // This is going to wrap at 2**32 / 1e7 = 420 secs, or about 7 minutes.
        // Hopefully the queue will ensure it's never more than ~4 minutes.
        int delta_seconds = sync_unixtime - older_sync_unixtime_;
        if (delta_seconds > 0 && delta_seconds < 400) {
          decimicros_per_sec_ = (10 * (sync_micros - older_sync_micros_)) / 
                                delta_seconds;
        }
      }
    }
    base_micros_ = sync_micros;
    base_unixtime_ = sync_unixtime;
    synced_ = true;

    nanoseconds_error_per_sec_ = all_time_nanos_per_sec(sync_unixtime, sync_micros);
  }

  long all_time_nanos_per_sec(time_t sync_unixtime, unsigned long sync_micros) {
    // Use the oldest sync to calculate the long-time average nanos per sync.
    if (oldest_sync_micros_ == 0) {
      // we have not yet started tracking error.
      return 0;
    }
    long micros_drift = sync_micros - oldest_sync_micros_;
    measurement_period_secs_ = sync_unixtime - oldest_sync_unixtime_;
    if (measurement_period_secs_ == 0) {
      // still haven't yet collected enough data to measure.
      return 0;
    }
    // There will be wrap in the micros difference, but the residual after subtracting the seconds should be OK.
    micros_drift -= 1000000 * measurement_period_secs_;
    //Serial.print(name_);
    //Serial.print(" micros_drift=");
    //Serial.print(micros_drift);
    //Serial.print(" in ");
    //Serial.print(interval_seconds);
    //Serial.print(" seconds = ");
    int nanoseconds_error = (1000 * micros_drift) / (int)measurement_period_secs_;
    //Serial.print(nanoseconds_error);
    //Serial.println(" ns err");
    ppm_tracking_ = true;
    return nanoseconds_error;
  }

  void clear_sync_history(void) {
    // If we think there's a discontinuity in sync history.
    oldest_sync_unixtime_ = 0;
    oldest_sync_micros_ = 0;
    older_sync_unixtime_ = 0;
    older_sync_micros_ = 0;
    next_older_sync_unixtime_ = 0;
    next_older_sync_micros_ = 0;
    sync_count_ = 0;
    synced_ = false;
    nanoseconds_error_per_sec_ = 0;
    measurement_period_secs_ = 0;
    ppm_tracking_ = false;
  }
};

Clock int_rtc_clock("iRT");
Clock ext_rtc_clock("xRT");
Clock gps_clock("GPS");
Clock *sys_clock = &int_rtc_clock;

// ======================================================
// Track RTC times.
// ======================================================

#include "RTClib.h"

// from DS3231.cpp
#define CLOCK_ADDRESS 0x68

// Give up on RTC if no sync in this long.
#define MIN_RTC_SYNC_GAP 5

static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }

class DS3231_holder {
 public:
 
  RTC_DS3231 rtc;
  boolean have_rtc = false;
  TwoWire *pwire = NULL;
  int sqwPin = 0; 
  volatile unsigned long *prtc_micros = NULL;
  unsigned long last_rtc_micros = 0;
  Clock *pclock = NULL;
  int last_min = 0;
  int rtc_aging = 0;
  int rtc_temperature_centidegs = 0;

  DS3231_holder(int pin, volatile unsigned long *pmicros, Clock *p_clock) 
    : sqwPin(pin), prtc_micros(pmicros), pclock(p_clock) { 
  }

  void readAllRegisters(byte *registers, int num_bytes = 19) {
    // Read all 19 bytes from the clock status.
    if (num_bytes > 19) num_bytes = 19;
    pwire->beginTransmission(CLOCK_ADDRESS);
    pwire->write(0x0);
      pwire->endTransmission();
      pwire->requestFrom(CLOCK_ADDRESS, num_bytes);
    for(int i = 0; i < num_bytes; ++i) {
       registers[i] = pwire->read();
    }
  }
  
  int getTemperatureCentidegs(void) {
    // Read temperature register
    pwire->beginTransmission(CLOCK_ADDRESS);
    pwire->write(0x11);
    pwire->endTransmission();
    pwire->requestFrom(CLOCK_ADDRESS, 2);
    int regVal = pwire->read() << 2;
    regVal += pwire->read() >> 6;
    return 25 * regVal;
  }

  void getAlarm1(int *phour, int *pmin) const {
    // Read Alarm1 registers.
    // Alarm 1 is reg 0x07..0x0A, sec/min/hr/day
    pwire->beginTransmission(CLOCK_ADDRESS);
    pwire->write(0x08);
    pwire->endTransmission();
    pwire->requestFrom(CLOCK_ADDRESS, 2);
    *pmin = bcd2bin(pwire->read());
    *phour = bcd2bin(pwire->read());
  }

  void getAlarm2(int *phour, int *pmin) const {
    // Read Alarm2 registers.
    // Alarm 2 is reg 0x0B..0x0D, min/hr/day
    pwire->beginTransmission(CLOCK_ADDRESS);
    pwire->write(0x0B);
    pwire->endTransmission();
    pwire->requestFrom(CLOCK_ADDRESS, 2);
    *pmin = bcd2bin(pwire->read());
    *phour = bcd2bin(pwire->read());
  }

  int getAgingOffset(void) {
    // Read aging offset register
    pwire->beginTransmission(CLOCK_ADDRESS);
    pwire->write(0x10);
    pwire->endTransmission();
    pwire->requestFrom(CLOCK_ADDRESS, 1);
    int regVal = pwire->read();
    if (regVal > 127) regVal -= 256;
    return regVal;
  }

  void setAgingOffset(int offset) {
    // Write the aging register. 
    // It's a signed 8 bit value, -128..127
    // Crystal *slows* by approx 0.1 ppm per unit.
    pwire->beginTransmission(CLOCK_ADDRESS);
    pwire->write(0x10);
    pwire->write(offset);
    pwire->endTransmission();
  }

  void sync_clock_to_rtc(void) {
    // This is called soon after an A0 transition is detected, so RTC unixtime is still current.
    last_rtc_micros = *prtc_micros;
    pclock->sync(rtc.now().unixtime(), last_rtc_micros);
  }
  
  bool pending_interrupt(void) {
    unsigned long rtc_micros = *prtc_micros;
    if (rtc_micros == last_rtc_micros)  return false;
    last_rtc_micros = rtc_micros;
    return true;
  }

  void setup(TwoWire *p_wire) {
    pwire = p_wire;
    if (!rtc.begin(pwire)) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      abort();
    }
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz); // Place SQW pin into 1 Hz mode
    pinMode(sqwPin, INPUT_PULLUP); // Set alarm pin as pullup
    // Sync even without interrupts
    *prtc_micros = micros();
    sync_clock_to_rtc();
  }

  void update(time_t sys_secs) {
    if (pending_interrupt()) {
      sync_clock_to_rtc();
    }
    have_rtc = (sys_secs - pclock->base_unixtime_) < MIN_RTC_SYNC_GAP;
    if (have_rtc) {
      int sys_mins = (sys_secs / 60) % 60;
      if (sys_mins != last_min) {
        last_min = sys_mins;
        // Update temp 1/min.
        rtc_temperature_centidegs = getTemperatureCentidegs();
        rtc_aging = getAgingOffset();
      }
    }
  }

  void delta_trim(int delta) {
    // Adjust the DS3231 "aging register" trim by specified amount relative.
    rtc_aging = getAgingOffset() + delta;
    setAgingOffset(rtc_aging);
  }
};

DS3231_holder int_rtc(int_sqwPin, &int_rtc_micros, &int_rtc_clock);
DS3231_holder ext_rtc(ext_sqwPin, &ext_rtc_micros, &ext_rtc_clock);

class DS3231_holder *active_rtc = &ext_rtc;
class DS3231_holder *alt_rtc = &int_rtc;


#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);


// ======================================================
// Track GPS times.
// ======================================================

#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/

// GPS talks via secondary UART interface.  Default on RP2040, needs init on ESP32
#define SerialGPS Serial1 

TinyGPS gps;

time_t gps_now(class TinyGPS& gps) {
  tmElements_t tm;
  unsigned long date, time, age_millis;
  gps.get_datetime(&date, &time, &age_millis);
  // We *could* add age_millis to get current time, but we really want the sync'd time from the last second.
  // date and time are stored as digit pairs in decimal i.e. ddmmyy and hhmmsscc.
  int centis = time % 100;
  //Serial.print("gps centis=");
  //Serial.println(centis);
  // it's always 0.
  time /= 100;
  tm.Second = time % 100;
  time /= 100;
  tm.Minute = time % 100;
  time /= 100;
  tm.Hour = time % 100;
  tm.Year = (date % 100) + 2000 - 1970;
  date /= 100;
  tm.Month = date % 100;
  date /= 100;
  tm.Day = date % 100;
  return makeTime(tm);
}

bool gps_time_valid(class TinyGPS& gps) {
  // It's only valid if it was updated within the past second.
  unsigned long time, age_millis;
  gps.get_datetime(0, &time, &age_millis);
  return (time != gps.GPS_INVALID_TIME) && (age_millis < 1000);
}

unsigned long last_gps_micros = 0;

bool pending_GPS_interrupt(void) {
  if (gps_micros == last_gps_micros)  return false;
  last_gps_micros = gps_micros;
  return true;
}

void sync_time_from_GPS(void) {
  // This is called soon after an A1 transition is detected, so GPS unixtime is still current.
  if (gps_time_valid(gps)) {
    // We can assume that the last-stored time from the GPS messages is the second *preceeding* this mark,
    // so we add 1 second to get the actual time corresponding to the mark.
    // (potential race condition).
    gps_clock.sync(gps_now(gps) + 1, gps_micros);
    last_gps_micros = gps_micros;
  }
}

void setup_GPS_serial(void) {
#ifdef ARDUINO_ARCH_RP2040  // Needed to compile on M4
  SerialGPS.begin(9600);
#else
  SerialGPS.begin(9600, SERIAL_8N1, /* rxPin= */2, /* txPin= */1);
#endif
}

void update_GPS_serial(void) {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}

DS3231_holder* request_sync_RTC = NULL;
bool request_sync_output = false;

void setup_GPS(void) {
    pinMode(ppsPin, INPUT_PULLUP); // Set alarm pin as pullup
}

void update_GPS(void) {
  if (pending_GPS_interrupt()) {
    sync_time_from_GPS();
    // Request for sync e.g. from button press.
    if (request_sync_RTC) {
      Serial.print("Setting ");
      Serial.print(request_sync_RTC->pclock->name_);
      Serial.println(" DS3231 from GPS");
      request_sync_RTC->rtc.adjust(DateTime(gps_clock.unixtime()));
      request_sync_RTC->pclock->clear_sync_history();  // Old sync records are irrelevant now.
      request_sync_RTC = NULL;
    }
    if (request_sync_output) {
      emit_sync_command(gps_clock.unixtime());
      request_sync_output = false;
    }
  }
}

// ======================================================
// Time display
// ======================================================

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

char *sprint_int2(char *s, int n)
{  // Always 2 digits, assume n nonnegative, < 99.
  *s++ = '0' + (n / 10);
  *s++ = '0' + (n % 10);
  return s;
}

char *sprint_interval(char *s, int secs) {
  // Print an interval in compact format: 33s, 33m59, 4h33
  if (secs < 60) {
    s = sprint_int(s, secs);
    *s++ = 's';
  } else if (secs < 3600) {
    s = sprint_int(s, secs / 60);
    *s++ = 'm';
    s = sprint_int2(s, secs % 60);
  } else {
    int mins = (secs + 30) / 60;
    s = sprint_int(s, mins / 60);
    *s++ = 'h';
    s = sprint_int2(s, mins % 60);
  }
  return s;
}

char *sprint_unixtime(char *s, time_t t, bool show_date=false)
{  // Returns full terminated string
  char *entry_s = s;
  // s must provide 20 bytes, or 9 if just time.
  tmElements_t tm;
  breakTime(myTZ.toLocal(t), tm);
  if (show_date) {
    s = sprint_int(s, tm.Year + 1970);
    *s++ = '-';
    s = sprint_int2(s, tm.Month);
    *s++ = '-';
    s = sprint_int2(s, tm.Day);
    *s++ = ' ';
  }
  s = sprint_int2(s, tm.Hour);
  *s++ = ':';
  s = sprint_int2(s, tm.Minute);
  *s++ = ':';
  s = sprint_int2(s, tm.Second);
  *s++ = 0;
  return entry_s;
}

char *sprint_sync_command(char *s, time_t t) {
  // Generate a "ZYYYYMMDDHHMMSS" string that can be sent over serial to resync a flipclock.
  char *entry_s = s;
  // s must provide 16 bytes.
  tmElements_t tm;
  breakTime(t, tm);
  *s++ = 'Z';
  s = sprint_int(s, tm.Year + 1970);
  s = sprint_int2(s, tm.Month);
  s = sprint_int2(s, tm.Day);
  s = sprint_int2(s, tm.Hour);
  s = sprint_int2(s, tm.Minute);
  s = sprint_int2(s, tm.Second);
  *s++ = 0;
  return entry_s;
}

void emit_sync_command(time_t t) {
  char s[16];
  Serial.println(sprint_sync_command(s, t));
}

void serial_print_unixtime(time_t t) {
  char s[20];
  sprint_unixtime(s, t);
  Serial.print(s);
}

char *sprint_clock_comparison(char *s, class Clock& clock, class Clock& ref_clock)
{  // Returns full terminated string
  char *entry_s = s;
  // Report skew in ms with 1 dp.
  s = sprint_int(s, clock.skew_micros(ref_clock) / 100, /* dp */ 1);
  strcpy(s, "ms ");
  s += strlen(s);
  // Approximate (1 + a)/(1 + b) - 1 as (a - b) (i.e. 1/(1 + b) = 1 - b, and a/(1 + b) = a), for a, b << 1.
  //int decimicro_deviation = ref_clock.decimicros_per_sec_ - clock.decimicros_per_sec_;
  //s = sprint_int(s, decimicro_deviation, /* dp */ 1);
  //strcpy(s, "ppm ");
  //s += strlen(s);
  if (clock.ppm_tracking_) {
    Serial.print(clock.name_);
    Serial.print(" ppm_tracking: ");
    Serial.println(clock.nanoseconds_error_per_sec_);
    // A positive ppm means ref clock has more nanosecs per tick than comparison clock
    // i.e. comparison clock is running fast (and so aging register loading should be
    // increased).
    s = sprint_int(s, ref_clock.nanoseconds_error_per_sec_ - clock.nanoseconds_error_per_sec_, /* dp */ 3);
    strcpy(s, "ppm/");
    s += strlen(s);
    s = sprint_interval(s, clock.measurement_period_secs_);
  }
  *s++ = 0;
  return entry_s;
}

char *sprint_rtc_info(char *s, const class DS3231_holder &rtc) {
  // Read RTC info from globals.
  char *entry_s = s;
  //strcpy(s, "RTC: ");
  //s += strlen(s);
  if (!rtc.have_rtc) {
    strcpy(s, "no RTC");
    return entry_s;
  }
  strcpy(s, "t=");
  s += strlen(s);
  s = sprint_int(s, rtc.rtc_temperature_centidegs/10, 1);
  strcpy(s, " a=");
  s += strlen(s);
  s = sprint_int(s, rtc.rtc_aging);
  strcpy(s, " A2=");
  s += strlen(s);
  int h, m;
  rtc.getAlarm2(&h, &m);
  s = sprint_int2(s, h);
  *s++ = ':';  
  s = sprint_int2(s, m);
  *s++ = ' ';   // Trailing space to overwrite cruft.
  *s++ = 0;
  return entry_s;
}

void serial_print_clock_comparison(class Clock& clock, class Clock& ref_clock) {
  char s[64];
  sprint_clock_comparison(s, clock, ref_clock);
  Serial.print(s);
}

void serial_clock_debug(class Clock* pclock) {
    char s[9];
  Serial.print(" ");
  Serial.print(pclock->name_);
  Serial.print(": ");
  serial_print_unixtime(pclock->unixtime());
  Serial.print(" dmspt=");
  Serial.print(pclock->decimicros_per_sec_);
  Serial.print(" syncs=");
  Serial.print(pclock->sync_count_);
  Serial.print(" sync=");
  sprint_unixtime(s, pclock->base_unixtime_, false);
  Serial.print(s);
  Serial.print(" osut=");
  sprint_unixtime(s, pclock->older_sync_unixtime_, false);
  Serial.print(s);
}


#define NUM_REGISTERS 19

void serial_print_registers(void) {
  // Read all 19 DS3231 registers and print out in hex.
  Serial.print("DS3231 Regs: ");
  byte registers[NUM_REGISTERS];
  active_rtc->readAllRegisters(registers, NUM_REGISTERS);
  for (int i = 0; i < NUM_REGISTERS; ++i) {
    if (registers[i] < 16)  Serial.print("0");
    Serial.print(registers[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}


// ======================================================
// =========== OLED Display ==============
// ======================================================

#include <SPI.h>
#include <Adafruit_GFX.h>

#include <Adafruit_GFX.h>

#ifdef ARDUINO_ARCH_RP2040  // Needed to compile on M4
  #define DISPLAY_SH1107  // 128x64 mono OLED in Feather stack
  // Wire1 is the internal I2C on Feather RP2040
  #define WIRE_INTERNAL Wire1
  #define WIRE_EXTERNAL Wire
#else
  #define DISPLAY_ST7789  // Built-in display on ESP32-S3 TFT
  //#define DISPLAY_SSD1351  // Exernal 128x128 RGB TFT
  #define WIRE_INTERNAL Wire
  #define WIRE_EXTERNAL Wire1
#endif

#ifdef DISPLAY_SSD1351
  // Arduino - Expect SQWV input on D3
  const uint8_t sqwvPin = 3;

  #include <Adafruit_SSD1351.h>
  // Screen dimensions
  #define SCREEN_WIDTH  128
  #define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.
  #define SIZE_1X

  // Hardware SPI pins 
  // (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
  // an output. 
  #define DC_PIN   4
  #define CS_PIN   5
  #define RST_PIN  6
  Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

  // Color definitions
  #define BLACK           0x0000
  #define BLUE            0x001F
  #define RED             0xF800
  #define GREEN           0x07E0
  #define CYAN            0x07FF
  #define MAGENTA         0xF81F
  #define YELLOW          0xFFE0  
  #define WHITE           0xFFFF
#endif

#ifdef DISPLAY_ST7789
  // ESP32-S2/3 TFT
  
  #include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

  #define SCREEN_WIDTH  240
  #define SCREEN_HEIGHT 135 // Change this to 96 for 1.27" OLED.
  #define SIZE_2X  // All text double-size

  // Use dedicated hardware SPI pins
  Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

  const int backlightPin = TFT_BACKLITE;  // PWM output to drive dimmable backlight

  #define WHITE ST77XX_WHITE
  #define BLACK ST77XX_BLACK
  #define BLUE  ST77XX_BLUE
  #define RED   ST77XX_RED
  #define GREEN ST77XX_GREEN
  #define CYAN  ST77XX_CYAN
  #define MAGENTA ST77XX_MAGENTA
  #define YELLOW  ST77XX_YELLOW 
#endif

#ifdef DISPLAY_SH1107
  // Feather (RP2040) stack

  #include <Adafruit_SH110X.h>
  
  #define SCREEN_WIDTH  128
  #define SCREEN_HEIGHT 64
  #define SIZE_1X

  Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, &WIRE_INTERNAL);

  // Monochrome, all colors are white
  #define WHITE SH110X_WHITE
  #define BLACK SH110X_BLACK
  #define BLUE  WHITE
  #define RED   WHITE
  #define GREEN WHITE
  #define CYAN  WHITE
  #define MAGENTA WHITE
  #define YELLOW  WHITE 

#endif

bool display_on = false;

void setup_display(void) {
#ifdef DISPLAY_SSD1351
  display.begin();
#endif
#ifdef DISPLAY_ST7789
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  analogWrite(TFT_BACKLITE, 128);

  display.init(135, 240); // Init ST7789 240x135
  display.setRotation(3);
#endif
#ifdef DISPLAY_SH1107
  display.begin(0x3C, true); // Address 0x3C default
  display.display();  // Splashscreen
  delay(500);
  display.clearDisplay();
  display.display();
  display.setRotation(1);
#endif

  display.fillScreen(BLACK);

  // text display 
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,0);
  display.print("synchronizer_feather");

  display_on = true;
}

#ifdef SIZE_1X
  // 1x size
  #define SMALL_SIZE 1
  #define LARGE_SIZE 2
  #define ROW_H 8
  #define CHAR_W 6
#else
  // 2x size
  #define SMALL_SIZE 2
  #define LARGE_SIZE 4
  #define ROW_H 16
  #define CHAR_W 12
#endif

void wake_up_display(void) {
  display_on = true;
  update_display(active_rtc);
}

void sleep_display(void) {
#ifdef DISPLAY_SH1107
  display.clearDisplay();
  display.display();
#else
  display.fillScreen(BLACK);
#endif
  display_on = false;
}

void update_display(class DS3231_holder *prtc) {
  if (!display_on) {
    return;
  }
  char s[64];  // We can only use up to 21, but sprint_clock_comparison could get large for very large skews.
  // Line 1: Current time, sync source.
#ifdef DISPLAY_SH1107
  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
#endif
  display.setCursor(0, 0);
  display.setTextSize(LARGE_SIZE);
  display.print(sprint_unixtime(s, (*sys_clock).unixtime(), false));
  display.setCursor(17 * CHAR_W, 0);
  display.setTextSize(SMALL_SIZE);
  display.print(sys_clock->name_);
  // Line 2: RTC vs GPS.
  display.setCursor(0, 4 * ROW_H);
  display.print(prtc->pclock->name_);
  display.print(":");
  display.setCursor(0, 5 * ROW_H);
  display.print(sprint_clock_comparison(s, *(prtc->pclock), *sys_clock));
  // Line 3: RTC status
  display.setCursor(0, 6 * ROW_H);
  display.print(sprint_rtc_info(s, *prtc));

#ifdef DISPLAY_SH1107
  display.display();
#endif
}

// ======================================================
// =========== Button management ==============
// ======================================================

#define NUM_BUTTONS 3   // on D9, D6, D5 on feather OLED wing are GPIO 9, 8, 7 on RP2040 Feather
#ifdef ARDUINO_ARCH_RP2040  // Needed to compile on M4
int button_pins[NUM_BUTTONS] = {9, 8, 7};
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

void buttons_setup(time_t t) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    pinMode(button_pins[button], INPUT_PULLUP);
  }
  secs_last_action = t;
}

void swap_rtcs(void) {
  // Swap RTCs
  class DS3231_holder* tmp = active_rtc;
  active_rtc = alt_rtc;
  alt_rtc = tmp;
  update_display(active_rtc);
}

void buttons_update(time_t t) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    // Buttons are pulled up, so read as 1 when open, 0 when pressed.
    int new_state = 1 - digitalRead(button_pins[button]);
    if (button_state[button] == new_state) {
      continue;
    }
    // State has changed.
    secs_last_action = t;  // Reset sleep display timeout.
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
    if (!display_on) {
      // Any keypress when display is off just wakes up display.
      wake_up_display();
      return;
    }
    switch(button) {
      case 0:
        if (long_press) {
          sleep_display();
        } else {
          swap_rtcs();
        }
        break;
      case 1:
        // Increase aging register
        if (long_press) {
          // Emit a sync command.
          request_sync_output = true;
        } else {
          active_rtc->delta_trim(1);
          update_display(active_rtc);
        }
        break;
      case 2:
        if (long_press) {
          // Write GPS time to RTC at next good opportunity.
          request_sync_RTC = active_rtc;
        } else {
          // Decrease aging register
          active_rtc->delta_trim(-1);
          update_display(active_rtc);
        }
        break;
    }
  }
}

// -------------------------------------------------------------------
// Input commands over serial line
// -------------------------------------------------------------------

void cmd_setup(void) {
  // Nothing to do?
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(time_t t) {
  if (Serial.available() > 0) {
    secs_last_action = t;  // Reset sleep display timeout.
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a' && cmd0 <= 'z')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case '?':
            Serial.println("Y    - sYnc DS3231 to GPS");
            Serial.println("R    - print DS3231 Registers");
            Serial.println("S    - Swap DS3231 internal/external");
            Serial.println("Axx  - Set Aging Offset -128 to 127");
            Serial.println("D    - Dim screen");
            break;
          case 'Y':
            // Sync DS3231 to GPS
            request_sync_RTC = active_rtc;
            break;
          case 'R':
            // Read DS3231 registers
            serial_print_registers();
            break;
          case 'S':
            // Swap RTC displayed.
            swap_rtcs();
            break;
          case 'D':
            // Dim screen
            if (display_on) {
              sleep_display();
            } else {
              wake_up_display();
            }
            break;
          case 'A':
            // Set DS3231 aging register.
            int aging_offset = atoi(cmd_buffer + 1);
            active_rtc->setAgingOffset(aging_offset);
            Serial.print("Aging offset set to ");
            Serial.println(aging_offset);
            update_display(active_rtc);
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

// ======================================================
// Main setup() and loop()
// ======================================================

bool serial_available = false;

#define MAXWAIT_SERIAL 200  // 200 = 2 seconds.

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
  //delay(500);
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  open_serial();
  
  Serial.print("synchronizer_feather ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.println("Post-reset settling...");
  delay(1000);
  digitalWrite(ledPin, LOW);

#ifdef ARDUINO_ARCH_RP2040
  // Configure Pico RP2040 I2C
  WIRE_EXTERNAL.setSDA(ext_sda_pin);
  WIRE_EXTERNAL.setSCL(ext_scl_pin);
  WIRE_EXTERNAL.begin();
#else
  Serial.println("Beginning Wire...");
  WIRE_EXTERNAL.begin(ext_sda_pin, ext_scl_pin);
  WIRE_INTERNAL.begin();
#endif
  // I2C interface is started by display driver?
  //Wire1.begin();

  Serial.println("Display setup...");
  setup_display();  // includes i2c init.
  Serial.println("int_rtc setup...");
  int_rtc.setup(&WIRE_INTERNAL);
  Serial.println("ext_rtc setup...");
  ext_rtc.setup(&WIRE_EXTERNAL);
  Serial.println("gps_serial setup...");
  setup_GPS_serial();
  Serial.println("gps setup...");
  setup_GPS();
  Serial.println("buttons setup...");
  buttons_setup(sys_clock->unixtime());
  Serial.println("cmd setup...");
  cmd_setup();

  Serial.println("interrupts setup...");
  setup_interrupts();
  Serial.println("Setup done.");
};

// How long can we miss syncs before we give up on GPS clock?
#define GPS_TIMEOUT_SECS 5

// How long before screen dims?
#define DISPLAY_SLEEP_SECS 300
time_t last_action = 0;

boolean gps_has_been_live = false;

time_t secs_last_change = 0;

void loop() {
  time_t sys_secs = sys_clock->unixtime();

  digitalWrite(ledPin, digitalRead(active_rtc->sqwPin));

  buttons_update(sys_secs);
  cmd_update(sys_secs);

  int_rtc.update(sys_secs);
  ext_rtc.update(sys_secs);
  update_GPS();
  if(gps_clock.synced_ && 
    (!gps_has_been_live || (((long)gps_clock.base_unixtime_) - sys_secs) < GPS_TIMEOUT_SECS)) {
    // GPS is sync'ing, and it has been sync'd recently, or it's the first time we've seen it sync'd
    // (in which case the difference from sys_secs may be immaterial).
    sys_clock = &gps_clock;
    gps_has_been_live = true;
  } else {
    sys_clock = active_rtc->pclock;
  }
  update_GPS_serial();
  if ((secs_last_change != sys_secs)) {
    secs_last_change = sys_secs;
    serial_clock_debug(&gps_clock);
    serial_clock_debug(int_rtc.pclock);
    serial_clock_debug(ext_rtc.pclock);
    Serial.println();
    // And on display:
    update_display(active_rtc);
  }
  // Maybe sleep display
  if (secs_last_action && (sys_secs - secs_last_action) > DISPLAY_SLEEP_SECS) {
    sleep_display();
  }
}
