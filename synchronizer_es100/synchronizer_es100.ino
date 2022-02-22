// Synchronizer_es100
//
// Get time from RTC chip, ES100/WWVB, and GPS.
// Display relative time alignment of each (relative to millis())
// Allow synchronization to each
// Emit time sync messages over serial on demand.

// This version actually runs on the ES100 DevBoard.

// Designed to run on the Canaduino ES100 DevBoard, 
// i.e. with a 4x20 display and 3 input buttons.

// =============================================================
// Port C change time recording.
// =============================================================

// Wiring:
//   DS3231 SQWV out -> A0 pin on Arduino (for interrupt inputs)
//   GPS 1PPS out    -> A1 pin
//   ES100 INT out   -> A2 pin ?  Or use existing D2

//   GPS tx out      -> D8 (for AltSoftSerial) or A2? (on ES100 board with no D pins)

// UI:
//  Top button:     Sync DS3231 to time from GPS
//  Middle button:  
//  Bottom button:  (Cycle display?)


#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
#include "RTClib.h"

RTC_DS1307 ds1307;
#ifdef TRY_DS3231
RTC_DS3231 ds3231;
#endif

volatile unsigned long A0_micros = 0;
volatile unsigned long A1_micros = 0;
volatile unsigned long A2_micros = 0;

volatile byte last_portc_state = 0;

// Record time of last edges.
unsigned long last_A0_micros = 0;
unsigned long last_A1_micros = 0;
unsigned long last_A2_micros = 0;


// ======================================================
// Base Clock class tracks unixtime rel to millis/micros.
// ======================================================

// Ignore the first few syncs for RTC and GPS, to avoid noisy initial pulses.
#define MIN_SYNC_COUNT 4  
// How many secs back to let the oldest sync go
#define OLDEST_CLOCK_SYNC_SECS 30


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
#ifdef ROBUST_MEAN
  class RobustMean robust_mean;
  int sync_interval_secs_ = 0;
#else
  // for time since oldest sync.
  time_t oldest_sync_unixtime_ = 0;
  unsigned long oldest_sync_micros_ = 0;
  // Store one intermediate value so we can hop.
  time_t next_oldest_sync_unixtime_ = 0;
  unsigned long next_oldest_sync_micros_ = 0;
#endif
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
    return (base_micros_ - ref.base_micros_) - micros_per_sec * (base_unixtime_ - ref.base_unixtime_);
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
#ifdef ROBUST_MEAN
    int this_sync_interval = sync_unixtime - base_unixtime_;
    if (this_sync_interval == sync_interval_secs_) {
      // Successive syncs.
      robust_mean.insert_value(10 * (sync_micros - base_micros_));
      decimicros_per_sec_ = robust_mean.mean() / sync_interval_secs_;
    }
    sync_interval_secs_ = this_sync_interval;
#else
    if (sync_count_ > min_sync_count_) {
      if (oldest_sync_micros_ == 0) {
        oldest_sync_micros_ = sync_micros;
        oldest_sync_unixtime_ = sync_unixtime;
        next_oldest_sync_micros_ = sync_micros;
        next_oldest_sync_unixtime_ = sync_unixtime;
      } else {
        if ((sync_unixtime - next_oldest_sync_unixtime_) > OLDEST_CLOCK_SYNC_SECS) {
          // Push a (time, micros) sync pair onto the (short) queue every 2 minutes.
          oldest_sync_micros_ = next_oldest_sync_micros_;
          oldest_sync_unixtime_ = next_oldest_sync_unixtime_;
          next_oldest_sync_micros_ = sync_micros;
          next_oldest_sync_unixtime_ = sync_unixtime;
        }
        // This is going to wrap at 2**32 / 1e7 = 420 secs, or about 7 minutes.
        // Hopefully the queue will ensure it's never more than ~4 minutes.
        int delta_seconds = sync_unixtime - oldest_sync_unixtime_;
        if (delta_seconds < 400) {
          decimicros_per_sec_ = (10 * (sync_micros - oldest_sync_micros_)) / 
                                delta_seconds;
        }
      }
    }
#endif
    base_micros_ = sync_micros;
    base_unixtime_ = sync_unixtime;
    synced_ = true;
  }

  void clear_sync_history(void) {
    // If we think there's a discontinuity in sync history.
#ifdef ROBUST_MEAN
    sync_interval_secs_ = 0;
#else
    oldest_sync_unixtime_ = 0;
    oldest_sync_micros_ = 0;
    next_oldest_sync_unixtime_ = 0;
    next_oldest_sync_micros_ = 0;
#endif
    sync_count_ = 0;
    synced_ = false;
  }

};

Clock rtc_clock("RTC");
Clock gps_clock("GPS");
Clock atx_clock("ATX", /* min_sync_count */ 0);
Clock *sys_clock = &rtc_clock;

// ======================================================
// Track RTC times.
// ======================================================

#define CLOCK_ADDRESS 0x68

void write_clock_addr(byte addr, byte value) {
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(addr);
  Wire.write(value);
  Wire.endTransmission(); 
}

byte read_clock_addr(byte addr) {
  byte value;
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(CLOCK_ADDRESS, 1);
  value = Wire.read();
  return value;
}

#ifdef TRY_DS3231

bool check_for_ds3231(RTC_DS3231 &ds3231) {
    // On ds3231, register 0x12 is the LSB of the temp, and the bottom 6 bits are zero.
    // See if we can write something else into it.
    const byte ds3231_temp_lsb_reg = 0x12;
    const byte test_val = 0xAA;
    write_clock_addr(ds3231_temp_lsb_reg, test_val);
    byte value = read_clock_addr(ds3231_temp_lsb_reg);
    if (value == test_val) {
      Serial.println("NOT ds3231");
      return false;
    } else {
      Serial.println("IS ds3231");
      // Value was modified, assume bottom 6 bits were cleared to zero.
      return true;
    }
}

bool is_ds3231 = false;
#endif // TRY_DS3231

bool pending_RTC_interrupt = false;

void sync_time_from_RTC(void) {
  // This is called soon after an A0 transition is detected, so RTC unixtime is still current.
  time_t now_time;
#ifdef TRY_DS3231
  if (is_ds3231) now_time = ds3231.now().unixtime();
  else 
#endif
    now_time = ds1307.now().unixtime();
  rtc_clock.sync(now_time, last_A0_micros);
}

void setup_RTC(void) {
  // Make output pin be 1PPS (frequency 0) (drives A0 for interrupts).
  ds1307.begin();
#ifdef TRY_DS3231
  if (check_for_ds3231(ds3231)) {
    ds3231.writeSqwPinMode(DS3231_SquareWave1Hz);
    is_ds3231 = true;
  } else
#endif // TRY_DS3231
    ds1307.writeSqwPinMode(DS1307_SquareWave1HZ);
  // Grab a time value even before we see an interrupt, simulate setting the micros.
  last_A0_micros = micros();
  sync_time_from_RTC();
}

void adjust_RTC(time_t epoch) {
  DateTime dt(epoch);
#ifdef TRY_DS3231
  if (is_ds3231) ds3231.adjust(dt);
  else
#endif // TRY_DS3231
    ds1307.adjust(dt);
}

void update_RTC(void) {
  if (pending_RTC_interrupt) {
    sync_time_from_RTC();
    pending_RTC_interrupt = false;
  }
}


//ISR(PCINT1_vect)   // Port C change interrupt (analog pins).
// To be compatible with SoftwareSerial, which monopolizes the pin change interrupts,
// we write this as a separate handler which is inserted into a hook added to the
// SoftwareSerial interrupt handler.
void dpwe_portc_isr(void)
{
  long micros_now = micros();
  byte portc_state = PINC & 0b00000111;
  // Check the three input pins, recording times of low to high transitions..
  byte bits_gone_high = (portc_state ^ last_portc_state) & portc_state;
  byte bits_gone_low = (portc_state ^ last_portc_state) & (~portc_state);
  // DS3231 mark is on falling edge.
  if (bits_gone_low & 0b00000001) {
    A0_micros = micros_now;
  }
  // GPS mark is on rising edge.
  if (bits_gone_high & 0b00000010) {
    A1_micros = micros_now;
  }
  // ES100 mark is on falling edge.
  if (bits_gone_low & 0b00000100) {
    A2_micros = micros_now;
  }
  last_portc_state = portc_state;
} 

#include <SoftwareSerial.h>

void setup_portc_change_interrupt() {
  // Setup interrupts on port C, which controls the analog lines.
  // A0, A1, A2 as interrupt inputs.
  // Following https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/ .
  cli();
  // Set A0, A1, A2 as inputs.
  DDRC &= 0b11111000;
  // Turn on pin change interrupts for port C.
  PCICR |= 0b00000010;    
  // Turn on pin PC0, which is A0.
  PCMSK1 |= 0b00000001;    
  // Turn on pin PC1, which is A1.
  PCMSK1 |= 0b00000010;    
  // Turn on pin PC2, which is A2.
  PCMSK1 |= 0b00000100;    
  sei();

  // Insert the hook into SoftwareSerial.
  SoftwareSerial::set_extern_pc_isr(dpwe_portc_isr);
}

// ======================================================
// Port C change detection
// ======================================================

int update_portc_change_times() {
  // Check for times being set by 
  int change = 0;
  
  if (last_A0_micros != A0_micros) {
    last_A0_micros = A0_micros;
    change |= 0x01;
  }
  if (last_A1_micros != A1_micros) {
    last_A1_micros = A1_micros;
    change |= 0x02;
  }
  if (last_A2_micros != A2_micros) {
    last_A2_micros = A2_micros;
    change |= 0x04;
  }
  return change;
}

// =======================
// Pseudo median tracking
// =======================

#ifdef ROBUST_MEAN

// We get a series of estimates of the micros-per-tick for each clock.
// But there are sometimes outliers that mess up a simple average.

// During warm up, store them all until the buffer is full.
// Then only update with those within 1% of the current average.

// Must be odd, so there's a unique middle bin
#define ROBUST_MEAN_BINS 7
// Middle bin.  7 bins -> median is values_[3]
#define ROBUST_MEAN_MIDDLE ((ROBUST_MEAN_BINS - 1) / 2)
// How often to subtract a value
#define ROBUST_MEAN_DECAY_EVERY 16
// Don't decay until one of the peaks is at least this big.
#define ROBUST_MEAN_MIN_MAX 4
// When decaying, if peak count is this big, slash all counts in half.
#define ROBUST_MEAN_MAX_MAX 64

// Strategy: Accept new value, then drop extremum furthest from median.
// So, insert into list, making an N+1 point list, then decide which end to drop.

int insert_into_counted_sorted_list(unsigned long value, unsigned long values[], int counts[], int nitems) {
  // values[], counts[] are nitems + 1 size, but only firsts nitems are filled.
  // If value matches an existing values[], increment corresponding count.
  // Otherwise insert value at a sorted position, set count to 1, and extend list to length nitems + 1.
  // Return the new nitems.
  int i, j;
  for (i = 0; i < nitems; ++i) {
    if (value == values[i]) {
      // Duplicate value - increment count, wait.
      counts[i]++;
      return nitems;
    }
    if (value < values[i])  break;
  }
  // Shift down from i.
  for (j = nitems; j > i; --j) {
    values[j] = values[j - 1];
    counts[j] = counts[j - 1];
  }
  // New value.
  values[i] = value;
  counts[i] = 1;
  return nitems + 1;
}

class RobustMean {
 public:
  unsigned long values_[ROBUST_MEAN_BINS + 1];
  int counts_[ROBUST_MEAN_BINS + 1];
  int values_count_ = 0;
  int since_last_decay_ = 0;

  unsigned long mean(void) {
    if (values_count_ == 0)  return 0L;
    unsigned long values_sum = 0;
    int counts_sum = 0;
    for (int i = 0; i < values_count_; ++i) {
      int count = counts_[i] - 1;  // Discount values with single count (outliers).
      if (count > 0) {
        values_sum += count * values_[i];
        counts_sum += count;
      }
    }
    if (counts_sum == 0){
      return 0;
    }
    return (values_sum + counts_sum / 2) / counts_sum;
  }
  
  void insert_value(unsigned long value) {
    values_count_ = insert_into_counted_sorted_list(value, values_, counts_, values_count_);
    if (values_count_ > ROBUST_MEAN_BINS) {
      // We need to drop first or last value.
      unsigned long median = (values_[ROBUST_MEAN_MIDDLE] + values_[ROBUST_MEAN_MIDDLE + 1]) / 2;
      if ((median - values_[0]) > (values_[ROBUST_MEAN_BINS] - median)) {
        // lowest bin is further from median, so shift down.
        for (int i = 0; i < ROBUST_MEAN_BINS; ++i) {
          values_[i] = values_[i + 1];
          counts_[i] = counts_[i + 1];
        }
      }
      // Drop the final value.
      values_count_--;
    }
    // Maybe decay.
    since_last_decay_ += 1;
    if (since_last_decay_ > ROBUST_MEAN_DECAY_EVERY) {
      decay();
      since_last_decay_ = 0;
    }
  }

  void decay(void) {
    // Periodically reduce the counts of all the items to avoid holding on to outliers for ever.
    // Only do this if the largest value is not going to be wiped out by this.
    int max_count = 0;
    for (int i = 0; i < values_count_; ++i) {
      if (counts_[i] > max_count) {
        max_count = counts_[i];
      }
    }      
    if (max_count < ROBUST_MEAN_MIN_MAX) {
      // Don't decay if we don't have a peak.
      return;
    }
    for (int i = 0; i < values_count_; ++i) {
      if (max_count >= ROBUST_MEAN_MAX_MAX) {
        // Trim all the counts so things don't get numerically too large.
        counts_[i] /= 2;  // Will also eliminate counts of 1.
      } else {
        counts_[i] -= 1;
      }
    }
    // Close up gaps.
    int i = 0;
    while(i < values_count_) {
      if (counts_[i] > 0) {
        i++;
      } else {
        // Empty bin - copy down rest of list on top of it.
        for(int j = i; j < values_count_; ++j) {
          values_[j] = values_[j + 1];
          counts_[j] = counts_[j + 1];
        }
        values_count_--;
      }
    }
  }

  void print(void) {
    Serial.print("RobustMean values_count=");
    Serial.print(values_count_);
    Serial.print(" mean=");
    Serial.print(mean());
    Serial.print(" since_decay=");
    Serial.print(since_last_decay_);
    Serial.println();
    for (int i = 0; i < values_count_; ++i) {
      Serial.print(" ");
      Serial.print(values_[i]);
      Serial.print("x");
      Serial.print(counts_[i]);
    }
    Serial.println();
  }
};

#endif // ROBUST_MEAN

// ======================================================
// Track GPS times.
// ======================================================

#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/

// AltSoftSerial - Only RX on Pin 8, so can't work with ES100 devboard
//#include <AltSoftSerial.h> // https://github.com/PaulStoffregen/AltSoftSerial
//AltSoftSerial SerialGPS;   // Teensy: rx pin 20, tx pin 21.  Nano: rx 8, tx 9

// SoftwareSerial - will work with any pin, blocks, some linking problem.
//#include <SoftwareSerial.h>   // included above for portc interrupt handler.
SoftwareSerial SerialGPS = SoftwareSerial(A3, 12);  // receive on pin A3

TinyGPS gps;

bool pending_GPS_interrupt = false;

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

void sync_time_from_GPS(void) {
  // This is called soon after an A1 transition is detected, so GPS unixtime is still current.
  if (gps_time_valid(gps)) {
    // We can assume that the last-stored time from the GPS messages is the second *preceeding* this mark,
    // so we add 1 second to get the actual time corresponding to the mark.
    // (potential race condition).
    gps_clock.sync(gps_now(gps) + 1, last_A1_micros);
  }
}

void setup_GPS_serial(void) {
  SerialGPS.begin(9600);
}

void update_GPS_serial(void) {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}

bool request_RTC_sync = false;
bool request_sync_output = false;

void update_GPS(void) {
  if (pending_GPS_interrupt) {
    sync_time_from_GPS();
    pending_GPS_interrupt = false;
    // Request for sync e.g. from button press.
    if (request_RTC_sync) {
      Serial.println("GPS->RTC");
      adjust_RTC(gps_clock.unixtime());
      rtc_clock.clear_sync_history();  // Old sync records are irrelevant now.
      request_RTC_sync = false;
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


char *sprint_unixtime(char *s, time_t t, bool show_date=false)
{  // Returns full terminated string
  char *entry_s = s;
  // s must provide 20 bytes, or 9 if just time.
  tmElements_t tm;
  breakTime(t, tm);
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
  int decimicro_deviation = ref_clock.decimicros_per_sec_ - clock.decimicros_per_sec_;
  s = sprint_int(s, decimicro_deviation, /* dp */ 1);
  strcpy(s, "ppm");
  return entry_s;
}

void serial_print_clock_comparison(class Clock& clock, class Clock& ref_clock) {
  char s[64];
  sprint_clock_comparison(s, clock, ref_clock);
  Serial.print(s);
}

void serial_clock_debug(class Clock& clock) {
    char s[9];
  Serial.print(" ");
  Serial.print(clock.name_);
  Serial.print(": ");
  serial_print_unixtime(clock.unixtime());
  Serial.print(" dmspt=");
  Serial.print(clock.decimicros_per_sec_);
  Serial.print(" syncs=");
  Serial.print(clock.sync_count_);
  Serial.print(" sync=");
  sprint_unixtime(s, clock.base_unixtime_, false);
  Serial.print(s);
  Serial.print(" osut=");
  sprint_unixtime(s, clock.oldest_sync_unixtime_, false);
  Serial.print(s);
}

// ======================================================
// =========== Button management ==============
// ======================================================

#define NUM_BUTTONS 3   // on D3, D6, D7
int button_pins[NUM_BUTTONS] = {3, 6, 7};
int button_state[NUM_BUTTONS] = {0, 0, 0};
unsigned long button_last_change_time[NUM_BUTTONS] = {0, 0, 0};
#define DEBOUNCE_MILLIS 50

void buttons_setup(void) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    pinMode(button_pins[button], INPUT_PULLUP);
  }
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
    if (!new_state) {
      continue;
    }
    // State change was to "pressed".
    switch(button) {
      case 0:
        // Write GPS time to RTC at next good opportunity.
        request_RTC_sync = true;
        break;
      case 1:
        // Emit a sync command.
        request_sync_output = true;
        break;
      case 2:
        break;
    }
  }
}

// ======================================================
// ============= ES100 management ==============
// ======================================================

#include <ES100.h>
#define es100Int 2
#define es100En 13
ES100 es100;

volatile unsigned long lastInterruptMicros = 0;
unsigned long lastSyncMicros = 0;

volatile unsigned int interruptCnt = 0;
unsigned int lastinterruptCnt = 0;

boolean rx_active = false;        // variable to determine if we are in receiving mode
boolean rx_request = true;        // variable to trigger the reception.  Initialize to true so reception starts on boot.
boolean continuous = true;        // variable to tell the system to continuously receive atomic time, if not it will happen every night at midnight
boolean validdecode = false;      // variable to rapidly know if the system had a valid decode done lately

void ES100Interrupt() {
  // Called procedure when we receive an interrupt from the ES100
  // Got a interrupt and store the current ticks for future use if we have a valid decode
  lastInterruptMicros = micros();
  interruptCnt++;
}

void ES100_setup() {
  es100.begin(es100Int, es100En);  
  attachInterrupt(digitalPinToInterrupt(es100Int), ES100Interrupt, FALLING);
}

// Globals to hold state of last ES100 readout.
ES100Status0  ES100_status0;
ES100NextDst  ES100_nextDst;

void ES100_update() {
  // Do we need to start a new request?
  if (!rx_active && rx_request) {
    interruptCnt = 0;
    
    es100.enable();
    es100.startRx();
    
    rx_active = true;
    rx_request = false;

    /* Important to set the interrupt counter AFTER the startRx because the es100 
     * confirms that the rx has started by triggering the interrupt. 
     * We can't disable interrupts because the wire library will stop working
     * so we initialize the counters after we start so we can ignore the first false 
     * trigger
     */
    lastinterruptCnt = 0;
    interruptCnt = 0;
  }

  // Has there been a new interrupt?
  if (lastinterruptCnt < interruptCnt) {
    Serial.print("ES100 Interrupt received... ");
  
    if (es100.getIRQStatus() == 0x01 && es100.getRxOk() == 0x01) {
      validdecode = true;
      Serial.println("Valid decode");
      // Update lastSyncMicros for lcd display
      lastSyncMicros = micros();
      // We received a valid decode
      tmElements_t d = es100.getDateTime();
      // Updating the RTC
      //rtc.setDate(d.day, d.month, 2000+d.year);
      // We assume (safely, I think) that it's within one second of interrupt, so the
      // recorded time is still (close to) "now".
      Serial.println("Micros from interrupt to RTC set: ");
      Serial.print(lastSyncMicros - lastInterruptMicros);
      //rtc.setTime(d.hour, d.minute, d.second);
      atx_clock.sync(makeTime(d), lastInterruptMicros);
      serial_clock_debug(atx_clock);
      Serial.println();
      serial_print_clock_comparison(atx_clock, gps_clock);
      Serial.println();
      
      // Get everything before disabling the chip.
      ES100_status0 = es100.getStatus0();
      ES100_nextDst = es100.getNextDst();
      /* DEBUG */
      Serial.print("status0.rxOk = B");
      Serial.println(ES100_status0.rxOk, BIN);
      Serial.print("status0.antenna = B");
      Serial.println(ES100_status0.antenna, BIN);
      Serial.print("status0.leapSecond = B");
      Serial.println(ES100_status0.leapSecond, BIN);
      Serial.print("status0.dstState = B");
      Serial.println(ES100_status0.dstState, BIN);
      Serial.print("status0.tracking = B");
      Serial.println(ES100_status0.tracking, BIN);
      /* END DENUG */
  
      if (!continuous) {
        //es100.stopRx();   // Unneeded - stops automatically after IRQ.
        es100.disable();
        rx_active = false;
      } else {
        es100.startRx();   // Immediately initiate another sync.
      }
    }
    else {
      Serial.println("Invalid decode");
    }
    lastinterruptCnt = interruptCnt;
  }
}

void ES100_request_resync(void) {
  if (!continuous && !rx_active) {
    rx_request = true;
  }
}

// ======================================================
// =========== LCD Display ==============
// ======================================================

#include <LiquidCrystal.h>

#define lcdRS 4
#define lcdEN 5
#define lcdD4 8
#define lcdD5 9
#define lcdD6 10
#define lcdD7 11
LiquidCrystal lcd(lcdRS, lcdEN, lcdD4, lcdD5, lcdD6, lcdD7);

void LCD_setup(void) {
  lcd.begin(20, 4);
  lcd.clear();
}

void print_spaces(int n) {
  while(n > 0) {
    lcd.print(" ");
    --n;
  }
}

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
// US Pacific Time Zone (Las Vegas, Los Angeles)
//TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};
//TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};
Timezone myTZ(myDST, mySTD);

void LCD_update(void) {
  char s[64];  // We can only use up to 21, but sprint_clock_comparison could get large for very large skews.
  // Line 1: Current time, sync source.
  lcd.setCursor(0, 0);
  lcd.print(sprint_unixtime(s, myTZ.toLocal((*sys_clock).unixtime()), false));
  print_spaces(17 - strlen(s));
  lcd.print(sys_clock->name_);
  // Line 2: RTC vs GPS.
  lcd.setCursor(0, 1);
  lcd.print("R: ");
  lcd.print(sprint_clock_comparison(s, rtc_clock, *sys_clock));
  print_spaces(20 - strlen(s) - strlen("R: "));
  // Line 3: ATX vs GPS.
  lcd.setCursor(0, 2);
  lcd.print("A: ");
  if (atx_clock.synced_) {
    lcd.print(sprint_clock_comparison(s, atx_clock, *sys_clock));
  } else {
    s[0] = '\0';  // So rest of line is cleared.
  }
  print_spaces(20 - strlen(s) - strlen("A: "));
  // Line 4: Time of most recent ATX sync.
  lcd.setCursor(0, 3);
  lcd.print("A sync: ");
  if (atx_clock.synced_) {
    lcd.print(sprint_unixtime(s, myTZ.toLocal(atx_clock.base_unixtime_), false));
  }
}

// ======================================================
// Main setup() and loop()
// ======================================================

void setup() {
  // Start the I2C interface
  Wire.begin();
  
  // initialize serial communication at 9600 bits per second.
  Serial.begin(9600);
  Serial.println("settling");
  delay(500);
  Serial.println("synchronizer_es100");

  setup_RTC();
  setup_GPS_serial();
  setup_portc_change_interrupt();

  LCD_setup();
  buttons_setup();
  ES100_setup();
  
}

long secs_last_change = 0;

void loop() {
  buttons_update();
  int change = update_portc_change_times();
  if (change & 0x01)  pending_RTC_interrupt = true;
  if (change & 0x02)  pending_GPS_interrupt = true;
  update_RTC();
  update_GPS();
  if(gps_clock.synced_) {
    sys_clock = &gps_clock;
  }
  update_GPS_serial();
  ES100_update();
  long rtc_secs = rtc_clock.unixtime();
  if ((secs_last_change != rtc_secs)) {
    secs_last_change = rtc_secs;
    serial_clock_debug(rtc_clock);
    serial_clock_debug(gps_clock);

    Serial.print(" Aint=");
    Serial.print(interruptCnt);
    Serial.println();
    // And on LCD:
    LCD_update();
  }
}
