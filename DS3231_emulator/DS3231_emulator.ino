/*
  DS3231_emulator

  Arduino as I2C minion emulates behavior of DS3231 RTC
  including alarms and 1Hz SQWV output on SQWV_PIN
  but no 32 kHz output nor other SQWV frequencies
  and aging offset has no influence on time.

  Currently time is driven by millis() (i.e. Arduino system clock).

  Connections:

     A4  I2C Data
     A5  I2C Clock
     D3  SQWV/_INT output

  DESIGN

  For greatest accuracy, the RTC state needs to update as soon as possible
  after a "tick" event, either from internal or external hardware timers.  
  To avoid processing delay, we precompute the state *at the next second*
  and set up a "double buffered" set of registered.  Then, when the "tick"
  interrupt occurs, we switch the register pointer to the precomputed set, 
  and update the output pin (as appropriate).  Then, back in the foreground
  loop, we notice that the time has changed, and set up for the next tick.

  Any configuration change (writing to registers) triggers a recompute of
  the next-tick state.

  The synchronization to the hardware timer is reset when the seconds
  register is written (only).

  Note, registers can be written "at any time" by i2c transactions.  In
  theory this can affect derived state, for instance alarm trigger state. 
  We have to avoid race conditions where registers are modified, but then
  a tick causes a double-buffer swap before the new register values are 
  propagated to the setup for the next tick.

  dpwe@google.com 2022-12-31
*/

// Include Arduino Wire library for I2C
#include <Wire.h>

#ifdef ARDUINO_ARCH_RP2040
  // Hardware limits mean that pins 24 and 25 (A4 and A5, favored choice for ext_i2)
  // must be assigned to I2C0 aka Wire on RP2040.  Wire1 is only for pins 2(n+1), 2(n+1)+1.
  const int ext_sda_pin = 24;
  const int ext_scl_pin = 25;
  #define EXT_I2C Wire
  #define INT_I2C Wire1
#else
  // ESP32-S3
  const int ext_sda_pin = A4;
  const int ext_scl_pin = A5;
  #define EXT_I2C Wire1
  #define INT_I2C Wire
#endif

// Emit SQWV on LED pin.
#define SQWV_PIN 13

// Define minion I2C Address (matches DS3231 RTC)
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
#define DS3231_OUTPINSTATE    0x12 /// We use this byte to store the output pin state.  It's not in the chip.

#define DS3231_C_A1IE 0  // Alarm 1 Interrupt Enable is bit 0 of Control
#define DS3231_C_A2IE 1  // Alarm 2 Interrupt Enable
#define DS3231_C_INTCN 2  // SQWV reflects interrupts, not oscillator

#define DS3231_S_A1F 0  // Alarm 1 Fired is bit 0 of Status
#define DS3231_S_A2F 1  // Alarm 2 Fired is bit 1 of Status

// ------ Counter timer ------

// Use a hardware timer provide accurate PPS ticks.
// Architecture dependent.  Code must provide timer_setup(),
// then arrange to call clock_tick() once per second.
//
// Changes to nanosecs_per_sec_trim should adjust rate (more positive = slower).
//
// timer_reset_sync() should reset the phase of the ticks (so the following tick is
// exactly 1 sec later).

// Forward-declare the function that advances the clock.
void clock_tick(void);

int32_t tick_period_us = 1000000L;  // Counter is microseconds.

int32_t timer_next_firing = 0;

// Nanosecs per sec is ppb trim.
int32_t nanosecs_per_sec_trim = 0;
volatile int32_t cumulated_nanos = 0;

void timer_update_trim_ppb(int ppb) {
  nanosecs_per_sec_trim = ppb;
}

#ifdef ARDUINO_ARCH_RP2040

static bool _repeating_timer_callback(struct repeating_timer *t) {
  clock_tick();
  cumulated_nanos += nanosecs_per_sec_trim;
  int micros_offset = cumulated_nanos / 1000;
  cumulated_nanos -= micros_offset * 1000;
  //timerAlarmWrite(timer, tick_period_us + micros_offset, true);
  return true;
}

// Should probably use a succession of add_alarm_at to get variable timing.
struct repeating_timer mTimer;

void timer_setup(void) {
  // Setup regular timer interrupt.
  // Negative period specifies (negative of) delay between successive calls,
  // not between end of handler and next call.
  add_repeating_timer_us(-tick_period_us, _repeating_timer_callback, NULL, &mTimer);
}

void timer_reset_sync(void) {
  // Delete then restart the timer.
  cancel_repeating_timer(&mTimer);
  timer_setup();
}

#endif  // RP2040

#ifdef ESP32

hw_timer_t * timer = NULL;

void ARDUINO_ISR_ATTR onTimer(){
  clock_tick();
  // Setup for next alarm, including cumulated nanos.
  cumulated_nanos += nanosecs_per_sec_trim;
  int micros_offset = cumulated_nanos / 1000;
  cumulated_nanos -= micros_offset * 1000;
  timerAlarmWrite(timer, tick_period_us + micros_offset, true);
}

void timer_setup(void) {
  // After https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler to get 1us events.
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, tick_period_us, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void timer_reset_sync(void) {
  // Happens e.g. when seconds register is written.  Make seconds happen relative to now.
  timerWrite(timer, 0);
}

#endif // ESP32

// ----- DS3231 emulation -----
#include "RTClib.h"  // For DateTime etc.
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
static uint8_t dowToDS3231(uint8_t d) { return d == 0 ? 7 : d; }

// Space for registers.
#define NUM_REGISTERS 19
// Double-buffers to allow immediate register updates.
uint8_t registers0[NUM_REGISTERS + 1];    // 20th address holds target state of SQWV output
uint8_t registers1[NUM_REGISTERS + 1];
// Current register set.
uint8_t *registers = registers0;
uint8_t *registers_next = registers1;

class DateTime decode_time_from_regs(uint8_t *buffer) {
  // Convert ds3231 time registers to Unix time.
  // Just like reading from the chip in RTClib_DS3231.cpp.
 return DateTime(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5] & 0x7F),
                 bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
                 bcd2bin(buffer[0] & 0x7F));
}

void encode_time_to_regs(const DateTime& dt, uint8_t *buffer) {
  // Write a date/time back into the emulated ds3231 time registers.
  // Just like setting a new time to the chip in RTClib_DS3231.cpp.
  buffer[0] = bin2bcd(dt.second());
  buffer[1] = bin2bcd(dt.minute());
  buffer[2] = bin2bcd(dt.hour());
  buffer[3] = bin2bcd(dowToDS3231(dt.dayOfTheWeek()));
  buffer[4] = bin2bcd(dt.day());
  buffer[5] = bin2bcd(dt.month());
  buffer[6] = bin2bcd(dt.year() - 2000U);
}

void ds3231_setup() {
  // Zero-out registers.
  for (int i = 0; i < NUM_REGISTERS + 1; ++i) {
    registers[i] = 0;
  }
  // Initialize time to something legal.
  encode_time_to_regs(DateTime(2000, 1, 1, 0, 0, 0), registers);
  // Initialize temperature to something plausible, 25.0 C
  registers[DS3231_TEMPERATUREREG] = 25;
  
  // Copy to 2nd registers.
  for (int i = 0; i < NUM_REGISTERS + 1; ++i) {
    registers_next[i] = registers[i];
  }
}

class DateTime decode_alarm(uint8_t *buffer, uint8_t alarm_num, uint8_t *p_alarm_mode) {
  // Read the current alarm time as a DateTime, also return the mode (i.e., fields to match).
  // Essentially like getAlarm1
  uint8_t alarm_mode_bits = 0;
  uint8_t seconds;
  if (alarm_num == 1) {
    buffer += DS3231_ALARM1;  // Base of alarm1 data.
    seconds = bcd2bin(buffer[0] & 0x7F);
    alarm_mode_bits = (buffer[0] & 0x80) >> 7;
  } else { // Alarm 2
    buffer += DS3231_ALARM2 - 1;  // Base of alarm2, less 1 to leave "space" for seconds..
    seconds = 0;   // But alarm2 seccond are implicitly zero.
  }
  uint8_t minutes = bcd2bin(buffer[1] & 0x7F);
  alarm_mode_bits |= (buffer[1] & 0x80) >> 6;
  // Assumes hour is 24H mode.
  uint8_t hour = bcd2bin(buffer[2] & 0x3F);
  alarm_mode_bits |= (buffer[2] & 0x80) >> 5;
  alarm_mode_bits |= (buffer[3] & 0x80) >> 4;
  uint8_t day = 0;
  if (alarm_mode_bits == 0) {
    // Alarm is for a particular day, so check day-of-week or date.
    bool isDayOfWeek = (buffer[3] & 0x40) >> 6;
    // Encode in mode.
    alarm_mode_bits |= isDayOfWeek << 4;
    if (isDayOfWeek) {
      // Alarm set to match on day of the week
      day = bcd2bin(buffer[3] & 0x0F);
    } else {
      // Alarm set to match on day of the month
      day = bcd2bin(buffer[3] & 0x3F);
    }
  }
  *p_alarm_mode = alarm_mode_bits;
  // On the first week of May 2000, the day-of-the-week number
  // matches the date number.
  return DateTime(2000, 5, day, hour, minutes, seconds);
}

bool check_alarm_match(DateTime &now, DateTime &alarm, uint8_t alarm_mode) {
  // Return True if now and alarm are the same under the alarm mode.
  switch (alarm_mode) {
    case DS3231_A1_Day:
    case DS3231_A1_Date:
      if ((alarm_mode == DS3231_A1_Day) && (now.dayOfTheWeek() != alarm.dayOfTheWeek())) {
        return false;
      }
      if ((alarm_mode == DS3231_A1_Date) && (now.day() != alarm.day())) {
        return false;
      }
      // else fall through ...
    case DS3231_A1_Hour:
      if (now.hour() != alarm.hour()) {
        return false;
      }
      // else fall through ...
    case DS3231_A1_Minute:
      if (now.minute() != alarm.minute()) {
        return false;
      }
      // else fall through ...
    case DS3231_A1_Second:
      if (now.second() != alarm.second()) {
        return false;
      }
      // else fall through ...
    case DS3231_A1_PerSecond:
      return true;
    default:
      Serial.print("Invalid alarm mode: ");
      Serial.println(alarm_mode);
      return false;
  }
}

#ifdef ARDUINO_ARCH_RP2040
#define _BV(bit) (1 << bit)
#endif

#define CHECK_BIT(reg, bit) (reg & _BV(bit))

void ds3231_tick(uint8_t *registers, uint8_t advance=1) {
  // Add one second (by default) to the internal clock.
  // We also allow advance=0 to simply recalculate state (alarm outputs) without advancing clock.
  uint32_t time = decode_time_from_regs(registers).unixtime();
  time += advance;
  DateTime now(time);
  encode_time_to_regs(now, registers);
  
  // Check for alarm conditions.
  uint8_t alarm1_mode;
  DateTime alarm1_time = decode_alarm(registers, /* alarm */1, &alarm1_mode);
  if (check_alarm_match(now, alarm1_time, alarm1_mode)) {
    registers[DS3231_STATUSREG] |= _BV(DS3231_S_A1F);
  }

  uint8_t alarm2_mode;
  DateTime alarm2_time = decode_alarm(registers, /* alarm */2, &alarm2_mode);
  if (check_alarm_match(now, alarm2_time, alarm2_mode)) {
    registers[DS3231_STATUSREG] |= _BV(DS3231_S_A2F);
  }

  // Drive SQWV 1 Hz output if in oscillator mode (ignores RSx bits).
  if (!CHECK_BIT(registers[DS3231_CONTROL], DS3231_C_INTCN)) {
    registers[DS3231_OUTPINSTATE] = LOW;
  } else {
    // Output pin is reflecting interrupts.
    if (registers[DS3231_CONTROL] & registers[DS3231_STATUSREG] & 0x03) {
      // At least one alarm fired bit is set when the corresponding interrupt enable bit is set.
      registers[DS3231_OUTPINSTATE] = LOW;
    } else {
      // No interrupt.
      registers[DS3231_OUTPINSTATE] = HIGH;
    }
  }
  // Update the trim by 10 ppb per aging offset.  Positive aging offset makes clock slower.
  // Maybe we should only do this when it is changed, i.e. check writes to AGING register?
  timer_update_trim_ppb(10 * (int)((int8_t)registers[DS3231_AGING]));
}

void ds3231_setup_next_tick(int advance=1) {
  // Setup registers_next to be correct for the next tick event.
  // First, copy current registers to registers_next:
  for (int i = 0; i < NUM_REGISTERS + 1; ++i) {
    registers_next[i] = registers[i];
  }
  // Then, advance it by 1 second.
  ds3231_tick(registers_next, advance);
}

void ds3231_registers_updated(bool seconds_modified) {
  // Notification that the registers were modified externally (e.g. by I2C write).
  // Behavior varies depending on whether seconds were set, in which case the 
  // ticking changes phase.
  // In other situation, let the seconds advance at the next tick.
  ds3231_setup_next_tick(/* advance= */ seconds_modified ? 0 : 1);
  // Need to reset sync if we wrote seconds.
  if (seconds_modified) {
    timer_reset_sync();
    // Update output state from setup_next_tick.
    clock_tick();
  }
}


// ------- Tick interrupt -------

// Track whether we have a low pulse on sqwv that needs clearing.
volatile uint32_t last_sqwv_millis = 0;
// After how many ms should we return sqwv high?
const uint32_t sqwv_pulse_ms = 500;

// Detect a tick in foreground.
volatile bool tick_happened = false;
// Just to help interface with explorer
volatile uint32_t rtc_micros = 0;

void clock_tick(void) {
  // Swap the registers double-buffer.  Do this and update output pin as fast as possible.
  uint8_t *tmp = registers;
  registers = registers_next;
  registers_next = tmp;
  // Update the output pin.
  digitalWrite(SQWV_PIN, registers[DS3231_OUTPINSTATE]);
  if (!CHECK_BIT(registers[DS3231_CONTROL], DS3231_C_INTCN)) {
    // Set up semaphore for the oscillator pulse to be reset later.
    last_sqwv_millis = millis();
  }
  // Record the tick
  tick_happened = true;
  // To let explorer know that this happened.
  rtc_micros = micros();
}


// --------- I2C Delegate handlers (on EXT_I2C) --------------

uint8_t cursor = 0;  // Address of next register access.

void receiveEvent(int howmany) {
  // Assume we got at least one data byte, and it's the cursor.
  bool seconds_modified = false;
  cursor = EXT_I2C.read();

  // Any subsequent bytes are writes to that register.
  while (EXT_I2C.available()) {
    uint8_t x = EXT_I2C.read();
    if (cursor < NUM_REGISTERS) {
      registers[cursor] = x;
      if (cursor == 0) {
        // We wrote the seconds register, reset the timer sync.
        seconds_modified = true;
      }
    }
    ++cursor;
  }
  // Maybe act on the new register values.
  ds3231_registers_updated(seconds_modified);
}

void requestEvent() {
  // Send a stream of bytes back to master, starting from previous value of cursor until end of registers.
  // I *think* I2C indicates how many bytes it wants by sending a STOP after it's had enough.
  // but this doesn't appear to be visible through Wire.
  // So we send everything, and count on any excess being dropped.
  
  // It's critical that cursor has been set (via a preceding receiveEvent) before this runs.
  // The stock DS3231_RTC used i2c_dev->write_then_read, which (on the ESP32_ actually ended
  // up not servicing the receive event until *after* the output buffer had been stuffed, so
  // the cursor was always one transaction behind (works OK on RP2040 Pico).
  // Modifying it to use write() followed by read() fixed it on ESP32.
  
  EXT_I2C.write(registers + cursor, NUM_REGISTERS - cursor);
}


// --------------- setup(), loop() ---------------------

// Secondary I2C bus config.  This acts like the DS3231 chip.

void setup() {
  // Configure SQWV output pin (typically LED).
  pinMode(SQWV_PIN, OUTPUT);
  digitalWrite(SQWV_PIN, HIGH);  // Default state.
  
  // Setup Serial Monitor 
  Serial.begin(9600);
  delay(500);  // For port to wake up.
  Serial.println("DS3231_emulator ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  digitalWrite(SQWV_PIN, LOW);

  // Initialize I2C communications as minion
#ifdef ARDUINO_ARCH_RP2040
  // Configure Pico RP2040 I2C
  EXT_I2C.setSDA(ext_sda_pin);
  EXT_I2C.setSCL(ext_scl_pin);
  EXT_I2C.begin(DS3231_ADDRESS);
#else  // ESP32/ATmeta
  #define I2C_FREQ 100000
  EXT_I2C.begin(DS3231_ADDRESS, ext_sda_pin, ext_scl_pin, I2C_FREQ);
#endif
  // Function to run when data requested from master
  EXT_I2C.onRequest(requestEvent);
  // Function to run when data received from master
  EXT_I2C.onReceive(receiveEvent);

  ds3231_setup();
  timer_setup();
}

void loop() {
  uint32_t now_millis = millis();
  // Do we need to reset the SQWV output 1Hz low pulse?
  if (last_sqwv_millis && (now_millis - last_sqwv_millis) >= sqwv_pulse_ms) {
    digitalWrite(SQWV_PIN, HIGH);
    last_sqwv_millis = 0;  // Indicates no pulse waiting to be cleared.
  }
  // If the clock ticked, set up for next second.
  if (tick_happened) {
    ds3231_setup_next_tick();
    tick_happened = false;
    // Report seconds.
    Serial.println(registers[0], HEX);
  }

  // Time delay in loop
  delay(10);
}
