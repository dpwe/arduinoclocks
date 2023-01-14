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

// Emit SQWV on LED pin.
#define SQWV_PIN 13

// Include Arduino Wire library for I2C
#include <Wire.h>

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

// Use the hardware timer in the ESP32 to provide accurate PPS ticks.

uint32_t counter_ticks_between_firings = 1000000;  // Counter is microseconds.

uint32_t timer_next_firing = 0;

// Nanosecs per sec is ppb trim.
int32_t nanosecs_per_sec_trim = 0;
volatile int32_t cumulated_nanos = 0;

hw_timer_t * timer = NULL;

// Forward-declare the function that advances the clock.
void clock_tick(void);

void ARDUINO_ISR_ATTR onTimer(){
  clock_tick();
  // Setup for next alarm, including cumulated nanos.
  cumulated_nanos += nanosecs_per_sec_trim;
  int micros_offset = cumulated_nanos / 1000;
  cumulated_nanos += micros_offset * 1000;
  timerAlarmWrite(timer, counter_ticks_between_firings + micros_offset, true);
}

void timer_update_trim_ppb(int ppb) {
  nanosecs_per_sec_trim = ppb;
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
  timerAlarmWrite(timer, counter_ticks_between_firings, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void timer_reset_sync(void) {
  // Happens e.g. when seconds register is written.  Make seconds happen relative to now.
  timerWrite(timer, 0);
}

// ----- DS3231 emulation -----
#include "RTClib.h"  // For DateTime etc.
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
static uint8_t dowToDS3231(uint8_t d) { return d == 0 ? 7 : d; }

// Space for registers.
#define NUM_REGISTERS 20  // 20th address holds target state of SQWV output
// Double-buffers to allow immediate register updates.
uint8_t registers0[NUM_REGISTERS];
uint8_t registers1[NUM_REGISTERS];
// Current register set.
uint8_t *registers = registers0;
uint8_t *registers_next = registers1;

void ds3231_setup() {
  // Zero-out registers.
  for (int i = 0; i < NUM_REGISTERS; ++i) {
    registers[i] = 0;
    registers_next[i] = 0;
  }
  // Initialize temperature to something plausible, 25.0 C
  registers[DS3231_TEMPERATUREREG] = 25;
  registers_next[DS3231_TEMPERATUREREG] = 25;
}

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

#define CHECK_BIT(reg, bit) (reg & _BV(bit))

void ds3231_tick(uint8_t *registers) {
  // Add one second to the internal clock.
  uint32_t time = decode_time_from_regs(registers).unixtime();
  time += 1;
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
}

void ds3231_setup_next_tick(void) {
  // Setup registers_next to be correct for the next tick event.
  // First, copy current registers to registers_next:
  for (int i = 0; i < NUM_REGISTERS; ++i) {
    registers_next[i] = registers[i];
  }
  // Then, advance it by 1 second.
  ds3231_tick(registers_next);
}


// --------- I2C handlers --------------
// We need to track that the registers have been updated and may not yet
// have propagated to registers_next.
volatile bool registers_dirty = false;

uint8_t cursor = 0;  // Address of next register access.

void receiveEvent(int howmany) {
  // Assume we got at least one data byte, and it's the cursor.
  cursor = Wire.read();

  // Any subsequent bytes are writes to that register.
  while (Wire.available()) {
    uint8_t x = Wire.read();
    if (cursor < NUM_REGISTERS) {
      registers[cursor] = x;
      if (cursor == 0) {
        // We wrote the seconds register, reset the timer sync.
        timer_reset_sync();
      }
    }
    ++cursor;
  }
  registers_dirty = true;
}

void requestEvent() {
  // Send a stream of bytes back to master, starting from previous value of cursor until end of registers.
  // I *think* I2C indicates how many bytes it wants by sending a STOP after it's had enough.
  // but this doesn't appear to be visible through Wire.
  // So we send everything, and count on any excess being dropped.
  uint8_t x = 0;
#ifdef ONEBYONE
  while (cursor < NUM_REGISTERS) {
    x = registers[cursor];
    Serial.print("Send Reg[");
    Serial.print(cursor);
    Serial.print("]=");
    Serial.println(x);
    Wire.write(x);
    ++cursor;
  }
#else  // Use block write.
  //Serial.print("Send regs from ");
  //Serial.println(cursor);
  Wire.write(registers + cursor, NUM_REGISTERS - cursor);
#endif
}

// ------- Tick interrupt -------

// Track whether we have a low pulse on sqwv that needs clearing.
volatile uint32_t last_sqwv_millis = 0;
// After how many ms should we return sqwv high?
const uint32_t sqwv_pulse_ms = 500;

// Detect a tick in foreground.
volatile bool tick_happened = false;

void clock_tick(void) {
  if (registers_dirty) {
    // Registers have been modified externally e.g. by i2c_write.
    // This is normally handled by the foreground task and we specifically don't want to 
    // do it here, to avoid latency between clock tick and state update.  However, if the
    // registers *just* got changed by an i2c_write, and we haven't yet propagated to 
    // registers_next, we need to take care of that here - or we'll lose the register updates.
    ds3231_setup_next_tick();
    registers_dirty = false;
  }
  // Swap the registers double-buffer.  Do this and update output pin as fast as possible.
  uint8_t *tmp = registers;
  registers = registers_next;
  // Update the output pin.
  digitalWrite(SQWV_PIN, registers[DS3231_OUTPINSTATE]);
  registers_next = registers;
  // Copy the new registers as setup for the next tick.
  // Record the tick
  tick_happened = true;
  if (!CHECK_BIT(registers[DS3231_CONTROL], DS3231_C_INTCN)) {
    // Set up semaphore for the oscillator pulse to be reset later.
    last_sqwv_millis = millis();
  }
  // Update the trim.
  timer_update_trim_ppb((int8_t)registers[DS3231_AGING]);
}

// --------------- setup(), loop() ---------------------

void setup() {
  // Configure SQWV output pin.
  pinMode(SQWV_PIN, OUTPUT);
  digitalWrite(SQWV_PIN, HIGH);  // Default state.
  
  // Initialize I2C communications as minion
  Wire.begin(DS3231_ADDRESS);
  
  // Function to run when data requested from master
  Wire.onRequest(requestEvent); 
  
  // Function to run when data received from master
  Wire.onReceive(receiveEvent);
  
  // Setup Serial Monitor 
  Serial.begin(9600);
  Serial.println("DS3231_emulator ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

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
  if (registers_dirty) {
    // Registers have been modified externally e.g. by i2c_write.
    ds3231_setup_next_tick();
    registers_dirty = false;
  }
  // Time delay in loop
  delay(10);
}
