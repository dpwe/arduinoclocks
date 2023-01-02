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

  dpwe@google.com 2022-12-31
*/

#define SQWV_PIN 3

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

#define DS3231_C_A1IE 0  // Alarm 1 Interrupt Enable is bit 0 of Control
#define DS3231_C_A2IE 1  // Alarm 2 Interrupt Enable
#define DS3231_C_INTCN 2  // SQWV reflects interrupts, not oscillator

#define DS3231_S_A1F 0  // Alarm 1 Fired is bit 0 of Status
#define DS3231_S_A2F 1  // Alarm 2 Fired is bit 1 of Status

// ----- DS3231 emulation -----
#include "RTClib.h"  // For DateTime etc.
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
static uint8_t dowToDS3231(uint8_t d) { return d == 0 ? 7 : d; }

// Space for registers.
#define NUM_REGISTERS 19
uint8_t registers[NUM_REGISTERS];

void ds3231_setup() {
  // Zero-out registers.
  for (int i = 0; i < NUM_REGISTERS; ++i) {
    registers[i] = 0;
  }
  // Initialize temperature to something plausible, 25.0 C
  registers[DS3231_TEMPERATUREREG] = 25;
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

// Track whether we have a low pulse on sqwv that needs clearing.
uint32_t last_sqwv_millis = 0;
// After how many ms should we return sqwv high?
const uint32_t sqwv_pulse_ms = 500;

void ds3231_tick(int seconds=1) {
  // Drive SQWV 1 Hz output if in oscillator mode (ignores RSx bits).
  if (!(registers[DS3231_CONTROL] & _BV(DS3231_C_INTCN))) {
    digitalWrite(SQWV_PIN, LOW);
    // Set up semaphore for the oscillator pulse to be reset later.
    last_sqwv_millis = millis();
  }
  
  // Add some number of seconds to the internal clock.
  uint32_t time = decode_time_from_regs(registers).unixtime();
  time += seconds;
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

  if (registers[DS3231_CONTROL] & _BV(DS3231_C_INTCN)) {
    // Output pin is reflecting interrupts.
    if (registers[DS3231_CONTROL] & registers[DS3231_STATUSREG] & 0x03) {
      // At least one alarm fired bit is set when the corresponding interrupt enable bit is set.
      digitalWrite(SQWV_PIN, LOW);
    } else {
      // No interrupt.
      digitalWrite(SQWV_PIN, HIGH);
    }
  }
}

// --------- I2C handlers --------------
uint8_t cursor = 0;  // Address of next register access.

void receiveEvent(int howmany) {
  // Assume we got at least one data byte, and it's the cursor.
  cursor = Wire.read();

  // Any subsequent bytes are writes to that register.
  while (Wire.available()) {
    uint8_t x = Wire.read();
    if (cursor < NUM_REGISTERS) {
      registers[cursor] = x;
    }
    ++cursor;
  }
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
}

uint32_t last_millis = 0;

void loop() {
  uint32_t now_millis = millis();
  // Do we need to reset the SQWV output 1Hz low pulse?
  if (last_sqwv_millis && (now_millis - last_sqwv_millis) >= sqwv_pulse_ms) {
    digitalWrite(SQWV_PIN, HIGH);
    last_sqwv_millis = 0;  // Indicates no pulse waiting to be cleared.
  }
  // Should we tick on the clock?
  if (now_millis - last_millis >= 1000) {
    last_millis += 1000;
    ds3231_tick();
  }
  // Time delay in loop
  delay(10);
}
