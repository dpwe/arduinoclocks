// wwvb_simulator
// 
// Generate modulated 60 kHz signal
// to feed WWVB receivers

// See https://www.anishathalye.com/2016/12/26/micro-wwvb/

// We use Timer1 to generate a "FastPWM" signal with a frequency of 16 MHz / 267 = 59,925 Hz.
// On OC1A/PB1/Pin 9, we emit that signal continually, but on OC1B/PB2/Pin 10, we turn off
// the in-phase 60 kHz signal for part of each signal to generate the modulated carrier.  By
// adding PB1 and PB2 (e.g. with 1k to PB2 and 10k to PB1, so the modulation is deep) we get
// the voltage to feed to the antenna.

// Pin that goes high for a second at the start of each minute, just for debugging.
const uint8_t frame_pin = 12;

// ==========================
// 60 kHz output
// ==========================

void setup_60kHz(void) {
  // WGM1{3:0} = 0b1110 (14) => Fast PWM with TOP = ICR1
  // COM1[AB]{1:0} = 0b10 => Set OC1[AB] at bottom, clear at OCR1[AB] value (for FastPWM).
  // CS1{2:0} = 0b001 => Prescaler is 16 MHz / 1
  // initialize non-inverting fast PWM on OC1B (PB2, Arduino Pin 10) and OC1A (PB1, Arduino Pin 9)
  // count from BOTTOM to ICR1 (mode 14), using /1 prescaler
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) |(1 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (0 << WGM10);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
  // fast PWM:
  // f = f_clk / (N * (1 + TOP)), where N is the prescaler divider
  // we have f_clk = 16 MHz
  // for f = 60 kHz, we want N * (1 + TOP) = 266.67
  // we're using a prescaler of 1, so we want ICR1 = TOP = 266
  // this gives an f = (16MHz / 267) = 59.93 kHz
  // we can use OCR1B to set duty cycle (a fraction of ICR1)
  ICR1 = 266;
  OCR1A = 133; // by default, 50% (134/267) duty
  OCR1B = 133; // by default, 50% (134/267) duty
  DDRB |= (1 << PB1); // set PB1 = OCR1A = Pin10 to an output port
  DDRB |= (1 << PB2); // set PB2 = OCR1B = Pin9 to an output port
  // Enable overflow interrupts
  TIFR1  = 0x00;        // Timer1 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK1 = (1 << TOIE1);        // Timer1 INT Reg: Timer1 Overflow Interrupt Enable
}

// The interrupt routine is ~100 instructions; the short path (without seconds wrap) is ~70 instructions ; TOV1s occur every ~267 cycles.
// A 16 bit increment takes 18 instructions.
// See
// /Users/dpwe/Library/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin/avr-objdump -d /var/folders/_w/z80db8m163q5jtt95yc1wgqh0000gq/T/arduino_build_54692/wwvb_simulator.ino.elf

const unsigned int rf_cycles_per_sec = 59925;
unsigned int rf_cycles_per_duty = long(59925 * 0.8);  // Changed every second to achieve data modulation.
unsigned int rf_cycle_count = 0;

#define LED PB5  // Arduino pin13 == LED

// Semaphore/index for stepping through symbols.
volatile uint8_t symbol_index = 0;

ISR(TIMER1_OVF_vect) {
  TIFR1 = 0x00;         // Timer1 INT Flag Reg: Clear Timer Overflow Interrupt
  rf_cycle_count++;
  if (rf_cycle_count == rf_cycles_per_duty) {
    // Turn on the 60 kHz.
    OCR1B = 133;
    PORTB |= (1 << LED);  // Light LED
  } else if (rf_cycle_count == rf_cycles_per_sec) {
    rf_cycle_count = 0;
    // 1PPS event.
    OCR1B = 300;  // Stop the 60 kHz (it's larger than ICR1, so never reached).
    PORTB &= ~(1 << LED);  // Clear LED
    PORTB &= ~(1 << PB4);  // Clear frame signal (only set at start of 0th second).
    // Set semaphore to indicate that rf_cycles_per_duty should be updated, but let that happen outside interrupt.
    // We have at least 0.2 sec to fix it.
    ++symbol_index;
  }
}

// ============================
// RTC
// ============================
#include "RTClib.h"
RTC_Millis rtc;

// Function to return the compile date and time as a time_t value
class DateTime compileTime(void)
{
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char chMon[3], *m;

    strncpy(chMon, compDate, 3);
    chMon[3] = '\0';
    m = strstr(months, chMon);
    uint8_t month = ((m - months) / 3 + 1);
    uint8_t day = atoi(compDate + 4);
    uint16_t year = atoi(compDate + 7) - 1970;
    uint8_t hour = atoi(compTime);
    uint8_t minute = atoi(compTime + 3);
    uint8_t second = atoi(compTime + 6);

    return DateTime(year, month, day, hour, minute, second);
}

void setup_rtc(void){
  Serial.print(__DATE__);
  Serial.println(__TIME__);
  rtc.begin(compileTime());
}

// ============================
// Time Formatting
// ============================

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

char *sprint_datetime(char *s, DateTime &dt, bool show_date=false)
{  // Returns full terminated string
  char *entry_s = s;
  // s must provide 20 bytes, or 9 if just time.
  if (show_date) {
    s = sprint_int(s, dt.year() + 2000);
    *s++ = '-';
    s = sprint_int2(s, dt.month());
    *s++ = '-';
    s = sprint_int2(s, dt.day());
    *s++ = ' ';
  }
  s = sprint_int2(s, dt.hour());
  *s++ = ':';
  s = sprint_int2(s, dt.minute());
  *s++ = ':';
  s = sprint_int2(s, dt.second());
  *s++ = 0;
  return entry_s;
}

// ============================
// Manage 1PPS symbol sequence
// ============================

// Symbol types
const uint8_t ZERO = 0;
const uint8_t ONE = 1;
const uint8_t MARK = 2;

// Table that maps Symbol type into count of for low energy
// ZERO = 0.2 s = (0.2 * 60,000) cycles = 12000 cycles
// ONE  = 0.5 s = (0.5 * 60,000) cycles = 30000 cycles
// MARK = 0.8 s = (0.8 * 60,000) cycles = 48000 cycles
const unsigned int low_cycles[] = { 11985, 29963, 47940 };

// Table of symbols for each second in the 1-minute message.
const uint8_t symbols_len = 60;
byte symbols[symbols_len];

void setup_symbols(void) {
  for(int i = 0; i < symbols_len; ++i) {
    symbols[i] = ZERO;
  }
}

void update_duty_cycle(void) {
  // symbol_index has been incremented by interrupt routine.  Update the duty cycle.
  if (symbol_index == symbols_len) {
    // Reached end of table, go back to start
    symbol_index = 0;
    PORTB |= (1 << PB4);   // Set frame flag
  }
  rf_cycles_per_duty = low_cycles[symbols[symbol_index]];
}

uint8_t last_symbol_index = 0;

void update_symbols(void) {
  if (symbol_index != last_symbol_index) {
    last_symbol_index = symbol_index;
    update_duty_cycle();  // Will wrap symbol_index to zero when needed.
    if (symbol_index == 0) {
      // Update the symbols for the current minute.
      // Since the first symbol is always the same (MARK), we have at least 1 second to finish this.
      DateTime dt = rtc.now();
      set_timecode(dt);
      char time_str[20];
      Serial.println(sprint_datetime(time_str, dt));  
    }
  }
}

// ============================
// WWVB timecodesymbol sequence
// ============================

void setup_timecode(void) {
  // Assume symbols[] is set up.
  // Fixed portions of WWVB timecode (see https://en.wikipedia.org/wiki/WWVB)
  symbols[0] = MARK;
  for (int i = 0; i < 6; ++i) {
    symbols[10*i + 9] = MARK;
  }
  // Multiple bits are always zero, leave as initialized.
  // symbols[4, 10, 11, 14, 20, 21, 24, 34, 35, 44, 54] = ZERO.

  // Dummy DUT1.  DUT1 is -0.1 sec since 2021-07-17; 
  // see https://www.nist.gov/pml/time-and-frequency-division/time-realization/leap-seconds
  // DUT sign: negative
  symbols[36] = ZERO;
  symbols[37] = ONE;
  symbols[38] = ZERO;
  // DUT value: 0.1
  symbols[40] = ZERO;
  symbols[41] = ZERO;
  symbols[42] = ZERO;
  symbols[43] = ONE;
  // Leave Leap Seconds flag symbols[56] as ZERO.
  // Leave DST indicators symbols[57, 58] as ZERO (DST not in effect).
}

uint8_t *set_bcd(uint16_t value, uint8_t *buf, uint8_t nbits) {
  // Write the value as a series of nbits bits into *buf.
  if (nbits > 4) {
    // Recursively write upper value first
    buf = set_bcd(value / 10, buf, nbits - 4);
    buf++;  // We always skip a bit after high digit(s).
    nbits = 4;
    value = value % 10;
  }
  while(nbits--) {
    uint8_t test_bit = (1 << nbits);
    if (value & test_bit) {
      *buf = 1;
    } else {
      *buf = 0;
    }
    buf++;
  }
  return buf;
}

// Copied from RTClib.cpp
const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30};

static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000U)
    y -= 2000U;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += daysInMonth[i - 1];
  if (m > 2 && y % 4 == 0)
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

uint16_t day_of_year(DateTime& dt) {
  // Count of days since jan 1st; jan 1st = 1.
  return 1 + date2days(dt.year(), dt.month(), dt.day()) - date2days(dt.year(), 1, 1);
}

void set_timecode(DateTime& dt) {
  // Update the timecode for the given date.
  set_bcd(dt.minute(), symbols + 1, 7);
  set_bcd(dt.hour(), symbols + 12, 6);
  set_bcd(day_of_year(dt), symbols + 22, 10);
  set_bcd(dt.year() % 100, symbols + 45, 8);
  // Leap year flag.
  symbols[55] = (dt.year() % 4) == 0;
}

// ============================
// Main
// ============================


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    
  Serial.println("** wwvb_simulator **");
  // Default OC1A,B are output low
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  // Pin for frame start
  pinMode(frame_pin, OUTPUT);
  digitalWrite(frame_pin, LOW);

  setup_rtc();
  setup_symbols();
  setup_timecode();
  setup_60kHz();
}

void loop() {
  update_symbols();
}
