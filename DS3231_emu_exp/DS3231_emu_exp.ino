// DS3231_emu_exp
//
// Merger of DS3231_emulator and DS3231_explorer
//
// The emulator maintains a PPS timer and emulates the DS3231 logic.
// The explorer displays its state and allows control via CLI.
// For now, we disable the I2C delegate serving of the emulator, since we also need I2C mastery to drive the display.
//
// dpwe@google.com 2023-01-14

// Explorer Command set:
// A - read aging offset register
// Ann - Set aging offset register
// Bx - Enable (x=1) / Disable (x=0) sqwv output on battery power
// Cx - Enable (x=1) / Disable (x=0) the 32 kHz output
// D - Display all registers
// Ex - Enable (x=1) / Disable (x=0) clock oscillator when on battery
// Ix - Enable alarm interrupt outputs on sqwv pin (x=1) / Enable sqwv frequency output (x=0) 
// L - Read Alarm 1
// Lx - Enable (x=1) / Disable (x=0) Alarm 1
// Lss - Set Alarm 1 for every minute at seconds SS
// Lmmss - Set Alarm 1 for every hour at min/sec MM:SS
// Lhhmmss - Set Alarm 1 for every day at time HH:MM:SS
// LWhhmmss - Set Alarm 1 for every week on dow W (Sunday = 1) at HH:MM:SS
// LDDhhmmss - Set Alarm 1 for every month on date DD at HH:MM:SS
// M - Read Alarm 2
// Mx - Enable (x=1) / Disable (x=0) Alarm 2
// Mmm - Set Alarm 2 for every hour at minutes mm
// Mhhmm - Set Alarm 2 for every day at time hh:mm
// MWhhmm - Set Alarm 2 for every week on dow W (Sunday = 1) at hh:mm
// MDDhhmm - Set Alarm 2 for every month on date DD at hh:mm
// Px - Enable (x=1) / Disable (x=0) continuous polling of current time.
// Q - Read the sqwv frequency
// Qx - Set sqwv frequency : x=0 -> 1 Hz / x=1 -> 1024 Hz / x=2 -> 4096 Hz / x=3 -> 8192 Hz
// R - Reset the Oscillator Stop Flag
// Sx - Enable (x=1) / Disable (x=0) clock read on SQWV interrupt (on D3).
// T - Report most recent temp measurement
// T1 - Initiate a new temperature conversion
// Z - Read date/time
// ZYYYYMMDDhhmmss - Set date/time

// Emulator design:
//   Arduino as I2C Secondary emulates behavior of DS3231 RTC
//  including alarms and 1Hz SQWV output on SQWV_PIN
//  but no 32 kHz output nor other SQWV frequencies
//  and aging offset has no influence on time.
//
//  Connections:
//
//     A4  I2C Data
//     A5  I2C Clock
//     D13 SQWV/_INT output
//
//  DESIGN
//
//  For greatest accuracy, the RTC state needs to update as soon as possible
//  after a "tick" event, either from internal or external hardware timers.
//  To avoid processing delay, we precompute the state *at the next second*
//  and set up a "double buffered" set of registered.  Then, when the "tick"
//  interrupt occurs, we switch the register pointer to the precomputed set,
//  and update the output pin (as appropriate).  Then, back in the foreground
//  loop, we notice that the time has changed, and set up for the next tick.
//
//  Any configuration change (writing to registers) triggers a recompute of
//  the next-tick state.
//
//  The synchronization to the hardware timer is reset when the seconds
//  register is written (only).
//
//  Note, registers can be written "at any time" by i2c transactions.  In
//  theory this can affect derived state, for instance alarm trigger state.
//  We have to avoid race conditions where registers are modified, but then
//  a tick causes a double-buffer swap before the new register values are
//  propagated to the setup for the next tick.
//
//  dpwe@google.com 2022-12-31

#include <SPI.h>
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

#include <RTClib.h>         // Adafruit; defines RTC_DS3231

#ifdef ARDUINO_ARCH_RP2040
  // Hardware limits mean that pins 24 and 25 (A4 and A5, favored choice for ext_i2)
  // must be assigned to I2C0 aka Wire on RP2040.  Wire1 is only for pins 2(n+1), 2(n+1)+1.
  const int ext_sda_pin = 24;
  const int ext_scl_pin = 25;
  #define EXT_I2C Wire
  const int int_sda_pin = 2;
  const int int_scl_pin = 3;
  #define INT_I2C Wire1

  #define DISPLAY_SH1107  // 128x(64,128) mono OLED in Feather stack

//  #define SCREEN_HEIGHT 64
  #define SCREEN_HEIGHT 128

#else
  // ESP32-S3
  const int ext_sda_pin = A4;
  const int ext_scl_pin = A5;
  #define EXT_I2C Wire1
  #define INT_I2C Wire

  #define DISPLAY_ST7789  // Built-in display on ESP32-S3 TFT
  //#define DISPLAY_SSD1351  // Exernal 128x128 RGB TFT

#endif


// ------------- Display ---------------

#include <Adafruit_GFX.h>

#ifdef DISPLAY_SSD1351
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
  #include <Adafruit_SH110X.h>

  //#define FEATHER_OLED
  #ifdef FEATHER_OLED
    const int display_address = 0x3C;
    #define SCREEN_HEIGHT 64
  #else // standalone OLED
    const int display_address = 0x3D;
    #define SCREEN_HEIGHT 128
  #endif

  #define SCREEN_WIDTH  128
  #define SIZE_1X

  Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, &INT_I2C);

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
  Serial.println("About to initialize display...");
  display.begin(display_address, true);
  Serial.println("Display initialized.");
  display.display();  // Splashscreen
  delay(1000);
  display.clearDisplay();
  display.display();
  display.setRotation(1);
#endif

  display.fillScreen(BLACK);

  // text display 
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("DS3231_emu_exp");
}

char *CONTROL_SHORTNAMES[8] = {"E", "Q", "C", "R", "R", "I", "E", "E"};
char *STATUS_SHORTNAMES[8]  = {"O", "x", "x", "x", "3", "B", "F", "F"};

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

void print_bits_tft(uint16_t x, uint16_t y, uint8_t val, char* names[8], uint16_t fgcolor=WHITE, uint16_t bgcolor=BLACK) {
  // Print a bit set using an array of names.
  uint8_t mask = 0x80;  // Start with top bit.
  display.setTextColor(fgcolor, bgcolor);
  display.setCursor(x, y);
  for (uint8_t bit = 0; bit < 8; ++bit) {
    bool bitval = ((val & mask) > 0);
    // "Set" bits are printed in reverse video.
    if (bitval)   display.setTextColor(bgcolor, fgcolor);
    display.print(names[bit]);
    uint16_t w = strlen(names[bit]) * CHAR_W;
    if (bitval) {
      display.setTextColor(fgcolor, bgcolor);
      // Also add top/left edges
      display.drawFastHLine(x - 1, y - 1, w + 1, fgcolor);
      display.drawFastVLine(x - 1, y, ROW_H, fgcolor);
    } else {
      // Need to undraw the overbar.
      display.drawFastHLine(x - 1, y - 1, w + 1, bgcolor);
    }
    x += w;
    display.print(" ");
    x += CHAR_W;
    mask >>= 1;
  }
}

void getAlarmModeTemplateString(char *s, uint8_t mode, uint8_t alarm_num) {
  if (alarm_num == 2) {
    // Map alarm2 modes to alarm1 modes by shifting left.
    mode <<= 1;
  }
  // Templates are for alarm2; for alarm1, we'll add seconds later.
  switch (mode) {
    case DS3231_A1_PerSecond:
    case DS3231_A1_Second:
      strcpy(s, "--- --:--");
      break;
    case DS3231_A1_Minute:
      strcpy(s, "--- --:mm");
      break;
    case DS3231_A1_Hour:
      strcpy(s, "--- hh:mm");
      break;
    case DS3231_A1_Date:
      strcpy(s, "-DD hh:mm");
      break;
    case DS3231_A1_Day:  // Day of the week.
      strcpy(s, "DDD hh:mm");
      break;
  }
  if (alarm_num == 1) {
    // Need to append seconds field.
    char *s_end = s + strlen(s);
    if (mode == DS3231_A1_PerSecond) {
      strcpy(s_end, ":--");
    } else {
      strcpy(s_end, ":ss");
    }
  }
}

void ds3231_display(class RTC_DS3231& ds3231, const char *clock_name, bool gps_active) {
  // Graphical display of DS3231 state for 16x8 display:
  // HHHH::MMMM::SSSS
  // HHHH::MMMM::SSSS (double-size)
  // 2023-01-06
  // A1: --- --:03:00 (only enabled shown)
  // A2: Wed 03:59
  // C: E Q C R R I E E  (reverse video for set bits)
  // S: O x x x 3 B F F
  // A:-127  T:23.25C

  // Regs would be 19 bytes ~ 57 chars incl. spaces
  // SS MM HH OO DD MM YY  - maybe 7 bytes with N extra pixels between bytes = 7x12 + Nx6 - 96 for N=2, 120 for N=6 (128 is 21 chars @6)
  // S1 M1 H1 D1 M2 H2 D2
  // CC SS AO TH TL

  // Clock source identifier tag
  display.setTextSize(SMALL_SIZE);
  display.setTextColor(RED, BLACK);
  display.setCursor(18 * CHAR_W, 0);
  display.print(clock_name);

  // GPS status
  display.setCursor(18 * CHAR_W, ROW_H);
  if (gps_active) {
    display.setTextColor(BLACK, GREEN);
    display.print("GPS");
  } else {
    display.setTextColor(GREEN, BLACK);
    display.print("   ");
  }

  // Time, double size.
  display.setTextSize(LARGE_SIZE);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0, 0);
  char s[32];
  strcpy(s, "hh:mm:ss");
  ds3231.now().toString(s);
  display.print(s);

  // Date, normal size, yellow.
  display.setTextSize(SMALL_SIZE);
  display.setTextColor(YELLOW, BLACK);
  display.setCursor(0, 2 * ROW_H);
  strcpy(s, "YYYY-MM-DD");
  ds3231.now().toString(s);
  display.print(s);

  // Alarm1
  display.setTextColor(CYAN, BLACK);
  display.setCursor(0, 3 * ROW_H);
  strcpy(s, "A1: ");
  getAlarmModeTemplateString(s + 4, (uint8_t)ds3231.getAlarm1Mode(), /* alarm_num */ 1);  
  ds3231.getAlarm1().toString(s);
  display.print(s);

  // Alarm2
  display.setTextColor(BLUE, BLACK);
  display.setCursor(0, 4 * ROW_H);
  strcpy(s, "A2: ");
  getAlarmModeTemplateString(s + 4, (uint8_t)ds3231.getAlarm2Mode(), /* alarm_num */ 2);  
  ds3231.getAlarm2().toString(s);
  display.print(s);

  // Control byte
  display.setTextColor(GREEN, BLACK);
  display.setCursor(0, 5 * ROW_H);
  strcpy(s, "C: ");
  display.print(s);
  print_bits_tft(3 * CHAR_W, 5 * ROW_H, ds3231.getControlReg(), CONTROL_SHORTNAMES, GREEN, BLACK);

  // Status byte
  display.setTextColor(YELLOW, BLACK);
  display.setCursor(0, 6 * ROW_H);
  strcpy(s, "S: ");
  display.print(s);
  print_bits_tft(3 * CHAR_W, 6 * ROW_H, ds3231.getStatusReg(), STATUS_SHORTNAMES, YELLOW, BLACK);

  // Aging offset
  display.setTextColor(RED, BLACK);
  display.setCursor(0, 7 * ROW_H);
  strcpy(s, "A:");
  itoa(ds3231.getAging(), s + 2, 10);
  display.print(s);
  display.print("   ");

  // Temp
  display.setTextColor(MAGENTA, BLACK);
  display.setCursor(8 * CHAR_W, 7 * ROW_H);
  strcpy(s, "T:");
  float t = ds3231.getTemperature();
  itoa(int(t), s + 2, 10);
  char *s_end = s + strlen(s);
  *s_end = '.';
  itoa(100*(t - int(t)), s_end + 1, 10);
  display.print(s);
  display.print("   ");

#ifdef DISPLAY_SH1107
  display.display();
#endif
}

void display_skew_us(long int skew_microseconds) {
  char s[5]; // "-0.0\0"
  long int skew_milliseconds;
  if (skew_microseconds < 0) {
    s[0] = '-';
    skew_microseconds = -skew_microseconds;
  } else {
    s[0] = '+';
  }
  if (skew_microseconds >= 9950L) {
    // format as +/-123
    skew_microseconds += 500;  // rounding
    skew_milliseconds = skew_microseconds / 1000L;
    if (skew_milliseconds > 999L) {
      skew_milliseconds = 999L;
    }
    itoa(skew_microseconds, s + 1, 10);
  } else {
    // format as +/-0.1
    // Round up
    skew_microseconds += 50; // rounding
    skew_milliseconds = skew_microseconds / 1000L;
    s[1] = '0' + skew_milliseconds;
    s[2] = '.';
    s[3] = '0' + ((skew_microseconds / 100L) - (10 * skew_milliseconds));
  }
  s[4] = '\0';
  // Now actually display it.
  display.setTextColor(RED, BLACK);
  display.setCursor(17 * CHAR_W, 2 * ROW_H);
  display.print(s);
}

// -------------- Time --------------------

#define CLOCK_ADDRESS 0x68

#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_AGING 0x10     ///< Aging offset register
#define DS3231_TEMPERATUREREG 0x11 ///< Temperature register (high byte - low byte is at 0x12), 10-bit
                              ///< temperature value

#define time_t uint32_t

RTC_DS3231 ds3231;

void sprint_datetime(const DateTime &dt, char *s) {
  // s must have 20 bytes.
  strcpy(s, "YYYY-MM-DD hh:mm:ss");
  dt.toString(s);
}

void serial_print_time(const DateTime &dt) {
  char s[20];
  sprint_datetime(dt, s);
  Serial.println(s);
}

// getExternalTime is declared to expect a function returning a (signed) long int.
//time_t 
#ifdef ARDUINO_ARCH_RP2040  // Needed to compile on M4
long
#endif
long int RTC_utc_get(void) {
  return ds3231.now().unixtime();
}

void RTC_set_time(const DateTime& dt) {
  // Set the DS3231 time.
  Serial.print("Set RTC: ");
  serial_print_time(dt);
  ds3231.adjust(dt);
  // Resync TimeLib
  //setTime(ds3231.now().unixtime());
}

// ---- Misc formatting -----

void print2Digits(int digits, int base=10)
{
  // Print a 2-digit value with a leading zero if needed.
  if(digits < base)
    Serial.print('0');
  Serial.print(digits, base);
}

void itoa2(int num, char *s, int base=10) {
#define DTOA(d) ((d < 10) ? ('0' + d) : ('A' + d - 10))
  *s++ = DTOA(num / base);
  *s++ = DTOA(num % base);
  *s++ = '\0';
}

void sprint_bits(uint8_t val, char* names[8], char *s) {
  // Print a bit set using an array of names.
  uint8_t mask = 0x80;  // Actually start with top bit.
  for (uint8_t bit = 0; bit < 8; ++bit) {
    strcpy(s, names[bit]);
    s += strlen(s);
    *s++ = ':';
    *s++ = '0' + ((val & mask) > 0);
    *s++ = ' ';
    *s++ = ' ';
    *s = '\0';
    mask >>= 1;
  }
}

void print_bits(uint8_t val, char* names[8]) {
  // Print bits.  Special case for to_display.
  char s[70];
  sprint_bits(val, names, s);
  Serial.print(s);
}

// ----- Encode/decode DS3231 registers ---------

#define BCDTODEC(x) ((x) - 6 * ((x) >> 4))

DateTime ds3231_regs_to_datetime(uint8_t *regs) {
  // Format the 7 byte DS3231 time registers to a DateTime obj.
  uint8_t secs = BCDTODEC(regs[0]);
  uint8_t mins = BCDTODEC(regs[1]);
  uint8_t hours = BCDTODEC(regs[2]);
  uint8_t dow = BCDTODEC(regs[3]);  // 1-7.
  uint8_t day = BCDTODEC(regs[4]);  // 1-31
  uint8_t month = BCDTODEC(regs[5] & 0x7F);  // 1-12
  uint16_t year = 2000 + ((regs[5] & 0x80) ? 100 : 0) + BCDTODEC(regs[6]);
  return DateTime(year, month, day, hours, mins, secs);
}

void sprint_alarm(uint8_t *regs, char *s, bool has_secs=true) {
  // Format the status of alarm from the 4 bytes (0 + 3 bytes for Alarm 2).
  int8_t secs = 0;
  int8_t mode = 0;
  // Alarm2 has no seconds register, so only try to access if it's there.
  if (has_secs) {
    secs = BCDTODEC(regs[0] & 0x7F);
    mode = regs[0] >> 7;
  } else {
    // Alarm2, take seconds register as zero.
    regs -= 1;  // To make remaining registers line up.
  }
  int8_t mins = BCDTODEC(regs[1] & 0x7F);
  int8_t hours = BCDTODEC(regs[2] & 0x7F);
  int8_t days = BCDTODEC(regs[3] & 0x3F);
  mode |= ((regs[3] >> 7) << 3) | ((regs[2] >> 7) << 2) | ((regs[1] >> 7) << 1);
  int8_t daynotdate = (regs[3] >> 6) & 0x01;
  switch (mode) {
    case 0xF:
      strcpy(s, "every sec");
      break;
    case 0xE:
      strcpy(s, "every min at ");
      break;
    case 0xC:
      strcpy(s, "every hour at ");
      break;
    case 0x8:
      strcpy(s, "every day at ");
      break;
    case 0x0:
      if (daynotdate) {
        // Day of week.
        const char dow[] = "SunMonTueWedThuFriSat";
        strcpy(s, "every ");
        s += strlen(s);
        days %= 7;  // Ensure 0 (Sun) to 6 (Sat).
        for (int i = 0; i < 3; ++i) {
          s[i] = dow[3*days + i];
        }
        s[3] = '\0';
        s += strlen(s);
        strcpy(s, " at ");
      } else {
        // Day of month.
        strcpy(s, "on the ");
        s += strlen(s);
        itoa2(days, s);
        s += strlen(s);
        strcpy(s, " of each month at ");
      }
      break;
    default:
      Serial.print("Invalid Alarm mode 0x");
      Serial.println(mode, HEX);
      break;      
    }
    s += strlen(s);
    // Print the actual time.
    switch (mode) {
      case 0x0:
      case 0x8:
        // Print hours.
        itoa2(hours, s);
        s += strlen(s);
        // Fall through.
      case 0xC:
        // Print mins.
        *s++ = ':';
        itoa2(mins, s);
        s += strlen(s);
        // Fall through.
      case 0xE:
        // Print secs.
        *s++ = ':';
        itoa2(secs, s);
        s += strlen(s);
        break;
      default:
        break;
    }
}

void print_registers(uint8_t *registers) {
  // Display all 19 hex registers.
  Serial.print("Regs: ");
  for (int i = 0; i < 19; ++i) {
    if (registers[i] < 16)  Serial.print("0");
    Serial.print(registers[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

char *CONTROL_NAMES[8] = {"#EO", "BSQ", "CNV", "RS2", "RS1", "INT", "A2E", "A1E"};
char *STATUS_NAMES[8]  = {"OSF", " x ", " x ", " x ", "EN3", "BSY", "A2F", "A1F"};

void print_registers_fancy(uint8_t *registers) {
  // Decode the entire state of the DS3231 to the terminal.
  // registers[19] is return from ds3231.getRegisters().
  
  // Print date/time.
  char s[70];  // Needed for longest sprint_bits.
  Serial.print("Time:");
  for (int i=0; i < 7; ++i) {
    Serial.print(' ');
    print2Digits(registers[i], 16);
  }
  // Format the date/time
  DateTime dt;
  dt = ds3231_regs_to_datetime(registers);
  sprint_datetime(dt, s);
  Serial.print(":   ");
  Serial.println(s);

  Serial.print("Alarm1:       ");
  for (int i = 7; i < 11; ++i) {
    Serial.print(' ');
    print2Digits(registers[i], 16);
  }
  // Format the alarm.
  sprint_alarm(registers + 7, s);
  Serial.print(":   ");
  Serial.println(s);
 
  Serial.print("Alarm2:          ");
  for (int i = 11; i < 14; ++i) {
    Serial.print(' ');
    print2Digits(registers[i], 16);
  }
  // Format Alarm2 (no seconds register).
  sprint_alarm(registers + 11, s, /* has seconds= */false);
  Serial.print(":   ");
  Serial.println(s);

  Serial.print("Contrl:  ");
  print2Digits(registers[14], 16);
  Serial.print(":   ");
  print_bits(registers[14], CONTROL_NAMES);
  Serial.println("");
  Serial.print("Status:  ");
  print2Digits(registers[15], 16);
  Serial.print(":   ");
  print_bits(registers[15], STATUS_NAMES);
  Serial.println("");
  Serial.print("Aging:   ");
  print2Digits(registers[16], 16);
  Serial.print(":   ");
  Serial.println(*(int8_t *)(registers + 16), 10);

  Serial.print("Temp: ");
  print2Digits(registers[17], 16);
  Serial.print(' ');
  print2Digits(registers[18], 16);
  Serial.print(":   ");
  Serial.print(*(int8_t *)(registers + 17), 10);
  Serial.print('.');
  Serial.println(25 * (registers[18] >> 6), 10);
  Serial.println("");
}


// -------------------------------------------------------------------
// Input commands over serial line

// ms to pause between polling calls.  0=disable polling.
int polling_interval = 0;

// Flag as to whether to respond to a falling edge on sqwvPin by reading time.
bool enable_sqwv_int = true;

void cmd_setup(void) {
  // Nothing to do?
  cmd_prompt();
}

byte atoi2(char *s) {
  // Convert two ascii digits to a uint8.
  return (s[1] - '0') + 10 * (s[0] - '0');
}

bool atob(char *s) {
  // Convert len-1 string to boolean.
  if (strlen(s) != 1 || (s[0] != '0' and s[0] != '1')) {
    Serial.print("Arg ");
    Serial.print(s);
    Serial.println(" is not 0 or 1.");
  }
  return (s[0] == '1');
}

uint32_t htoi(char *s) {
  // Convert hex string to unsigned long int.
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

DateTime parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  DateTime dt;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  // YYYY, MM, DD, 
  // hh, mm, ss
  return DateTime(2000 + atoi2(time_string + 2), atoi2(time_string + 4), atoi2(time_string + 6), 
                  atoi2(time_string + 8), atoi2(time_string + 10), atoi2(time_string + 12));
}

void print_enabled_disabled(char *s, int v) {
  Serial.print(s);
  Serial.print(" ");
  if (v == 0) Serial.println("disabled.");
  else Serial.println("enabled.");
}

DateTime parse_alarm_spec(char *arg, uint8_t *pmode, uint8_t alarm=1) {
  // hhmmss at the end of the arg.
  uint8_t ix = strlen(arg);
  uint8_t day = 1;
  uint8_t hr = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint8_t mode = DS3231_A1_Second;
  bool daynotdate = false;
  if (alarm == 1) {
    // No sec for alarm 2
    sec = atoi2(arg + ix - 2);
    ix -= 2;
  }
  if (ix >= 2) {
    min = atoi2(arg + ix - 2);
    mode = DS3231_A1_Minute;
    ix -= 2;
  }
  if (ix >= 2) {
    hr = atoi2(arg + ix - 2);
    mode = DS3231_A1_Hour;
    ix -= 2;
  }
  if (ix == 1) {
    // Weekday.
    daynotdate = true;
    day = 1 + (arg[ix - 1] - '0' + 6) % 7;   // 1 (Mon) .. 7 (Sun).
    mode = DS3231_A1_Day;
  } else if (ix == 2) {
    // Day of month
    day = atoi2(arg + ix - 2);
    mode = DS3231_A1_Date;
  } else if (ix != 0) {
    Serial.print("Alarm set misparse - ");
    Serial.println(arg);
  }
  if (alarm == 2) {
    // Alarm2 modes are shifted down 1 bit compared to Alarm1.
    mode >>= 1;
  }
  *pmode = mode;
  // Following RTC_DS3231, the May 2000 started on a Monday, so date == DoW.
  return DateTime(2000, 5, day, hr, min, sec);
}

void cmd_prompt() {
  Serial.println("***Cmd: Ann/Bx/Cx/D/Ex/Ix/Lxxx/Mxxx/Px/Qx/R/Sx/T1/Zxxx");
  //Serial.flush();
}

// Macro to set or clear bits specified by bitmask in a register.
#define SET_BIT_IN_REG_TO(reg, bitmask, val)  if (val) reg |= (bitmask); else reg &= ~(bitmask);

const int16_t ds3231_freqs[4] = {1, 1024, 4096, 8192};

void handle_cmd(char cmd, char * arg) {
  // Actually interpret and execute command, already broken up into 1 char cmd and arg string.
  // Number of characters in argument.
  uint8_t ctrl, status;  // In case we need them.
  bool b; // In case we need it.
  int value; // In case we need it.
  DateTime dt; // In case we need it.
  char s[64]; // In case we need it.
  uint8_t regs[19];  // In case we need it.
  int alen = strlen(arg);
  switch (cmd) {
    
   case 'A':
    // Get/set aging offset.
    if (alen) {
      value = atoi(arg);
      ds3231.setAging(value);
    }
    Serial.print("Aging offset=");
    Serial.println((int8_t)ds3231.getAging());
    break;
    
   case 'B':
    // Enable/disable battery square wave output (bit 6 of CONTROL).
    ctrl = ds3231.getControlReg();
    if (alen) {
      SET_BIT_IN_REG_TO(ctrl, 0x40, atob(arg));
      ds3231.setControlReg(ctrl);
    }
    print_enabled_disabled("BBSQWV", ctrl & 0x40);
    break;
    
   case 'C':
    // Enable/disable 32 kHz output (bit 3 of STATUS).
    status = ds3231.getStatusReg();
    if (alen) {
      SET_BIT_IN_REG_TO(status, 0x08, atob(arg));
      ds3231.setStatusReg(status);
    }
    print_enabled_disabled("32 kHz output", status & 0x08);
    break;
    
   case 'D':
    // Display all registers
    uint8_t registers[19];
    ds3231.getRegisters(registers, 19);
    print_registers_fancy(registers);
    break;
    
   case 'E':
    // Enable/disable master oscillator (not bit 7 of CONTROL).
    ctrl = ds3231.getControlReg();
    if (alen) {
      // The flag is actuall NOT(enable osc), so flip the value.
      SET_BIT_IN_REG_TO(ctrl, 0x80, !atob(arg));
      ds3231.setControlReg(ctrl);
    }
    print_enabled_disabled("Master osc", !(ctrl & 0x80));
    break;
  
   case 'G':
    // Print current microseconds skew vs. GPS, if any.
    print_gps_skew();
    break;
    
   case 'I':
    // Enable alarm interrupt outputs on SQWV (bit 2 of CONTROL).
    ctrl = ds3231.getControlReg();
    if (alen) {
      SET_BIT_IN_REG_TO(ctrl, 0x04, atob(arg));
      ds3231.setControlReg(ctrl);
    }
    print_enabled_disabled("Alarm interrupt outputs", ctrl & 0x04);
    break;
    
   case 'L':
    // Alarm 1
    if (alen == 1) {
      // Alarm1 enable/disable (bit 0 of CONTROL).
      ctrl = ds3231.getControlReg();
      SET_BIT_IN_REG_TO(ctrl, 0x01, atob(arg));
      ds3231.setControlReg(ctrl);
      print_enabled_disabled("A1IE", ctrl & 0x01);
    } else if (alen > 1) {
      uint8_t mode;
      dt = parse_alarm_spec(arg, &mode, /* alarm */ 1);
      sprint_datetime(dt, s);
      Serial.println(s);
      // setAlarm only works when INTCN (bit 2 of control) is set.
      ctrl = ds3231.getControlReg();
      if (!(ctrl & 0x04))  ds3231.setControlReg(ctrl | 0x04);
      ds3231.setAlarm1(dt, (Ds3231Alarm1Mode)mode);      
      // Restore conv bit (if we changed it), also A1E (set by setAlarm1).
      ds3231.setControlReg(ctrl);
    }
    // Read in the 4 bytes defining alarm1.
    ds3231.getRegisters(regs, 4, DS3231_ALARM1);
    sprint_alarm(regs, s);
    Serial.print("Alarm 1: ");
    Serial.println(s);
    break;
    
   case 'M':
    // Alarm 2
    if (alen == 1) {
      // Alarm2 enable/disable (bit 1 of CONTROL).
      ctrl = ds3231.getControlReg();
      SET_BIT_IN_REG_TO(ctrl, 0x02, atob(arg));
      ds3231.setControlReg(ctrl);
      print_enabled_disabled("A2IE", ctrl & 0x02);
    } else if (alen > 1) {
      uint8_t mode;
      dt = parse_alarm_spec(arg, &mode, /* alarm */ 2);
      sprint_datetime(dt, s);
      Serial.println(s);
      // setAlarm only works when INTCN (bit 2 of control) is set.
      ctrl = ds3231.getControlReg();
      if (!(ctrl & 0x04))  ds3231.setControlReg(ctrl | 0x04);
      ds3231.setAlarm2(dt, (Ds3231Alarm2Mode)mode);
      // Restore conv bit (if we changed it), also A2E (set by setAlarm2).
      ds3231.setControlReg(ctrl);
    }
    // Simulate 4-byte Alarm1 registers by making first byte = 0.
    regs[0] = 0;
    ds3231.getRegisters(regs + 1, 3, DS3231_ALARM2);
    sprint_alarm(regs, s);
    Serial.print("Alarm 2: ");
    Serial.println(s);
    break;

   case 'P':
    // Enable/disable continuous polling of time across I2C.
    if (alen) {
      polling_interval = atoi(arg);
    }
    Serial.print("Polling interval (ms, 0=disabled)=");
    Serial.println(polling_interval);
    break;
    
   case 'Q':
    // SQWV frequency. RS2:RS1 are CONTROL bits 4 and 3
    ctrl = ds3231.getControlReg();
    uint8_t rs;
    if (alen) {
      rs = arg[0] - '0';  // RS2:RS1
      ctrl = (ctrl & 0xE7) | ((rs & 0x03) << 3);
      ds3231.setControlReg(ctrl);
    }
    rs = (ctrl & 0x18) >> 3;
    Serial.print("SQWV freq=");
    Serial.println(ds3231_freqs[rs]);
    break;
    
   case 'R':
    // Reset OSF, A1F, A2F (bits 7, 1, 0 of STATUS).
    status = ds3231.getStatusReg();
    status &= ~0x83;
    ds3231.setStatusReg(status);
    Serial.println("OSF, A1F, A2F cleared.");
    break;
    
   case 'S':
    // Enable/disable read of time in response to falling edge on SQWV.
    if (alen) {
      enable_sqwv_int = atob(arg);
    }
    print_enabled_disabled("SQWV interrupt", enable_sqwv_int);
    break;
    
   case 'T':
    // Read or initiate temp read (bit 5 of CONTROL).
    if (alen && atob(arg)) {
      ctrl = ds3231.getControlReg();
      ctrl |= 0x20;
      ds3231.setControlReg(ctrl);
      Serial.println("Temp conversion initiated.");
    }
    Serial.print("Temp=");
    Serial.println(ds3231.getTemperature());
    break;
    
   case 'Z':
    // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
    if (alen) {
      if (alen != 14) {
        Serial.println("Bad format - Zyyyymmddhhmmss");
      } else {
        dt = DateTime(parse_time_string(arg));
        RTC_set_time(dt);
      }
    }
    serial_print_time(ds3231.now());
    break;

  }  
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

bool enable_rtc_updates = true;

void cmd_update(void) {
  int value;
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n' || new_char == '\r') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        handle_cmd(cmd0, cmd_buffer + 1);
        // Reprint command prompt.
        cmd_prompt();
        // Interaction just happened - tickle screensaver.
        sleep_tickle();
      }
      cmd_len = 0;
    } else {
      if (cmd_len < CMD_BUF_LEN) {
        cmd_buffer[cmd_len++] = new_char;
      }
    }
  }
}

// ========== GPS input ========

// -------- PPS interrupt input -------

// Wiring:
//   GPS tx out      -> GP5 (for Uart1 RX)
//   GPS 1PPS out    -> GP8
const int ppsPin = 8; // PPS output from GPS board
volatile unsigned long gps_micros = 0;

#ifdef ARDUINO_ARCH_RP2040

unsigned long my_micros(void) {
  return timer_hw->timelr;
}

void gpio_transition() {
  unsigned long now_micros = timer_hw->timelr;

  if (gpio_get_irq_event_mask(ppsPin)) {
    gps_micros = now_micros;
    gpio_acknowledge_irq(ppsPin, IO_IRQ_BANK0);
  }
}

void setup_interrupts(void) {
    irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_transition);
    // GPS PPS pin samples on rise.
    gpio_set_irq_enabled(ppsPin, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}
#else // !RP2040
#error "PPS interrupt handling not implemented on this architecture."
#endif // RP2040

// -------- GPS serial input -------

#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/

// 2nd UART on Pico - RX,TX is GP5,GP4 (or GP9,GP8)
#define SerialGPS Serial2

TinyGPS gps;

DateTime gps_now(class TinyGPS& gps) {
  unsigned long date, time, age_millis;
  gps.get_datetime(&date, &time, &age_millis);
  // We *could* add age_millis to get current time, but we really want the sync'd time from the last second.
  // date and time are stored as digit pairs in decimal i.e. ddmmyy and hhmmsscc.
  int centis = time % 100;
  //Serial.print("gps centis=");
  //Serial.println(centis);
  // it's always 0.
  time /= 100;
  uint8_t sec = time % 100;
  time /= 100;
  uint8_t min = time % 100;
  time /= 100;
  uint8_t hour = time % 100;
  uint16_t year = (date % 100) + 2000;
  date /= 100;
  uint8_t month = date % 100;
  date /= 100;
  uint8_t day = date % 100;
  return DateTime(year, month, day, hour, min, sec);
}

bool gps_time_valid(class TinyGPS& gps) {
  // It's only valid if it was updated within the past second.
  unsigned long time, age_millis;
  gps.get_datetime(0, &time, &age_millis);
  Serial.print("gps_time=");
  Serial.print(time);
  Serial.print(" age_millis=");
  Serial.println(age_millis);
  return (time != gps.GPS_INVALID_TIME) && (age_millis < 1000);
}

unsigned long last_gps_micros = 0;

void sync_time_from_GPS(void) {
  // This is called soon after an A1 transition is detected, so GPS unixtime is still current.
  Serial.println("sync_time_from_gps");
  if (gps_time_valid(gps)) {
    Serial.println("gps_time_valid");
    // We can assume that the last-stored time from the GPS messages is the second *preceeding* this mark,
    // so we add 1 second to get the actual time corresponding to the mark.
    // (potential race condition).
    delayMicroseconds(997000 - (my_micros() - gps_micros));
    RTC_set_time(DateTime(gps_now(gps).unixtime() + 1));
  }
}

void setup_GPS_serial(void) {
#ifdef ARDUINO_ARCH_RP2040
  // Configure Pico UART2
  Serial2.setRX(5);
  Serial2.setTX(4);
#endif
  //Serial2.begin(9600);
  SerialGPS.begin(9600);
}

void update_GPS_serial(void) {
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    //Serial.print(c);
    gps.encode(c);
  }
}

bool gps_active = false;

bool request_RTC_sync = false;

void setup_GPS(void) {
    pinMode(ppsPin, INPUT_PULLUP); // Set alarm pin as pullup
}

void update_GPS(void) {
  if (gps_micros != last_gps_micros) {
    last_gps_micros = gps_micros;
    // Request for sync e.g. from button press.
    if (request_RTC_sync) {
      Serial.println("Setting DS3231 from GPS");
      sync_time_from_GPS();
      request_RTC_sync = false;
    }
  }
  if((micros() - last_gps_micros) < 2000000) {
    gps_active = true;
  } else {
    gps_active = false;
  }
}



// ================ DS3231_emulator ================

// Emit SQWV on LED pin.
#ifdef ARDUINO_ARCH_RP2040
#define SQWV_PIN_RP2040  25 // On-board LED on Pico
#endif
#define SQWV_PIN 13

// Include Arduino Wire library for I2C
//#include <Wire.h>

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
#define DS3231_OUTPINSTATE    0x13 /// We use this byte to store the output pin state.  It's not in the chip.

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

#define EXT_10MHZ_INPUT

#ifdef EXT_10MHZ_INPUT
#warning "Using EXT_10MHZ_INPUT"
// PPS ticks come from external 10MHz input to RP2040 (i.e., OCXO)
// Use the PWM counter as an external-input counter.

#include "hardware/irq.h"
#include "hardware/pwm.h"

const char *clock_name = "10M";

// Where the frequency to count is coming in.
int timer_tenMHzInputPin = 7; // 19;  // Must be odd-numbered (PWM Chan B) pin.

uint8_t timer_sliceNum = 0;

uint32_t timer_count_max = 10000000; // 10 million
volatile uint32_t timer_count_max_this_time = 0;
volatile uint32_t timer_count = 0;
volatile uint32_t timer_pwmTop = (1L << 16);

void one_sec_callback() {
  // Make the callback to the RTC simulator.
  clock_tick();
  // Setup for next alarm, including cumulated nanos.
  timer_count -= timer_count_max_this_time;  // Should give zero.
  // Figure fine-tuning.
  cumulated_nanos += nanosecs_per_sec_trim;
  int centinanos_offset = cumulated_nanos / 100;
  cumulated_nanos -= centinanos_offset * 100;
  timer_count_max_this_time = timer_count_max + centinanos_offset;
  // Return to full-scale wrapping.
  timer_pwmTop = (1L << 16);
  pwm_set_wrap(timer_sliceNum, timer_pwmTop - 1);
}

void timer_on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(timer_sliceNum);
    // Wind on the underlying counter.
    timer_count += timer_pwmTop;

    if (timer_count >= timer_count_max_this_time) {
      one_sec_callback();
    } else if ((timer_count_max_this_time - timer_count) < (1L << 16)) {
      // Adjust top for last ramp.
      timer_pwmTop = (timer_count_max_this_time - timer_count);
      pwm_set_wrap(timer_sliceNum, timer_pwmTop - 1);
    }
}

void timer_setup_pwm_counter(uint8_t freq_pin) {
    // Configure the PWM circuit to count pulses on freq_pin.
    // Only the PWM B pins can be used as inputs.
    assert(pwm_gpio_to_channel(freq_pin) == PWM_CHAN_B);
    timer_sliceNum = pwm_gpio_to_slice_num(freq_pin);

    // Count once for every rising edge on PWM B input
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv(&cfg, 1);
    pwm_init(timer_sliceNum, &cfg, false);
    gpio_set_function(freq_pin, GPIO_FUNC_PWM);
    pwm_set_enabled(timer_sliceNum, true);
    pwm_set_wrap(timer_sliceNum, timer_pwmTop - 1);
    timer_count_max_this_time = timer_count_max;

    // Setup the wraparound interrupt.
    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(timer_sliceNum);
    pwm_set_irq_enabled(timer_sliceNum, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, timer_on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
}

void timer_setup(void) {
  // Setup regular timer interrupt.
  timer_setup_pwm_counter(timer_tenMHzInputPin);
}

void timer_reset_sync(void) {
  // Clear the count to zero, restart the PWM counter too.
  timer_count = 0;
  pwm_set_counter(timer_sliceNum, 0);
}

#else // !EXT_10MHZ_INPUT - use RP2040 built-in timer

const char *clock_name = "RP2";

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

#endif  // !EXT_10MHZ_INPUT

#endif  // RP2040

#ifdef ESP32

const char *clock_name = "E32";

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

// ----- Temperature sensor -----

#ifdef ARDUINO_ARCH_RP2040

// Read the on-chip temp sensor with ADC
// See
// https://learnembeddedsystems.co.uk/using-the-rp2040-on-board-temperature-sensor

#include "hardware/adc.h"

void temperature_setup(void) {
  // Configure ADC
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);  // 5th ADC channel == temp sensor.
}

int temperature_get(void) {
  // Return current temperature in quarter-Cs.
  // Temp sensor V_be is nominally 0.706 v at 27 degC 
  // with a slope of -1.721 mV/deg.
  // temp_quarter-Cs = 4 * (27 - ((3.3 * adc_read() / 4096) - 0.706) / 0.001721)
  //                 = 4 * (27 - (0.468 * adc_read() - 410.22))
  //                 = 4 * (437.22 - 0.468 * adc_read())
  //                 = 
  //  = 108 - (13.2 / 4096 * adc_read() * 581 + 4*410
  //  = 1748.9 - 1.872 * adc_read()
  //  =/= 1749 - (15/8) * adc_read()
  //float adc_volts = (3.3f * adc_read()) / (float)(1L<<12);
  //float temperature = 27 - (adc_volts - 0.706) / 0.001721;
  //return temperature;
  return 1749 - ((15 * adc_read()) >> 3);
}

#else // !RP2040

#ifdef ESP32
// EPS32 onboard temp sensor
// See 
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/peripherals/temp_sensor.html

//#include "driver/temperature_sensor.h"

void temperature_setup(void) {
    //temperature_sensor_handle_t temp_sensor = NULL;
    //temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    //temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    //temperature_sensor_enable(temp_sensor);
}

// Copied from esp32.../esp32-hal.h, undocumented??  Returns C.
float temperatureRead();

int temperature_get(void) {
  // Return current temperature in quarter-Cs.
    float tsens_value;
    //temperature_sensor_get_celsius(temp_sensor, &tsens_value);
    tsens_value = temperatureRead();
    return (int)(round( 4 * tsens_value));
}
    
#else // !ESP32

void temperature_setup(void) {
  // nothing.
}

int temperature_get(void) {
  // Return current temperature in quarter-Cs.
  // dummy.
  return (4* 25);
}

#endif  // !ESP32
#endif  // !RP2040

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

void ds3231_delta_aging(int delta) {
  registers[DS3231_AGING] = (delta + (int8_t)registers[DS3231_AGING]);
  registers_next[DS3231_AGING] = registers[DS3231_AGING];
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
  // Update temperature - quantizede to quarter-degrees.
  int quantized_temp = temperature_get();
  registers[DS3231_TEMPERATUREREG] = (quantized_temp >> 2);
  // Fractional degrees
  registers[DS3231_TEMPERATUREREG + 1] = (quantized_temp & 3) << 6;
  
  // Update the trim by ?1 ppb per aging offset.  Positive aging offset makes clock slower.
  // Maybe we should only do this when it is changed, i.e. check writes to AGING register?
  timer_update_trim_ppb(1 * (int)((int8_t)registers[DS3231_AGING]));
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
// Interrupt-updated micrs() at last tick (to help interface with explorer).
volatile uint32_t tick_micros = 0;

void clock_tick(void) {
  // Swap the registers double-buffer.  Do this and update output pin as fast as possible.
  uint8_t *tmp = registers;
  registers = registers_next;
  registers_next = tmp;
  // Update the output pin.
  digitalWrite(SQWV_PIN, registers[DS3231_OUTPINSTATE]);
  #ifdef SQWV_PIN_RP2040
  digitalWrite(SQWV_PIN_RP2040, registers[DS3231_OUTPINSTATE]);
  #endif

  if (!CHECK_BIT(registers[DS3231_CONTROL], DS3231_C_INTCN)) {
    // Set up semaphore for the oscillator pulse to be reset later.
    last_sqwv_millis = millis();
  }
  // Record the tick
  tick_happened = true;
  // To let explorer know that this happened.
  tick_micros = micros();
}

void print_gps_skew(void) {
    if (!gps_active) {
      Serial.println("GPS not active.");
    } else {
      Serial.print("RTC - GPS microseconds=");
      Serial.println((long int)(tick_micros - gps_micros));
    }
  }

// ------------- Display sleep (screensaver) -----------

bool display_on = true;

void wake_up_display(void) {
  Serial.println("wake_display");
#ifdef DISPLAY_ST7789
  // turn on backlite
  analogWrite(TFT_BACKLITE, 128);
#endif
  display_on = true;
  ds3231_display(ds3231, clock_name, gps_active);
}

void sleep_display(void) {
  Serial.println("sleep_display");
#ifdef DISPLAY_SH1107
  display.clearDisplay();
  display.display();
#else
  display.fillScreen(BLACK);
#endif
#ifdef DISPLAY_ST7789
  // turn off backlite
  analogWrite(TFT_BACKLITE, 0);
#endif
  display_on = false;
}

// Sleep display after 5 mins.
#define DISPLAY_SLEEP_MILLIS (1000 * 5 * 60)
uint32_t millis_last_action = 0;

void sleep_update(void) {
  // Check the time and sleep display if we've been idle.
  // We use millis since ds3231 time may be changing.
  if (millis_last_action && (millis() - millis_last_action) > DISPLAY_SLEEP_MILLIS) {
    if (display_on) {
      sleep_display();
    }
  }
}

void sleep_tickle(void) {
  // Reset display sleep countdown.
  Serial.println("tickle");
  millis_last_action = millis();
  if (!display_on) {
    wake_up_display();
  }
}

// ======================================================
// =========== Button management ==============
// ======================================================

#define NUM_BUTTONS 3   // on D9, D6, D5 on feather OLED wing are GPIO 9, 8, 7 on RP2040 Feather
#ifdef ARDUINO_ARCH_RP2040
#ifdef PIN_NEOPIXEL // i.e., this is a Feather RP2040
int button_pins[NUM_BUTTONS] = {9, 8, 7};
#else // RP2040 Pico
int button_pins[NUM_BUTTONS] = {18, 19, 20};
#endif
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

void buttons_setup(void) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    pinMode(button_pins[button], INPUT_PULLUP);
  }
  sleep_tickle();
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
    // Action - tickle the screensaver.
    sleep_tickle();
    switch(button) {
      case 0:
        if (long_press) {
          sleep_display();
        } else {
          request_RTC_sync = true;
          Serial.print("gps_micros=");
          Serial.print(gps_micros);
          Serial.print(" last_gps_micros=");
          Serial.println(last_gps_micros);
        }
        break;
      case 1:
        // Increase aging register
        if (long_press) {

        } else {
          ds3231_delta_aging(1);
        }
        break;
      case 2:
        if (long_press) {

        } else {
          ds3231_delta_aging(-1);
        }
        break;
    }
  }
}

// --------- I2C Delegate handlers (on EXT_I2C) --------------

volatile uint8_t cursor = 0;  // Address of next register access.

void receiveEvent(int howmany) {
  // Assume we got at least one data byte, and it's the cursor.
  bool registers_written = false;
  bool seconds_modified = false;
  cursor = EXT_I2C.read();

  // Any subsequent bytes are writes to that register.
  while (EXT_I2C.available()) {
    uint8_t x = EXT_I2C.read();
    if (cursor < NUM_REGISTERS) {
      registers[cursor] = x;
      registers_written = true;
      if (cursor == 0) {
        // We wrote the seconds register, reset the timer sync.
        seconds_modified = true;
      }
    }
    ++cursor;
  }
  if (registers_written) {
    // Maybe act on the new register values.
    ds3231_registers_updated(seconds_modified);
  } else {
    // Just set the cursor, assume this is write_then_read, populate output buffer.
    //EXT_I2C.write(registers + cursor, NUM_REGISTERS - cursor);
  }
}

void requestEvent() {
  // Send a stream of bytes back to master, starting from previous value of cursor until end of registers.
  // I *think* I2C indicates how many bytes it wants by sending a STOP after it's had enough.
  // but this doesn't appear to be visible through Wire.
  // So we send everything, and count on any excess being dropped.
  
  // It's critical that cursor has been set (via a preceding receiveEvent) before this runs.
  // The stock DS3231_RTC used i2c_dev->write_then_read, which (on the ESP32) actually ended
  // up not servicing the receive event until *after* the output buffer had been stuffed, so
  // the cursor was always one transaction behind (works OK on RP2040 Pico).
  // Modifying it to use write() followed by read() fixed it on ESP32.
  
  EXT_I2C.write(registers + cursor, NUM_REGISTERS - cursor);
}


// ------ emu_exp-specific read and write functions ------

void read_registers_fn(uint8_t reg, uint8_t* buffer, uint8_t num) {
  for (int i = 0; i < num; ++i) {
    buffer[i] = registers[reg + i];
  }
}

void write_registers_fn(uint8_t reg, const uint8_t* buffer, uint8_t num) {
  for (int i = 0; i < num; ++i) {
    registers[reg + i] = buffer[i];
  }
  bool seconds_modified = (reg == 0 && num > 0);
  // Maybe act on the new register values.
  ds3231_registers_updated(seconds_modified);  
}


// ----------------- Merged emu and exp setup() and loop() ------------------------

#define MAXWAIT_SERIAL 1000  // 200 = 2 seconds.
bool serial_available = false;
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
  delay(1000);
}

void setup()
{
#ifdef ARDUINO_ARCH_RP2040
  // Configure Pico RP2040 I2C
  // Internal I2C is used to communicate with I2C peripherals.
  INT_I2C.setSDA(int_sda_pin);
  INT_I2C.setSCL(int_scl_pin);
  // External I2C is the one we act as a peripheral (DS3231) on.
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

  // Configure SQWV output pin (typically LED).
  pinMode(SQWV_PIN, OUTPUT);
  digitalWrite(SQWV_PIN, HIGH);  // Default state.
#ifdef SQWV_PIN_RP2040
  pinMode(SQWV_PIN_RP2040, OUTPUT);
  digitalWrite(SQWV_PIN_RP2040, HIGH);  // Default state.
#endif

  open_serial();

  Serial.print(F("DS3231_emu_exp "));
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  INT_I2C.begin();
  setup_display();

  // GPS input
  setup_GPS_serial();
  setup_interrupts();

  // Setup ds3231 to use accessor functions instead of reading across I2C.
  ds3231.begin(&read_registers_fn, &write_registers_fn);

  // Emulator setup
  ds3231_setup();
  timer_setup();
  temperature_setup();

  // Explorer setup
  cmd_setup();
  buttons_setup();
}

int last_sec = 0;
uint32_t last_tick_micros = 0;
time_t secs_last_change = 0;

void loop() {
  
  // Emulator loop

  uint32_t now_millis = millis();
  // Do we need to reset the SQWV output 1Hz low pulse?
  if (last_sqwv_millis && (now_millis - last_sqwv_millis) >= sqwv_pulse_ms) {
    digitalWrite(SQWV_PIN, HIGH);
  #ifdef SQWV_PIN_RP2040
    // 2nd pin mirrors SQWV.
    digitalWrite(SQWV_PIN_RP2040, HIGH);
  #endif
    last_sqwv_millis = 0;  // Indicates no pulse waiting to be cleared.
  }
  // If the clock ticked, set up for next second.
  if (tick_happened) {
    ds3231_setup_next_tick();
    tick_happened = false;
    // Report seconds.
    //Serial.println(registers[0], HEX);
  }

  // Explorer loop
  
  cmd_update();
  buttons_update();

  if (polling_interval || (enable_sqwv_int && (last_tick_micros != tick_micros))) {
    last_tick_micros = tick_micros;
    DateTime dt = ds3231.now();
    int now_sec = dt.second();
    if (now_sec != last_sec) {
      last_sec = now_sec;
      //update_display(dt);
      if(display_on) {
        if(gps_active) {
          display_skew_us((long int)(tick_micros - gps_micros));
        }
        ds3231_display(ds3231, clock_name, gps_active);
      }
      //Serial.print("tick - gps=");
      //Serial.println((long int)(tick_micros - gps_micros));
    }
  }

  // Maybe sleep display
  sleep_update();

  // Handle input from GPS
  update_GPS_serial();
  update_GPS();

  delay(polling_interval);
}
