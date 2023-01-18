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


#include <SPI.h>
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

#include <RTClib.h>         // Adafruit; defines RTC_DS3231

#ifdef ARDUINO_ARCH_RP2040
  // Hardware limits mean that pins 24 and 25 (A4 and A5, favored choice for ext_i2)
  // must be assigned to I2C0 aka Wire on RP2040.  Wire1 is only for pins 2(n+1), 2(n+1)+1.
  const int ext_sda_pin = 24;
  const int ext_scl_pin = 25;
  #define EXT_I2C Wire
  #define INT_I2C Wire1

  #define DISPLAY_SH1107  // 128x64 mono OLED in Feather stack

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
  
  #define SCREEN_WIDTH  128
  #define SCREEN_HEIGHT 64
  #define SIZE_1X

  Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, &Wire1);  // Wire1 is the internal I2C on Feather RP2040

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
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("DS3231_explorer");
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

void ds3231_display(class RTC_DS3231& ds3231) {
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
      }
      cmd_len = 0;
    } else {
      if (cmd_len < CMD_BUF_LEN) {
        cmd_buffer[cmd_len++] = new_char;
      }
    }
  }
}

// ================ DS3231_emulator ================

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
// Interrupt-updated micrs() at last tick (to help interface with explorer).
volatile uint32_t tick_micros = 0;

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
  tick_micros = micros();
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

// Delegate I2C bus config.  This acts like the DS3231 chip.
#define SDA_PIN A4
#define SCL_PIN A5
#define I2C_FREQ 100000

void setup()
{
  // Configure SQWV output pin (typically LED).
  pinMode(SQWV_PIN, OUTPUT);
  digitalWrite(SQWV_PIN, HIGH);  // Default state.
  
  Serial.begin(9600);
  // Wait for Serial port to open
  while (!Serial) {
    delay(10);
  }

  Serial.print(F("DS3231_emu_exp "));
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

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

  // Setup ds3231 to use accessor functions.
  ds3231.begin(&read_registers_fn, &write_registers_fn);

  // Emulator setup
  ds3231_setup();
  timer_setup();

  // Explorer setup
  setup_display();
  cmd_setup();
}

int last_sec = 0;
uint32_t last_tick_micros = 0;

void loop() {
  
  // Emulator loop

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
    //Serial.println(registers[0], HEX);
  }

  // Explorer loop
  
  cmd_update();

  if (polling_interval || (enable_sqwv_int && (last_tick_micros != tick_micros))) {
    last_tick_micros = tick_micros;
    DateTime dt = ds3231.now();
    int now_sec = dt.second();
    if (now_sec != last_sec) {
      last_sec = now_sec;
      //update_display(dt);
      ds3231_display(ds3231);
    }
  }
  delay(polling_interval);
}
