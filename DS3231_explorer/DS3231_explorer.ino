// DS3231_explorer
//
// Allows reading and writing all the bits of a DS3231 RTC.
// Also continually polls the clock (for P1) and displays current time on attached SSD1351 display.
// Also listens for SQWV on D3 input and reads time on falling edges (for S1).
//
// dpwe@google.com 2022-12-31

// Command set:
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

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
#include <RTClib.h>         // Adafruit; defines RTC_DS3231

// ------------- Display ---------------

#include <Adafruit_GFX.h>

// #define DISPLAY_SSD1351
#define DISPLAY_ST7789

#ifdef DISPLAY_SSD1351

// Arduino - Expect SQWV input on D3
const uint8_t sqwvPin = 3;

#include <Adafruit_SSD1351.h>
// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// Hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. 
#define DC_PIN   4
#define CS_PIN   5
#define RST_PIN  6
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

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

// ESP32-S3 TFT - Expect SQWV input on A0
const uint8_t sqwvPin = A0;

#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 135 // Change this to 96 for 1.27" OLED.

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int backlightPin = TFT_BACKLITE;  // PWM output to drive dimmable backlight

#define WHITE ST77XX_WHITE
#define BLACK ST77XX_BLACK

#endif


void setup_display(void) {
#ifdef DISPLAY_SSD1351
  tft.begin();
#endif
#ifdef DISPLAY_ST7789
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  analogWrite(TFT_BACKLITE, 128);

  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
#endif

  tft.fillScreen(BLACK);

  // text display 
  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(0,0);
  tft.print("DS3231_explorer");
}

void sprint_datetime(const DateTime &dt, char *s) {
  // s must have 20 bytes.
  strcpy(s, "YYYY-MM-DD hh:mm:ss");
  dt.toString(s);
}

void update_display(DateTime &dt) {
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(0,0);

  // Format time.
  // toString overwrites a format string with the actual date/time.
  char timestr[20];
  sprint_datetime(dt, timestr);
  //tft.fillRect(0, 0, 128, 16, BLACK);
  tft.print(timestr);
}

void print_bits_tft(uint8_t val, char* names[8]) {
  // Print a bit set using an array of names.
  uint8_t mask = 0x80;  // Actually start with top bit.
  for (uint8_t bit = 0; bit < 8; ++bit) {
    bool bitval = ((val & mask) > 0);
    // "Set" bits are printed in reverse video.
    if (bitval)   tft.setTextColor(BLACK, WHITE);
    else   tft.setTextColor(WHITE, BLACK);
    tft.print(names[bit]);
    if (bitval)   tft.setTextColor(WHITE, BLACK);
    tft.print(" ");
    mask >>= 1;
  }
}

// -------- print redirection --------

void print(char *str, bool to_tft=false) {
  if (to_tft) {
    tft.print(str);
  } else {
    Serial.print(str);
  }
}

void println(char *str="", bool to_tft=false) {
  if (to_tft) {
    tft.println(str);
  } else {
    Serial.println(str);
  }
}

void print(int i, int base, bool to_tft=false) {
  if (to_tft) {
    tft.print(i, base);
  } else {
    Serial.print(i, base);
  }
}

void println(int i, int base, bool to_tft=false) {
  if (to_tft) {
    tft.println(i, base);
  } else {
    Serial.println(i, base);
  }
}

void print(char c, bool to_tft=false) {
  if (to_tft) {
    tft.print(c);
  } else {
    Serial.print(c);
  }
}

void println(char c, bool to_tft=false) {
  if (to_tft) {
    tft.println(c);
  } else {
    Serial.println(c);
  }
}

// -------------- Interrupt ---------------

volatile unsigned long rtc_micros = 0;

void rtc_mark_isr(void)
{
  rtc_micros = micros();
}

void setup_interrupts() {
  // DS3231 is on falling edge.
  attachInterrupt(digitalPinToInterrupt(sqwvPin), rtc_mark_isr, FALLING);
} 


// -------------- Time --------------------

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
// US Pacific Time Zone (Las Vegas, Los Angeles)
//TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};
//TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};
Timezone myTZ(myDST, mySTD);

#define CLOCK_ADDRESS 0x68

#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUS 0x0F ///< Status register
#define DS3231_AGING 0x10     ///< Aging offset register
#define DS3231_TEMPERATUREREG 0x11 ///< Temperature register (high byte - low byte is at 0x12), 10-bit
                              ///< temperature value

#define time_t uint32_t

RTC_DS3231 ds3231;

void printByte(byte val)
{
  if (val < 16) {
    // Add a leading zero.
    Serial.print("0");
  }
  Serial.print(val, HEX);
}

void printBytes(byte *bytes, int n_bytes)
{
  for(int i = 0; i < n_bytes; ++i) {
    printByte(bytes[i]);
    Serial.print(" ");
  }
 Serial.println("");
}

void serial_print_time(const DateTime &dt) {
  // toString overwrites a format string with the actual date/time.
  char s[20];
  sprint_datetime(dt, s);
  Serial.println(s);
}

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(now());
}

// getExternalTime is declared to expect a function returning a (signed) long int.
//time_t 
long int RTC_utc_get(void) {
  return ds3231.now().unixtime();
}

void RTC_set_time(const DateTime& dt) {
  // Set the DS3231 time.
  Serial.print("Set RTC: ");
  serial_print_time(dt);
  ds3231.adjust(dt);
  // Resync TimeLib
  setTime(ds3231.now().unixtime());
}

// ------------------------
void print2Digits(int digits, int base=10, bool to_tft=false)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < base)
    print('0', to_tft);
  print(digits, base, to_tft);
}

void itoa2(int num, char *s, int base=10) {
#define DTOA(d) ((d < 10) ? ('0' + d) : ('A' + d - 10))
  *s++ = DTOA(num / base);
  *s++ = DTOA(num % base);
  *s++ = '\0';
}

// ----- low-level I2C access ---------

void wire_tx(byte i2c_address, byte address_offset, byte num_bytes, const byte *payload) {
  Wire.beginTransmission(i2c_address);
  Wire.write(address_offset);
  for (int i = 0; i < num_bytes; ++i) {
    Wire.write(payload[i]);
  }
  Wire.endTransmission();
}

void wire_rx(byte i2c_address, byte address_offset, byte num_bytes, byte *payload) {
  Wire.beginTransmission(i2c_address);
  Wire.write(address_offset);
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, num_bytes); 
  for (int i = 0; i < num_bytes; ++i) {
    payload[i] = Wire.read();
  }
}

void write_register(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t read_register(uint8_t reg) {
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(CLOCK_ADDRESS, 1);
  uint8_t val = Wire.read();
  return val;
}

void get_registers(uint8_t *registers) {
  // Read all 19 hex registers and print out.
  // registers must point to 19 free bytes.
  wire_rx(CLOCK_ADDRESS, 0, 19, registers);
}

void print_registers(uint8_t *registers) {
  // Display all 19 hex registers.
  Serial.print("Regs: ");
  wire_rx(CLOCK_ADDRESS, 0, 19, registers);
  for (int i = 0; i < 19; ++i) {
    if (registers[i] < 16)  Serial.print("0");
    Serial.print(registers[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
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

void print_bits(uint8_t val, char* names[8], bool to_tft) {
  // Print bits.  Special case for to_tft.
  if (to_tft) {
    print_bits_tft(val, names);
  } else {
    char s[70];
    sprint_bits(val, names, s);
    Serial.print(s);
  }
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
        for (int i = 0; i < 3; ++i) {
          s[i] = dow[3*(days - 1) + i];
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

char *CONTROL_NAMES[8] = {"#EO", "BSQ", "CNV", "RS2", "RS1", "INT", "A2E", "A1E"};
char *STATUS_NAMES[8] = {"OSF", " x ", " x ", " x ", "EN3", "BSY", "A2F", "A1F"};

void print_registers_fancy(uint8_t *registers, bool to_tft=false) {
  // registers is return from get_registers.
  // Print date/time.
  char s[70];  // Needed for longest sprint_bits.
  print("Time:", to_tft);
  for (int i=0; i < 7; ++i) {
    print(' ', to_tft);
    print2Digits(registers[i], 16, to_tft);
  }
  // Format the date/time
  DateTime dt;
  dt = ds3231_regs_to_datetime(registers);
  sprint_datetime(dt, s);
  if (to_tft) println("", to_tft);
  else print(": ", to_tft);
  print("  ", to_tft);
  println(s, to_tft);

  print("Alarm1: ", to_tft);
  if (!to_tft) print("      ", to_tft);
  for (int i = 7; i < 11; ++i) {
    print(' ', to_tft);
    print2Digits(registers[i], 16, to_tft);
  }
  // Format the alarm.
  sprint_alarm(registers + 7, s);
  if (to_tft) println("", to_tft);
  else print(": ", to_tft);
  print("  ", to_tft);
  println(s, to_tft);
 
  print("Alarm2: ", to_tft);
  if (!to_tft)  print("         ", to_tft);
  for (int i = 11; i < 14; ++i) {
    print(' ', to_tft);
    print2Digits(registers[i], 16, to_tft);
  }
  // Format Alarm2 (no seconds register).
  sprint_alarm(registers + 11, s, /* has seconds= */false);
  if (to_tft) println("", to_tft);
  else print(": ", to_tft);
  print("  ", to_tft);
  println(s, to_tft);

  print("Contrl:  ", to_tft);
  print2Digits(registers[14], 16, to_tft);
  if (to_tft) println("", to_tft);
  else print(": ", to_tft);
  print("  ", to_tft);
  print_bits(registers[14], CONTROL_NAMES, to_tft);
  println("", to_tft);
  print("Status:  ", to_tft);
  print2Digits(registers[15], 16, to_tft);
  if (to_tft) println("", to_tft);
  else print(": ", to_tft);
  print("  ", to_tft);
  print_bits(registers[15], STATUS_NAMES, to_tft);
  println("", to_tft);
  print("Aging:   ", to_tft);
  print2Digits(registers[16], 16, to_tft);
  print(":   ", to_tft);
  println(*(int8_t *)(registers + 16), 10, to_tft);

  print("Temp: ", to_tft);
  print2Digits(registers[17], 16, to_tft);
  print(' ', to_tft);
  print2Digits(registers[18], 16, to_tft);
  print(":   ", to_tft);
  print(*(int8_t *)(registers + 17), 10, to_tft);
  print('.', to_tft);
  println(25 * (registers[18] >> 6), 10, to_tft);
  println("", to_tft);
}

void update_display_with_registers() {
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(0,0);

  uint8_t registers[19];
  get_registers(registers);
  print_registers_fancy(registers, /* to_tft= */true);
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
    day = arg[ix] - '0';
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
  return DateTime(2000, 1, day, hr, min, sec);
}

void cmd_prompt() {
  Serial.println("***Cmd: Ann/Bx/Cx/D/Ex/Ix/Lxxx/Mxxx/Px/Qx/R/Sx/T1/Zxxx");
  //Serial.flush();
}

const int16_t ds3231_freqs[4] = {1, 1024, 4096, 8192};

void handle_cmd(char cmd, char * arg) {
  // Actually interpret and execute command, already broken up into 1 char cmd and arg string.
  // Number of characters in argument.
  uint8_t ctrl, status;  // In case we need them.
  bool b; // In case we need it.
  int value; // In case we need it.
  DateTime dt; // In case we need it.
  char s[64]; // In case we need it.
  uint8_t regs[4];  // In case we need it.
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
    ctrl = read_register(DS3231_CONTROL);
    if (alen) {
      b = atob(arg);
      if (b) ctrl |= 0x40;
      else   ctrl &= (0xFF - 0x40);
      write_register(DS3231_CONTROL, ctrl);
    }
    print_enabled_disabled("BBSQWV", ctrl & 0x40);
    break;
    
   case 'C':
    // Enable/disable 32 kHz output (bit 3 of STATUS).
    status = read_register(DS3231_STATUS);
    if (alen) {
      b = atob(arg);
      if (b) status |= 0x08;
      else   status &= (0xFF - 0x08);
      write_register(DS3231_STATUS, status);
    }
    print_enabled_disabled("32 kHz output", status & 0x08);
    break;
    
   case 'D':
    // Display all registers
    uint8_t registers[19];
    get_registers(registers);
    print_registers_fancy(registers);
    //dt = ds3231_regs_to_datetime(registers);
    //sprint_datetime(dt, s);  
    //Serial.println(s);
    break;
    
   case 'E':
    // Enable/disable master oscillator (not bit 7 of CONTROL).
    ctrl = read_register(DS3231_CONTROL);
    if (alen) {
      // The flag is actuall NOT(enable osc), so flip the value.
      b = !atob(arg);
      if (b) ctrl |= 0x80;
      else   ctrl &= (0xFF - 0x80);
      write_register(DS3231_CONTROL, ctrl);
    }
    print_enabled_disabled("Master osc", !(ctrl & 0x80));
    break;
    
   case 'I':
    // Enable alarm interrupt outputs on SQWV (bit 2 of CONTROL).
    ctrl = read_register(DS3231_CONTROL);
    if (alen) {
      b = atob(arg);
      if (b) ctrl |= 0x04;
      else   ctrl &= (0xFF - 0x04);
      write_register(DS3231_CONTROL, ctrl);
    }
    print_enabled_disabled("Alarm interrupt outputs", ctrl & 0x04);
    break;
    
   case 'L':
    // Alarm 1
    if (alen == 1) {
      // Alarm1 enable/disable (bit 0 of CONTROL).
      ctrl = read_register(DS3231_CONTROL);
      b = atob(arg);
      if (b) ctrl |= 0x01;
      else   ctrl &= (0xFF - 0x01);
      write_register(DS3231_CONTROL, ctrl);
      print_enabled_disabled("A1IE", ctrl & 0x01);
    } else if (alen > 1) {
      uint8_t mode;
      dt = parse_alarm_spec(arg, &mode, /* alarm */ 1);
      sprint_datetime(dt, s);
      Serial.println(s);
      // setAlarm only works when INTCN (bit 2 of control) is set.
      ctrl = read_register(DS3231_CONTROL);
      if (!(ctrl & 0x04))  write_register(DS3231_CONTROL, ctrl | 0x04);
      ds3231.setAlarm1(dt, (Ds3231Alarm1Mode)mode);      
      // Restore conv bit
      if (!(ctrl & 0x04))  write_register(DS3231_CONTROL, ctrl);
    }
    wire_rx(CLOCK_ADDRESS, 7, 4, regs);
    sprint_alarm(regs, s);
    Serial.print("Alarm 1: ");
    Serial.println(s);
    break;
    
   case 'M':
    // Alarm 2
    if (alen == 1) {
      // Alarm2 enable/disable (bit 1 of CONTROL).
      ctrl = read_register(DS3231_CONTROL);
      b = atob(arg);
      if (b) ctrl |= 0x02;
      else   ctrl &= (0xFF - 0x02);
      write_register(DS3231_CONTROL, ctrl);
      print_enabled_disabled("A2IE", ctrl & 0x02);
    } else if (alen > 1) {
      uint8_t mode;
      dt = parse_alarm_spec(arg, &mode, /* alarm */ 2);
      sprint_datetime(dt, s);
      Serial.println(s);
      // setAlarm only works when INTCN (bit 2 of control) is set.
      ctrl = read_register(DS3231_CONTROL);
      if (!(ctrl & 0x04))  write_register(DS3231_CONTROL, ctrl | 0x04);
      ds3231.setAlarm2(dt, (Ds3231Alarm2Mode)mode);
      // Restore conv bit
      if (!(ctrl & 0x04))  write_register(DS3231_CONTROL, ctrl);
    }
    regs[0] = 0;
    wire_rx(CLOCK_ADDRESS, 11, 3, regs + 1);
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
    ctrl = read_register(DS3231_CONTROL);
    uint8_t rs;
    if (alen) {
      rs = arg[0] - '0';  // RS2:RS1
      ctrl = (ctrl & 0xE7) | ((rs & 0x03) << 3);
      write_register(DS3231_CONTROL, ctrl);
    }
    rs = (ctrl & 0x18) >> 3;
    Serial.print("SQWV freq=");
    Serial.println(ds3231_freqs[rs]);
    break;
    
   case 'R':
    // Reset OSF, A1F, A2F (bits 7, 1, 0 of STATUS).
    status = read_register(DS3231_STATUS);
    status &= (0xFF - 0x83);
    write_register(DS3231_STATUS, status);
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
      ctrl = read_register(DS3231_CONTROL);
      ctrl |= 0x20;
      write_register(DS3231_CONTROL, ctrl);
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

// -----------------------------------------

void setup()
{
  // Configure the PPS input pin
  pinMode(sqwvPin, INPUT_PULLUP); // Set alarm pin as pullup

  // Start the I2C interface

  Serial.begin(9600);
  // Wait for Serial port to open
  while (!Serial) {
    delay(10);
  }
  //delay(500);

  Serial.print(F("DS3231_explorer "));
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  // Wire is initialized inside OLED display
  //Wire.begin();

  setup_display();

  if (!ds3231.begin(&Wire)) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  setSyncProvider(RTC_utc_get);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet)
     Serial.println("Unable to sync with the RTC");
  else
     Serial.println("RTC has set the system time");

  Serial.print("DS3231 ");
  uint8_t registers[19];
  get_registers(registers);
  print_registers(registers);
  Serial.println();
  //print_registers_fancy(registers);

  cmd_setup();
  setup_interrupts();
}

unsigned long last_rtc_micros = 0;

int last_sec = 0;
void loop()
{

  cmd_update();

  if (polling_interval || (enable_sqwv_int && (last_rtc_micros != rtc_micros))) {
    last_rtc_micros = rtc_micros;
    DateTime dt = DateTime(myTZ.toLocal(ds3231.now().unixtime()));
    int now_sec = dt.second();
    if (now_sec != last_sec) {
      last_sec = now_sec;
      //update_display(dt);
      update_display_with_registers();
    }
  }
  
  delay(polling_interval);
}
