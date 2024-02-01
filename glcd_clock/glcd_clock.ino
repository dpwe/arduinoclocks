#include <RTClib.h>

/*
 *  glcd_clock (based on openGLCD Library)
 *  
 *  ks0108 128x64 LCD dot-matrix display is connected on *a lot* of pins, 
 *  as specified in openGLCD/config/ks0108/PinConfig_ks0108-Uno.h.
 *  
 *  This differs from the default openglcd because:
 *   - The data pins are in order (LCD D0 = Arduino D4 .. LCD D7 = Arduin D11) rather than nybble-swapped.
 *   - LCD E is moved to D12, freeing D3 ..
 *   - Backlight is on D3, which is PWM enabled
 *   
 *   For the clock RTC...
 *    - DS3231 SDA on A4
 *    - DS3231 SCL on A5
 *    - DS3231 SQWV on D2 (interrupt-capable).
 */

void assert(bool) {
  // nothing.
}

typedef long unsigned int time_t;

typedef unsigned long time_t;
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire
#include "RTClib.h"

// no font headers have to be included
#include <openGLCD.h>

// --------------------
// 1PPS interrupt
// --------------------

const int sqwPin = 2; // The number of the pin for DS3231 1PPS SQWV output

volatile unsigned long rtc_micros = 0;
volatile bool pending_RTC_interrupt = false;

void rtc_mark_isr(void)
{
  rtc_micros = micros();
  pending_RTC_interrupt = true;
}

void setup_interrupts() {
  pinMode(sqwPin, INPUT_PULLUP); // Set alarm pin as pullup
  // DS3231 is on falling edge.
  // attachInterrupt doesn't work for pin 13.
  attachInterrupt(digitalPinToInterrupt(sqwPin), rtc_mark_isr, FALLING);
} 

// ======================================================
// Track RTC times.
// ======================================================
#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

RTC_DS3231 rtc;

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

unsigned long int now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(rtc.now().unixtime());
}

void setup_RTC(void) {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  rtc.writeSqwPinMode(DS3231_SquareWave1Hz); // Place SQW pin into 1 Hz mode
  Serial.println(now_local());
  Serial.println("RTC OK");
}

void RTC_set_time(const DateTime &dt) {
  // Set the DS3231 time.
  Serial.print("Set RTC: ");
  serial_print_time(dt);
  rtc.adjust(dt);
  // Resync TimeLib
  //setTime(ds3231.now().unixtime());
}

//------------------------
// Display
//------------------------

void setup_display(void) {
  // Initialize the GLCD 

  GLCD.Init();

  // Set text area to be more centered.
  GLCD.DefineArea(14, 0, 127, 38);

  // Box around the second progress bar, 1 pixel separated.
  GLCD.DrawRect(2, 56, 124, 8);

  //GLCD.SelectFont(Arial_bold_14);
  //GLCD.SelectFont(TimesNewRoman16_bold);
  //GLCD.SelectFont(CalBlk36);
}

const char *fmt = "hh mm ";
uint8_t colon_visible = true;
uint8_t last_day = 0;

void update_display(const DateTime& dt) {

   // Day/Date at top.
   if (dt.day() != last_day) {
      last_day = dt.day();
      GLCD.SelectFont(System5x7);
      GLCD.CursorToXY(0, 0);
      char datefmt[] = "DDD YYYY-MM-DD";
      GLCD.print(dt.toString(datefmt));
      // Switch font back.
      GLCD.SelectFont(CalBlk36);
    }
    
    // Main time digits
    //GLCD.ClearScreen();
    GLCD.CursorToXY(0, 14);
    char buf[8];
    strcpy(buf, fmt);
    GLCD.print(dt.toString(buf));
    if (colon_visible) {
      buf[2] = '\0';
      GLCD.CursorToXY(GLCD.StringWidth(buf), 14);
      GLCD.print(":");
    }
    colon_visible = !colon_visible;

    // Seconds progress bar.
    uint8_t x_scale = 2;
    uint8_t height = 4;
    uint8_t base_y = 58;
    uint8_t base_x = 64 - x_scale * 30;
    // Bar width goes from 1 to 60 (instead of 0 to 59), so we have to work with one second ago.
    int prev_minute = dt.minute();
    int prev_second = dt.second() - 1;
    if (prev_second < 0) {
      prev_minute = (prev_minute + 59) % 60;
      prev_second += 60;
    }
    uint8_t width_l = x_scale * (1 + prev_second);
    // Draw/undraw transition occurs on second 1.  Second 0 is drawing the 60th tick on the line.
    if (prev_minute & 1) {
      // undraw bar.
      GLCD.FillRect(base_x, base_y, width_l, height, PIXEL_OFF);
    } else {
      // draw bar.
      GLCD.FillRect(base_x, base_y, width_l, height);
    }
}

// ---------------------------------
// Backlight
// ---------------------------------

// Config for backlight day/night mode.
const int light_low = 32;
int backlight_brightness = 255;
const int hour_up = 7;
const int hour_down = 22;

const int backlightPin = 3;

void setup_backlight(void) {
  // Backlight
  pinMode(backlightPin, OUTPUT);  // sets the pin as output
  analogWrite(backlightPin, light_low);
}

int target_brightness = 0;

void update_backlight_time(int hour) {
  target_brightness = light_low;
  if (hour >= hour_up && hour < hour_down) {
    target_brightness = backlight_brightness;
  }
}

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

int brightness = 0;
int brightness_tick = 0;
const int ticks_per_step = 1;

void update_backlight() {
  int bright_delta = target_brightness - brightness;
  if (bright_delta) {
    if (++brightness_tick >= ticks_per_step) {
      brightness_tick = 0;
      brightness += (bright_delta >> 6) + sgn(bright_delta);
      analogWrite(backlightPin, brightness);
      //Serial.print("brightness=");
      //Serial.println(brightness);
    }
  }
}

// ---------------------- CLI -----------------------------------------
// Input commands over serial line

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
  while (*s) {
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

void print_enabled_disabled(const char *s, int v) {
  Serial.print(s);
  Serial.print(" ");
  if (v == 0) Serial.println("disabled.");
  else Serial.println("enabled.");
}

DateTime parse_alarm_spec(char *arg, uint8_t *pmode, uint8_t alarm = 1) {
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
    day = 1 + (arg[ix - 1] - '0' + 6) % 7;  // 1 (Mon) .. 7 (Sun).
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
  Serial.println("***Cmd: Zxxx");
  //Serial.flush();
}

// Macro to set or clear bits specified by bitmask in a register.
#define SET_BIT_IN_REG_TO(reg, bitmask, val) \
  if (val) reg |= (bitmask); \
  else reg &= ~(bitmask);

const int16_t ds3231_freqs[4] = { 1, 1024, 4096, 8192 };

// Trim subtracted from predelay on GPS sync.
int32_t predelay_trim_us = 0;


void handle_cmd(char cmd, char *arg) {
  // Actually interpret and execute command, already broken up into 1 char cmd and arg string.
  // Number of characters in argument.
  uint8_t ctrl, status;  // In case we need them.
  bool b;                // In case we need it.
  int value;             // In case we need it.
  DateTime dt;           // In case we need it.
  char s[64];            // In case we need it.
  uint8_t regs[19];      // In case we need it.
  int alen = strlen(arg);
  switch (cmd) {


    case 'O':
      // Set active backlight brightness
      if (alen) {
        backlight_brightness = atoi(arg);
      }
      Serial.print("Backlight brightness (0..255)=");
      Serial.println(backlight_brightness);
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
      serial_print_time(rtc.now());
      break;
  }
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

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
        if (cmd0 >= 'a') cmd0 -= ('a' - 'A');
        handle_cmd(cmd0, cmd_buffer + 1);
        // Reprint command prompt.
        cmd_prompt();
        // Interaction just happened - tickle screensaver.
        //sleep_tickle();
      }
      cmd_len = 0;
    } else {
      if (cmd_len < CMD_BUF_LEN) {
        cmd_buffer[cmd_len++] = new_char;
      }
    }
  }
}

// -------------------------
// Main
// -------------------------
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
  open_serial();

  Serial.println("glcd_clock");

  setup_RTC();
  setup_interrupts();
  setup_backlight();

  setup_display();

  cmd_setup();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  if (pending_RTC_interrupt) {
    pending_RTC_interrupt = false;
    time_t now_now = now_local();
    DateTime dt(now_now);
    update_backlight_time(dt.hour());
    update_display(dt);
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // Backlight update is at full-speed, not 1 Hz.
  update_backlight();
  cmd_update();
  delay(10);
}
