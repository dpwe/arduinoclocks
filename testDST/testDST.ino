/*
 * TestDST
 * 
 * Script to exercise the mini DST routine developed for Trinket.
 * 
 * You enter a UTC, and it reports the local time.
 */

// --------------------------
// DST routine under test (copied from awesomeclock_1632_trinket)
// --------------------------

typedef uint32_t time_t;

class Time
{
public:
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   date;
  uint8_t   month;
  uint16_t  year;

  Time(uint16_t year=2000, uint8_t mon=1, uint8_t date=1, uint8_t hour=0, uint8_t min=0, uint8_t sec=0);
};

Time::Time(uint16_t year, uint8_t mon, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec)
{
  this->year = year;
  this->month  = mon;
  this->date = date;
  this->hour = hour;
  this->min  = min;
  this->sec  = sec;
}

int dst_cache_year = -1;
int dst_start_day = 0;
int dst_end_day = 0;

uint8_t days_in_month(uint8_t month, bool leapyear) {
  if (month == 2) return 28 + leapyear;
  return 30 + ((month + (month > 7)) % 2);
}

int day_of_year(uint8_t year, uint8_t month, uint8_t day) {
  // Jan 1st is 0
  bool leapyear = (year % 4) == 0;
  int day_of_year = 0;
  for (int i = 1; i < month; ++i) day_of_year += days_in_month(i, leapyear);
  return day_of_year + (day - 1);
}

void calc_timechange_days(int year) {
  // Jan 1st 2000 was a Saturday.  So what day is March 1st this year? 0 = Sun.
  year -= 2000;
  uint8_t march_first_dow = (6 + 365L * year + ((year + 4) / 4) + 31 + 28) % 7;
  uint8_t second_sunday_date = 14 - ((march_first_dow - 1) % 7);
  dst_start_day = 31 + 28 + ((year % 4) == 0) + second_sunday_date - 1;
  // March and November are 245 days == 35.0 weeks apart, so 1st sunday in Nov is the same DOW
  dst_end_day = dst_start_day + 245 - 7;  // 1st sunday, not 2nd.
  dst_cache_year = year;
}

// ET DST begins at 2am local time on 2nd Sunday in March.
// At that point, local time is UTC-5, so 2am local is 7am UTC.
// The local time jumps to 3am.
// ET DST ends at 2am local time on 1st Sunday in November.
// At that point, local time is UTC-4, so 2am local is 6am UTC.
// The local time then slips back to 1am (UTC-5).

// Only works for UTC-1 to UTC-12
// so that 2am local is still the same date in UTC, even in DST,
// and logic only allows shift to local time to move date *backwards*.
#define STANDARD_TIME_DIFF_HOURS -5  // US ET

void make_localtime(Time &now) {
  if (dst_cache_year != now.year)  calc_timechange_days(now.year);
  // 2am local in North America is 2am + 4/5 in UTC
  int DoY = day_of_year(now.year - 2000, now.month, now.date);
  bool sprung_forward = (DoY > dst_start_day) ||
                        ((DoY == dst_start_day) && (now.hour >= 2 - STANDARD_TIME_DIFF_HOURS));
  bool fallen_back = (DoY > dst_end_day) ||
                      ((DoY == dst_end_day) && (now.hour >= 2 - (STANDARD_TIME_DIFF_HOURS + 1)));  // offset is DST
  bool is_dst = sprung_forward - fallen_back;
  int signed_hour = now.hour;  // Cast hour out of unsigned type to allow it to be < 0 after shifting.
  signed_hour += STANDARD_TIME_DIFF_HOURS + is_dst;
  if (signed_hour < 0) {
    signed_hour += 24;
    now.date -= 1;
    if (now.date == 0) {
      now.month -= 1;
      if (now.month == 0) {
        now.month = 12;
        now.year -= 1;
      }
      now.date = days_in_month(now.month, (now.year % 4) == 0);
    }
  }
  now.hour = signed_hour;
}

// -------------------------------------------------------------------
// Input commands over serial line

void printDigits(int digits)
{
  // utility function for digital clock display: prints leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void serial_print_tm(const Time &tm)
{
  Serial.print(tm.year);
  Serial.print("-");
  printDigits(tm.month);
  Serial.print("-");
  printDigits(tm.date);
  Serial.print(" ");
  printDigits(tm.hour);
  Serial.print(":");
  printDigits(tm.min);
  Serial.print(":");
  printDigits(tm.sec);
  Serial.println();
}


void cmd_setup(void) {
  // Nothing to do?
}

byte atoi2(char *s) {
  // Convert two ascii digits to a uint8.
  return (s[1] - '0') + 10 * (s[0] - '0');
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

Time parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  Time tm;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  tm.year = 2000 + atoi2(time_string + 2);
  tm.month = atoi2(time_string + 4);
  tm.date = atoi2(time_string + 6);
  tm.hour = atoi2(time_string + 8);
  tm.min = atoi2(time_string + 10);
  tm.sec = atoi2(time_string + 12);
  return tm;
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(void) {
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n' || new_char == '\r') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            if (strlen(cmd_buffer) < 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              // Do the test.
              Time t = parse_time_string(cmd_buffer + 1);
              make_localtime(t);
              serial_print_tm(t);
            }
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

// -------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("** TestDST **");

  cmd_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  cmd_update();
}
