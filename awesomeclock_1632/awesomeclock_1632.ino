#include "HT1632.h"

#define DATA 2
#define WR   3
#define CS   4
#define CS2  5

// use this line for single matrix
HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS);
// use this line for two matrices!
//HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS, CS2);


// ---------------------------------
// RTC clock
// ---------------------------------

#include "RTClib.h"

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

RTC_DS1307 rtc;

void rtc_setup() {
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(rtc.now().unixtime());
}

// ---------------------------------
// Matrix display
// ---------------------------------

void matrix_setup() {
  Serial.begin(9600);
  Serial.println("** awesomeclock_1632 **");
  matrix.begin(HT1632_COMMON_16NMOS);  
  matrix.clearScreen();
  matrix.setBrightness(0);
}

void draw_seconds(int seconds, int minutes) {
  // Seconds in 6 groups of 2x5 pixels
  // Turn on when minutes is even, turn off when minutes is off.
  if (seconds == 0) {
    seconds = 60;
    minutes -= 1;
  }
  seconds -= 1;  // We light the bottom-left LED at seconds == 1.
  int y = 13 - (seconds % 5);
  int x = 3 + 3 * (seconds / 10) + ((seconds % 10) >= 5) + (seconds >= 30);
  if ((minutes & 1) == 0) {
    matrix.setPixel(x, y);
  } else {
    matrix.clrPixel(x, y);
  }
}

char state[4] = {' ', ' ', ' ', ' '};

uint8_t transition_state = 0;
const uint8_t num_transition_states = 7;

void draw_hours_minutes(int minutes, int hours) {
  matrix.setTextSize(1);    // size 1 == 8 pixels high
  char new_state[4];
  new_state[0] = '0' + (hours / 10);
  new_state[1] = '0' + (hours % 10);
  new_state[2] = '0' + (minutes / 10);
  new_state[3] = '0' + (minutes % 10);
  bool state_changed = new_state[3] != state[3];
  if (state_changed) {
    for (int i = 0; i < 4; ++i) {
      if (new_state[i] != state[i]) {
        int x = 6 * i + (i > 1);  // 2nd two chars are 1 pixel over.
        // undraw old character
        //matrix.setCursor(x, 0);
        //matrix.setTextColor(0);   // 'off' LEDs
        //matrix.write(state[i]);   
        // draw new character   
        //matrix.setCursor(x, 0);
        //matrix.setTextColor(1);   // 'on' LEDs
        //matrix.write(new_state[i]);      
        // Undraw previous transition.
        matrix.drawCharTx(x, 0, state[i], new_state[i], 0, 1, transition_state);
        // Draw next transition.
        matrix.drawCharTx(x, 0, state[i], new_state[i], 1, 1, transition_state + 1);
      }
    }
    // Update state change status for each rows.
    transition_state += 1;
    if (transition_state >= num_transition_states) {
      transition_state = 0;
      for (int i = 0; i < 4; ++i) {
        state[i] = new_state[i];
      }
      // Copying new_state to state implicitly releases state_changed.
    }
  }
}

void matrix_update(DateTime &now) {
  draw_hours_minutes(now.minute(), now.hour());
  draw_seconds(now.second(), now.minute());
  matrix.writeScreen();
}

// ---------------------------------
// Backlight
// ---------------------------------

// Config for backlight day/night mode.
const int light_low = 1;
const int light_high = 15;
const int hour_up = 7;
const int hour_down = 22;

void backlight_setup(void) {
  // Backlight
}

int brightness = 0;
int bright_tick = 0;
const int ticks_per_step = 1;

static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void backlight_update(int hour) {
  int target_brightness = light_low;
  if (hour >= hour_up && hour < hour_down) {
    target_brightness = light_high;
  }
  if (++bright_tick >= ticks_per_step) {
    // Slow down the brightness change steps.
    bright_tick = 0;
    int bright_delta = target_brightness - brightness;
    if (bright_delta) {
      brightness += sgn(bright_delta);
      matrix.setBrightness(brightness);
      Serial.print("brightness=");
      Serial.println(brightness);
    }
  }
}

// ---------------------------------
// Main
// ---------------------------------

void setup() {
  matrix_setup();
  rtc_setup();
  backlight_setup();
}

uint32_t last_time = 0;

void loop() {
  int num_delays = 0;
  while(rtc.now().unixtime() == last_time) {
    delay(20);
    ++num_delays;
  }
  //Serial.print("num_delays=");
  //Serial.println(num_delays);  // 2 or 3 when millis_to_wait is from 950.
  last_time = rtc.now().unixtime();
  int millis_start = millis();

  DateTime local_now = now_local();
  
  matrix_update(local_now);
  while(transition_state != 0) {
    matrix_update(local_now);
    delay(20);  
  }
  backlight_update(local_now.hour());
  
  int millis_to_wait = 950 - (millis() - millis_start);
  Serial.print("millis_to_wait=");
  Serial.println(millis_to_wait);
  delay(millis_to_wait);
}
