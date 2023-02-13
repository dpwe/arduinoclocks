// esp32_pps_timer
//
// Reading an ESP32 timer in response to an interrupt from an external PPS pin.
//
// dan.ellis@gmail.com 2023-02-10

#include "driver/timer.h"

const int ppsPin = A2;      // (A2) PPS output from GPS board


// After https://github.com/espressif/esp-idf/blob/v4.3/examples/peripherals/timer_group/main/timer_group_example_main.c
// Use timer 0 of group 0 (first of 4 timers total).
// Set 8 divider for prescaler to get 0.1us counts.
#define TIMER_GROUP 0
// Perhaps timer 0 is being used for millis?  Stay clear of it?
// Note: TIMER_INDEX is hard-coded in register names in decimicros()
#define TIMER_INDEX 1

void timer_setup(void) {
  timer_group_t group = (timer_group_t)TIMER_GROUP;
  timer_idx_t timer = (timer_idx_t)TIMER_INDEX;
  timer_config_t config;
  config.divider = 4;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_START;
  config.alarm_en = TIMER_ALARM_DIS;
  config.auto_reload = TIMER_AUTORELOAD_EN;
  // default clock source is APB
  timer_init(group, timer, &config);
}

inline uint32_t decimicros_inline(void) { 
  *(uint32_t *)TIMG_T1UPDATE_REG(TIMER_GROUP) = 0; 
  return *(uint32_t *)TIMG_T1LO_REG(TIMER_GROUP); 
}

volatile uint32_t gps_micros = 0;
volatile uint32_t gps_period_micros = 0;

void gps_mark_isr(void) {
  uint32_t m = decimicros_inline();
  gps_period_micros = m - gps_micros;
  gps_micros = m;
}

void setup() {
  Serial.begin(9600);
  // Wait for Serial port to open
  while (!Serial) {
    delay(10);
  }
  Serial.print("** esp32_pps_timer **");

  timer_setup();

  pinMode(ppsPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ppsPin), gps_mark_isr, RISING);
}

uint32_t last_gps_micros = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (gps_micros != last_gps_micros) {
    last_gps_micros = gps_micros;
    Serial.print("gps_period=");
    Serial.println(gps_period_micros);
  }
}
