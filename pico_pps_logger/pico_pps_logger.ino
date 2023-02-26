// pico_pps_logger
//
// Log relative timings of multiple periodic edges
// intended for use with PPS edges from various clocks.
// Handles multiple clocks
// Emits results to serial, to be logged on a host.
//
// 2023-02-26 dan.ellis@gmail.com

// This version runs on the RPi Pico RP2040 chip.

#include <Wire.h>
// Make efault I2C ("Wire") actually be Wire1 to match default on RP2040 Feather
#define Wire Wire1

// Pico pin setup, chosen to match pins used on RP2040 Feather (Wire1)
const int sda_pin = 2;
const int scl_pin = 3;

const int ledPin = 25; // On-board LED on Pico

const int gps_pps_pin = 4; // Master input.

// =============================================================
// PPS change time recording.
// =============================================================

typedef struct pps_pin_info {
  int pin;      // Which GPIO pin the PPS occurs on.
  bool rising;  // true if rising edge is mark; false if falling edge is mark
  bool active;  // true if this pps pin is active.
} pps_pin_info_t;

#define MAX_PPSS 4
pps_pin_info_t pps_pins[MAX_PPSS] = {
  {gps_pps_pin, true, true},  // pin 4 = GPS PPS (rising edge)
  {5, false, true},  // pin 5 = Real DS3231 (falling edge)
  {6, false, true},
  {7, false, true},
};
// Last recorded event times for each input.
volatile uint32_t last_pps_time[MAX_PPSS];
// Snapshot of last_pps_time for pps[0] transition.
volatile uint32_t snapshot_pps_time[MAX_PPSS];

void gpio_transition() {
  unsigned long now_micros = timer_hw->timelr;
  for (int pps = 0; pps < MAX_PPSS; ++pps) {
    int pps_pin = pps_pins[pps].pin;
    if (gpio_get_irq_event_mask(pps_pin)) {
      gpio_acknowledge_irq(pps_pin, IO_IRQ_BANK0);
      // pps 0 is a special case, copy all the currrent times.
      if (pps == 0) {
        for (int pps2 = 0; pps2 < MAX_PPSS; ++pps2) {
          snapshot_pps_time[pps2] = last_pps_time[pps2];
        }
      }
      // Update the last event time for this pin.
      last_pps_time[pps] = now_micros;
    }
  }
}

void setup_interrupts(void) {
  irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_transition);
  for (int pps = 0; pps < MAX_PPSS; ++pps) {
    if (pps_pins[pps].active) {
      int pps_pin = pps_pins[pps].pin;
      gpio_init(pps_pin); 
      gpio_set_dir(pps_pin, GPIO_IN);
      if (pps_pins[pps].rising) {
        gpio_set_irq_enabled(pps_pin, GPIO_IRQ_EDGE_RISE, true);
      } else {
        gpio_set_irq_enabled(pps_pin, GPIO_IRQ_EDGE_FALL, true);
      }
    }
  irq_set_enabled(IO_IRQ_BANK0, true);
  }
}

// -------------------------------------------------------------------
// Input commands over serial line

void cmd_setup(void) {
  // Nothing to do?
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(void) {
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a' && cmd0 <= 'z')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case '?':
            Serial.println("(no commands)");
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


// ======================================================
// Main setup() and loop()
// ======================================================

bool serial_available = false;
#define MAXWAIT_SERIAL 200  // 200 = 2 seconds.

void open_serial(int baudrate = 9600) {
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
  //delay(500);
}

void setup() {
  // Configure Pico RP2040 I2C
  Wire.setSDA(sda_pin);
  Wire.setSCL(scl_pin);

  // initialize serial communication at 9600 bits per second.
  open_serial();
  Serial.print("pico_pps_logger ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  delay(1000);
  Serial.println("Ready to go...");

  // Light the on-board LED.
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  cmd_setup();
  setup_interrupts();
}

uint32_t last_seen_pps_time = 0;

void loop() {
  digitalWrite(ledPin, digitalRead(gps_pps_pin));

  cmd_update();
  
  if(last_seen_pps_time != last_pps_time[0]) {
    last_seen_pps_time = last_pps_time[0];
    // Emit log - micros skew for each clock to 0 edge.
    for (int pin = 0; pin < MAX_PPSS; ++pin) {
      if (pin > 0) {
        Serial.print(", ");
      }
      Serial.print(last_seen_pps_time - snapshot_pps_time[pin]);
    }
    Serial.println();
  }
}
