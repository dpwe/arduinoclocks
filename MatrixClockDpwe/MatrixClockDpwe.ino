/*
 * This example is pulled from the DS1307RTC library with slight
 * modifications to demonstrate how to use the alternate I2C pins
 * accessible on the SmartMatrix Shield.
 *
 * Original DS1307RTC Library:
 * https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
 *
 * Requires a DS1307, DS1337 or DS3231 RTC chip connected to
 * Teensy pins 19 (SCL) and 18 (SDA)
 * 
 * Use 3.3V power for RTC module and I2S Pullup Resistors as some Teensy modules aren't 5V tolerant
 *
 * If using an old (V1-V3) SmartMatrix Shield or bare Teensy 3 with address pins connected to your matrix
 * you'll need to connect Teensy pins 16 (SCL) and 17 (SDA), and uncomment the alternate pin code below
 *
 * Not tested, and most likely doesn't work on ESP32
 * 
 * This SmartMatrix example uses just the indexed color layer
 */

#include <Wire.h>
#include <TimeLib.h>
//#include <DS1307RTC.h>

// uncomment one line to select your MatrixHardware configuration - configuration header needs to be included before <SmartMatrix.h>
#include <MatrixHardware_Teensy3_ShieldV4.h>        // SmartLED Shield for Teensy 3 (V4)
//#include <MatrixHardware_Teensy4_ShieldV5.h>        // SmartLED Shield for Teensy 4 (V5)
//#include <MatrixHardware_Teensy3_ShieldV1toV3.h>    // SmartMatrix Shield for Teensy 3 V1-V3
//#include <MatrixHardware_Teensy4_ShieldV4Adapter.h> // Teensy 4 Adapter attached to SmartLED Shield for Teensy 3 (V4)
//#include <MatrixHardware_ESP32_V0.h>                // This file contains multiple ESP32 hardware configurations, edit the file to define GPIOPINOUT (or add #define GPIOPINOUT with a hardcoded number before this #include)
//#include "MatrixHardware_Custom.h"                  // Copy an existing MatrixHardware file to your Sketch directory, rename, customize, and you can include it like this
#include <SmartMatrix.h>
#include <FastLED.h>

#define COLOR_DEPTH 24                  // Choose the color depth used for storing pixels in the layers: 24 or 48 (24 is good for most sketches - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24)
const uint16_t kMatrixWidth = 64;       // Set to the width of your display, must be a multiple of 8
const uint16_t kMatrixHeight = 64;      // Set to the height of your display
const uint8_t kRefreshDepth = 24;       // Tradeoff of color quality vs refresh rate, max brightness, and RAM usage.  36 is typically good, drop down to 24 if you need to.  On Teensy, multiples of 3, up to 48: 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48.  On ESP32: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save RAM, more to keep from dropping frames and automatically lowering refresh rate.  (This isn't used on ESP32, leave as default)
const uint8_t kPanelType = SM_PANELTYPE_HUB75_64ROW_MOD32SCAN;   // Choose the configuration that matches your panels.  See more details in MatrixCommonHub75.h and the docs: https://github.com/pixelmatix/SmartMatrix/wiki
const uint32_t kMatrixOptions = (SM_HUB75_OPTIONS_NONE);        // see docs for options: https://github.com/pixelmatix/SmartMatrix/wiki
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);

const int defaultBrightness = 64;
//const int defaultBrightness = (2*255)/100;     // dim: 2% brightness

//const SM_RGB clockColor = {0x10, 0x40, 0xc0};
const SM_RGB clockColor = {0xc0, 0xc0, 0xc0};
//const SM_RGB clockColor2 = {0x50, 0x60, 0x80};

const SM_RGB bgColor = {0x0, 0x0, 0x0};

const SM_RGB dotColor = {0xff, 0x0, 0x0};


short int rtc_tick = 0;

// from https://forum.pjrc.com/threads/32254-Teensy-3-x-RTC-Seconds-Interrupt
void rtc_seconds_isr(void)
{
    static int state=0;

    state = state ^ 1;
    pinMode(13, OUTPUT);
    digitalWrite(13, state);

    rtc_tick = 1;
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

// Auto DST

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
// US Pacific Time Zone (Las Vegas, Los Angeles)
//TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};
//TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};
Timezone myTZ(myDST, mySTD);


// Excised from BurstClockSerial, so we can support the same clock set syntax.

byte atoi2(char *s) {
  // Convert two ascii digits to a uint8.
  return (s[1] - '0') + 10 * (s[0] - '0');
}

tmElements_t parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  tmElements_t tm;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  tm.Year = y2kYearToTm(atoi2(time_string + 2));
  tm.Month = atoi2(time_string + 4);
  tm.Day = atoi2(time_string + 6);
  tm.Hour = atoi2(time_string + 8);
  tm.Minute = atoi2(time_string + 10);
  tm.Second = atoi2(time_string + 12);
  return tm;
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
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            if (strlen(cmd_buffer) != 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              Teensy3Clock.set(makeTime(parse_time_string(cmd_buffer + 1)));
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


void setup() {

  // initialize serial communication at 9600 bits per second.
  Serial.begin(9600);
  Serial.println("MatrixClock starting.");

  RTC_IER |= 0x10;  // set the TSIE bit (Time Seconds Interrupt Enable)
  NVIC_ENABLE_IRQ(IRQ_RTC_SECOND);
  
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  // setup matrix
  matrix.addLayer(&backgroundLayer); 
  matrix.addLayer(&indexedLayer); 
  matrix.begin();

  // Setup background.
  backgroundLayer.setBrightness(255);
  backgroundLayer.enableColorCorrection(true);

  //indexedLayer.setIndexedColor(1, bgColor);
  backgroundLayer.fillScreen(bgColor);

  for (int i = 0; i < 8; ++i) {
    backgroundLayer.fillRectangle(8 * i, 0, 8 * (i + 1) - 1, 10, {160 * (i & 1), 88 * (i & 2), 63 * (i & 4)});
  }
  for (int i = 0; i < 64; ++i) {
    backgroundLayer.fillRectangle(i, 10, i + 1, 20, {(160 * i) / 64, (176 * i) / 64, (255 * i) / 64});
  }

  backgroundLayer.swapBuffers();

  // display a simple message - will stay on the screen if calls to the RTC library fail later
  indexedLayer.fillScreen(0);
  indexedLayer.setFont(gohufont11b);
  indexedLayer.setIndexedColor(1, clockColor);
  indexedLayer.drawString(0, kMatrixHeight / 2 - 6, 1, "CLOCK");
  
  indexedLayer.swapBuffers(false);

  matrix.setBrightness(defaultBrightness);
}


void draw_fat_string(int x, int y, char *string) {
  // Convolve string with a 2x2 rectangle to make it fat.
  int color_index = 1;
  indexedLayer.drawString(x, y, color_index, string);
  indexedLayer.drawString(x+1, y, color_index, string);
  indexedLayer.drawString(x, y + 1, color_index, string);
  indexedLayer.drawString(x+1, y + 1, color_index, string);
}

int last_second = -1;
int hour_ = 0;
int minute_ = 0;
int second_ = 0;
int loop_count = 0;

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(now());
}

// int type that is transparently incremented every millisecond.
elapsedMillis millisSinceTick;

void loop() {

  cmd_update();

  loop_count = (loop_count + 1) % 256;

  // Time including DST.
  tmElements_t tm;
  breakTime(now_local(), tm);

  second_ = tm.Second;
  if (second_ != last_second) {
    // first loop since clock ticked.
    millisSinceTick = 0;

    last_second = second_;

    // Update the time variables once per tick.
    hour_ = tm.Hour;
    minute_ = tm.Minute;

    // Update the time digits on the indexed layer.
    char timeBuffer[12];

    // Clear screen before writing new text.
    indexedLayer.fillScreen(0);

    indexedLayer.setFont(font8x13);
    int char_width_pixels = 8; // for 8x13.
    int time_chars = strlen("00:00");
    int date_chars = strlen("12/31");

    // Time in top half.
    int x = kMatrixWidth/2 - (char_width_pixels * time_chars) / 2;
    int y = kMatrixHeight / 2 - 17;
    sprintf(timeBuffer, "%02d:%02d", hour_, minute_);
    // indexedLayer.setIndexedColor(1, clockColor);
    draw_fat_string(x, y, timeBuffer);

    // Date in bottom half.
    indexedLayer.setFont(font6x10);
    char_width_pixels = 6; // for 6x10.
    x = kMatrixWidth/2 - (char_width_pixels * date_chars) / 2;
    y = kMatrixHeight / 2 + 5;
    sprintf(timeBuffer, "%02d-%02d", tm.Month, tm.Day);
    // indexedLayer.setIndexedColor(1, clockColor2);
    //draw_fat_string(x, y, timeBuffer);
    indexedLayer.drawString(x, y, 1, timeBuffer);

    
    // Track # millis between each tick.
    //sprintf(timeBuffer, "%4d", cached_millisSinceTick);
    //indexedLayer.setFont(font5x7);
    //indexedLayer.drawString(44, 55, 1, timeBuffer);
  
    indexedLayer.swapBuffers();
  }

  backgroundLayer.fillScreen(bgColor);

  // Moving second hand.
  float angle = 2.0 * 3.1415926 * (float(second_) + float(millisSinceTick)/1000) / 60.0;
  float r = 28.0;
  float x_center = 32.0, y_center = 32.0;
  float x_point = x_center + r * sin(angle);
  float y_point = y_center - r * cos(angle);
  //backgroundLayer.fillCircle((int16_t)x_point, (int16_t)y_point, /* r= */ 3, dotColor);
  int halfwidth = 2;
  for (int x = (int16_t)x_point - halfwidth; x <= (int16_t)x_point + halfwidth; ++x) {
    for (int y = (int16_t)y_point - halfwidth; y <= (int16_t)y_point + halfwidth; ++y) {
      float x_rel = float(x) - x_point;
      float y_rel = float(y) - y_point;
      // Pixel intensity declines as distance-from-center squared out to sqrt(5) pixels.
      int value = int(255.0 * max(0, (1.0 - ((x_rel * x_rel + y_rel * y_rel) / 5.0))));
      SM_RGB color = CRGB(CHSV(140, 128, value));
      backgroundLayer.drawPixel(x, y, color);
    }
  }

  // 5-sec dots.
  for (int tick_hour = 0; tick_hour < 12; ++tick_hour) {
    angle = 2.0 * 3.1415926 * float(tick_hour) / 12.0;
    int x = (int16_t)round(x_center + r * sin(angle));
    int y = (int16_t)round(y_center - r * cos(angle));
    SM_RGB color = CRGB(CHSV(50, 128, 255));
    backgroundLayer.drawPixel(x, y, color);
  }

  // Rainbow seconds-bar.
  int first_pixel, last_pixel;
  if ((minute_ & 1) == 0) {
    first_pixel = 0;
    last_pixel = second_;
  } else {
    first_pixel = second_;
    last_pixel = 60;
  }
  // Precalculate intensity by row.
  //    int h = 5;
  //    int value = 25 * (10 - (abs(row - 2) * abs(row - 2)));
  int row_values[] = {150, 225, 250, 225, 150};
  int num_rows = sizeof(row_values) / sizeof(int);
  for (int pixel = first_pixel; pixel < last_pixel; ++pixel) {
    // Skip one pixel every 15 sec, and another at 30 sec.
    int x = pixel + 0 + (pixel / 15) + (pixel /30);
    // indexedLayer.drawPixel(pixel + 2, kMatrixHeight / 2 + 3, 1 + (pixel & 1));
    for (int row = 0; row < num_rows; ++row) {
      int y = kMatrixHeight / 2 - (num_rows / 2) + row;
      int hue = ((pixel + row) * 4 + loop_count) % 256;
      int saturation = 255;
      int value = row_values[row];
      SM_RGB color = CRGB(CHSV(hue, saturation, value));
      backgroundLayer.drawPixel(x, y, color);
    }
  }

  backgroundLayer.swapBuffers();

  // Start next loop at what we expect to be 10ms after the next tick.
  //delay(1010 - millisSinceTick);
  delay(10);
  
}
