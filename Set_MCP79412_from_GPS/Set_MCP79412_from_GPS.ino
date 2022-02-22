// Set_MCP79412_from_GPS

// Waits for a good lock from the GPS module
// then copies its UTC time to set the RTCC.
// Will set the RTCC time every second while it runs.

// GPS module is D3 (tx to GPS), D4 (rx from GPS)
// MCP is on I2C i.e. SCL on A5, SDA on A4

#include <MCP79412RTC.h>    // http://github.com/JChristensen/MCP79412RTC
#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/
#include <SoftwareSerial.h>
// TinyGPS and SoftwareSerial libraries are the work of Mikal Hart

SoftwareSerial SerialGPS = SoftwareSerial(4, 5);  // receive on pin 4
TinyGPS gps; 

time_t prevDisplay = 0; // when the digital clock was displayed

void setup() {
  Serial.begin(9600);
  while (!Serial) ; // Needed for Leonardo only
  SerialGPS.begin(9600);
  Serial.println("Setting MCP79412 from GPS ...");
  Serial.println("Waiting for GPS time ... ");
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

}

void loop() {
  while (SerialGPS.available()) {
    if (gps.encode(SerialGPS.read())) { // process gps messages
      // when TinyGPS reports new data...
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      if (age < 500) {
        // set the Time to the latest GPS reading
        setTime(Hour, Minute, Second, Day, Month, Year);
        //adjustTime(offset * SECS_PER_HOUR);
      }
      // Maybe set the MCP
      if (Year > 2020) {
          tmElements_t tm;
          tm.Second = Second;
          tm.Minute = Minute;
          tm.Hour = Hour;
          tm.Wday = 0;
          tm.Day = Day;
          tm.Month = Month;
          tm.Year = Year - 1970;  // RTC year is since Unix epoch.
          RTC.write(tm);
          Serial.println("Wrote UTC time to MCP79412");
      }
    }
  }
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if the time has changed
      prevDisplay = now();
      digitalClockDisplay();  
    }
  }
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
