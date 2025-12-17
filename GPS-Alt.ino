// Elevation display by Kris Linquist
// Requires TimeLib.h / Adafruit_GPS.h / SoftwareSerial.h / HK16K33.h


// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746

//And the Adafruit 7 segment I2C LCD

#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include "HT16K33.h"
#include "SunCalc.h"

HT16K33 seg(0x70);

tmElements_t te;  //Time elements structure
time_t unixTime; // a time stamp


int daytimeBrightness = 15;
// Brightness range is typically 0..15 (0 = dimmest).
// If 0 is too dim on your hardware, bump this up to 1+.
int nighttimeBrightness = 0;
int currentBrightness = -1;

// Error codes are displayed as 9xxx to avoid confusion with altitude.
// Example: 9002 => "no GPS fix".
enum ErrorCode : uint8_t {
  ERR_NONE = 0,
  ERR_DISPLAY_I2C = 1,
  ERR_NO_GPS_FIX = 2,
  ERR_NO_GPS_TIME = 3,
  ERR_NO_GPS_LOCATION = 4,
  ERR_SUN_CALC = 5,
};
uint8_t currentError = ERR_NONE;
uint32_t bootMs = 0;
bool displayOk = true;
bool displayRecovered = false;


// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
//

static const uint8_t DISPLAY_ADDR = 0x70;
static const uint32_t DISPLAY_UPDATE_MS = 2000;
static const uint32_t GPS_FIX_GRACE_MS = 30000;
// 96° zenith ≈ civil twilight (sun 6° below horizon). This keeps the display
// bright a bit after sunset and before sunrise, and avoids rapid toggling.
static const float BRIGHTNESS_ZENITH_DEG = 96.0f;

static float nmeaToDecimalDegrees(float ddmm_mmmm) {
  double degrees = floor((double)ddmm_mmmm / 100.0);
  double minutes = (double)ddmm_mmmm - (degrees * 100.0);
  return (float)(degrees + (minutes / 60.0));
}

static bool getGpsLatLonDeg(float &latDeg, float &lonDeg) {
  if (GPS.latitude == 0.0f && GPS.longitude == 0.0f) return false;
  if (!(GPS.lat == 'N' || GPS.lat == 'S')) return false;
  if (!(GPS.lon == 'E' || GPS.lon == 'W')) return false;

  latDeg = nmeaToDecimalDegrees(GPS.latitude);
  lonDeg = nmeaToDecimalDegrees(GPS.longitude);

  if (GPS.lat == 'S') latDeg = -latDeg;
  if (GPS.lon == 'W') lonDeg = -lonDeg;

  if (!(latDeg >= -90.0f && latDeg <= 90.0f)) return false;
  if (!(lonDeg >= -180.0f && lonDeg <= 180.0f)) return false;
  return true;
}

static bool getGpsUnixTimeUtc(time_t &out) {
  if (GPS.year == 0 || GPS.month == 0 || GPS.day == 0) return false;
  if (GPS.month < 1 || GPS.month > 12) return false;
  if (GPS.day < 1 || GPS.day > 31) return false;
  if (GPS.hour > 23 || GPS.minute > 59 || GPS.seconds > 59) return false;

  te.Second = GPS.seconds;
  te.Minute = GPS.minute;
  te.Hour = GPS.hour;
  te.Day = GPS.day;
  te.Month = GPS.month;
  te.Year = CalendarYrToTm(2000 + GPS.year);

  out = makeTime(te);
  return out != 0;
}

static bool isDisplayPresent() {
  Wire.beginTransmission(DISPLAY_ADDR);
  return Wire.endTransmission() == 0;
}

static bool ensureDisplayReady() {
  static uint32_t lastPingMs = 0;
  static bool lastOk = true;
  uint32_t nowMs = millis();
  if (nowMs - lastPingMs < 1000) return lastOk;
  lastPingMs = nowMs;

  bool okNow = isDisplayPresent();
  if (okNow) {
    if (!lastOk) displayRecovered = true;
    lastOk = true;
    return true;
  }

  // Attempt a simple recovery in case the HT16K33 reset due to power dip.
  Wire.begin();
  Wire.setClock(100000);
  seg.begin();
  seg.displayOn();
  if (currentBrightness < 0) currentBrightness = nighttimeBrightness;
  seg.brightness(currentBrightness);

  okNow = isDisplayPresent();
  if (okNow && !lastOk) displayRecovered = true;
  lastOk = okNow;
  return okNow;
}

static void displayError(uint8_t err) {
  int code = 9000 + (int)err;
  seg.displayInt(code);
}

static void ensureDisplayOn() {
  seg.displayOn();
  if (currentBrightness < 0) currentBrightness = nighttimeBrightness;
  seg.brightness(currentBrightness);
}

void setup()
{
  bootMs = millis();

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  Wire.begin();
  seg.begin();
  Wire.setClock(100000);
  seg.displayOn();
  seg.displayClear();  

  // Show a non-blank startup state.
  // Fail-safe dim until we have valid GPS time+location to decide day/night.
  currentBrightness = nighttimeBrightness;
  seg.brightness(currentBrightness);
  seg.displayInt(0);
  delay(500);


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  displayOk = ensureDisplayReady();
  if (displayOk) ensureDisplayOn();

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    // We can fail to parse a sentence; don't return early so the display can
    // keep updating with errors / last known state.
    (void)GPS.parse(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  }

  // approximately every 2 seconds or so, update the display
  if (millis() - timer > DISPLAY_UPDATE_MS) {
    timer = millis(); // reset the timer

    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    // Brightness control (requires valid time and location).
    time_t ts = 0;
    float latDeg = 0.0f, lonDeg = 0.0f;
    bool hasTime = getGpsUnixTimeUtc(ts);
    bool hasLoc = getGpsLatLonDeg(latDeg, lonDeg);
    if (hasTime && hasLoc) {
      unixTime = ts;
      SunriseSunsetUtc ss = calcSunriseSunsetUtc(unixTime, latDeg, lonDeg, BRIGHTNESS_ZENITH_DEG);
      if (!ss.valid) {
        currentError = ERR_SUN_CALC;
      } else {
        int nowMin = (GPS.hour * 60) + GPS.minute;
        bool isDay = false;
        if (ss.polarDay) {
          isDay = true;
        } else if (ss.polarNight) {
          isDay = false;
        } else if (ss.sunriseMin == ss.sunsetMin) {
          isDay = true;
        } else if (ss.sunriseMin < ss.sunsetMin) {
          isDay = (nowMin >= (int)ss.sunriseMin) && (nowMin < (int)ss.sunsetMin);
        } else {
          // Day wraps midnight (common in UT at some longitudes / seasons).
          isDay = (nowMin >= (int)ss.sunriseMin) || (nowMin < (int)ss.sunsetMin);
        }

        int desired = isDay ? daytimeBrightness : nighttimeBrightness;
        Serial.print("UTC ");
        if (GPS.hour < 10) Serial.print('0');
        Serial.print((int)GPS.hour);
        Serial.print(':');
        if (GPS.minute < 10) Serial.print('0');
        Serial.print((int)GPS.minute);
        Serial.print(" sunrise ");
        Serial.print((int)(ss.sunriseMin / 60));
        Serial.print(':');
        if ((ss.sunriseMin % 60) < 10) Serial.print('0');
        Serial.print((int)(ss.sunriseMin % 60));
        Serial.print(" sunset ");
        Serial.print((int)(ss.sunsetMin / 60));
        Serial.print(':');
        if ((ss.sunsetMin % 60) < 10) Serial.print('0');
        Serial.print((int)(ss.sunsetMin % 60));
        Serial.print(" -> ");
        Serial.print(isDay ? "day" : "night");
        Serial.print(" brightness ");
        Serial.println(desired);

        if (desired != currentBrightness) {
          Serial.print("Setting brightness to ");
          Serial.println(isDay ? "day" : "night");
          if (displayOk) seg.brightness(desired);
          currentBrightness = desired;
        }
        currentError = ERR_NONE;
      }
    } else {
      if (!hasTime) currentError = ERR_NO_GPS_TIME;
      else if (!hasLoc) currentError = ERR_NO_GPS_LOCATION;
    }

    // Altitude / error display
    if (displayOk && displayRecovered) {
      displayRecovered = false;
      displayError(ERR_DISPLAY_I2C);
      return;
    }

    if (GPS.fix) {
      // Meters to feet...
      double doubFt = GPS.altitude * 3.28084;
      // Round to nearest 10
      int mod = (int)doubFt % 10;
      if (mod < 5) {
        doubFt = doubFt - mod;
      } else {
        doubFt = doubFt + (10 - mod);
      }
      if (displayOk) seg.displayInt((int)doubFt);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    } else {
      // Give the GPS a bit of time after boot before showing an error.
      if (displayOk) {
        if (millis() - bootMs <= GPS_FIX_GRACE_MS) {
          seg.displayInt(0);
        } else if (currentError == ERR_NO_GPS_TIME) {
          displayError(ERR_NO_GPS_TIME);
        } else if (currentError == ERR_NO_GPS_LOCATION) {
          displayError(ERR_NO_GPS_LOCATION);
        } else {
          displayError(ERR_NO_GPS_FIX);
        }
      }
    }
  }
}
