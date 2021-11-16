// Elevation display by Kris Linquist
// Requires SunRise.h / TimeLib.h / Adafruit_GPS.h / SoftwareSerial.h / HK16K33.h


// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746

//And the Adafruit 7 segment I2C LCD

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <SunRise.h>
#include "HT16K33.h"

HT16K33 seg(0x70);

SunRise sr;

tmElements_t te;  //Time elements structure
time_t unixTime; // a time stamp


int daytimeBrightness = 15;
int nighttimeBrightness = 0;
int currentBrightness;


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


void setup()
{

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

  seg.begin();
  Wire.setClock(100000);
  seg.displayOn();
  seg.displayClear();  

  //Just show 0.000 until we get a GPS fix
  seg.displayFloat(0,4);
  Serial.print("Startup: setting brightness to: ");
  Serial.println(nighttimeBrightness);
  seg.brightness(nighttimeBrightness);
  currentBrightness = nighttimeBrightness;
  delay(5000);
 

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

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, update the display
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    te.Second = GPS.seconds;
    te.Hour = GPS.hour;
    te.Minute = GPS.minute;
    te.Day = GPS.day;
    te.Month = GPS.month;
    int actualYear = 2000 + GPS.year;
    te.Year = actualYear - 1970;
    unixTime = makeTime(te);

    float lata = GPS.latitude/100;
    float longi = 0 - (GPS.longitude/100);
    int tz = -8;

    sr.calculate(lata, longi, unixTime);
    Serial.print("Unixtime: ");
    Serial.println(unixTime);

    if (sr.isVisible){
      Serial.println("It's daytime!");
      if (currentBrightness != daytimeBrightness) {
        Serial.println("Setting brightness to day");
        seg.brightness(daytimeBrightness);
        currentBrightness = daytimeBrightness;
      }
    } else {
      Serial.println("It's nighttime!");
      if (currentBrightness != nighttimeBrightness) {
        Serial.println("Setting brightness to night");
        seg.brightness(nighttimeBrightness);
        currentBrightness = nighttimeBrightness;
      }   
    }


    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
       //Meters to feet...
       double doubFt = GPS.altitude * 3.28084;
       //Round to nearest 10
       int mod = (int) doubFt % 10;
       if (mod < 5) {
        doubFt = doubFt - mod;
       } else {
         doubFt = doubFt + (10 - mod);
       }
       seg.displayInt((int) doubFt); 
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    } 
  }
}
