## Elevation Display

This will display your current elevation.  It even dims the display after sunset.

Error codes:

* `9001` - Display I2C not responding
* `9002` - No GPS fix (after ~30s)
* `9003` - GPS time/date not valid yet
* `9004` - GPS location not valid yet

Brightness behavior:

* Uses civil twilight (sun ~6° below horizon) so it stays bright a bit after sunset and before sunrise.
* `nighttimeBrightness` defaults to `0` (typically 0..15, where 0 is dimmest).

For elevations above 9,999 feet, the first character will be displayed in hex. So 11,200 ft will be B200. 

I haven't tested above 16,000 ft.


![Elevation Display](https://photos.smugmug.com/photos/i-k5pwxjc/0/M/i-k5pwxjc-M.jpg)

Parts required:

* [Arduino Metro Mini](https://www.adafruit.com/product/2590)
* [GPS](https://www.adafruit.com/product/4279)
* [7 Segment Display with I2C backpack](https://www.adafruit.com/product/1002)
* [12v to microUSB power supply](https://dongar.tech) - Dongar Tech makes many adapters that give you a plug-and-play USB port from your exisitng mirror wiring harness.


Some soldering & coding may be required!


* Connect GPS to +5v, GND, Digital pins 7 & 8
* Connect display to +5V, GND, SCL, & SCA (A4 & A5 on the Metro Mini)
* Upload the sketch included in this repository to the Arduino using the Arduino IDE & a USB cable!
