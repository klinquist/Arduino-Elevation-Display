## Elevation Display

This will display your current elevation.  It even dims the display after sunset.

Error codes:

* `9001` - Display I2C not responding
* No GPS fix (after ~30s; shows `----` first, then `-GPS`)
* `9003` - GPS time/date not valid yet (tracked internally)
* `9004` - GPS location not valid yet (tracked internally)

Brightness behavior:

* Uses civil twilight (sun ~6° below horizon) so it stays bright a bit after sunset and before sunrise.
* `nighttimeBrightness` defaults to `0` (typically 0..15, where 0 is dimmest).

Boot / time behavior:

* While waiting for a GPS fix, the display shows `----` for ~30s, then `-GPS` if no fix.
* Once GPS time is valid, the display shows local time for ~10s, then switches to altitude.

Timezone configuration:

* Edit `LOCAL_STD_OFFSET_HOURS` and `LOCAL_OBSERVES_US_DST` in `GPS-Alt.ino` to match your timezone.
* Common US standard offsets: Pacific `-8`, Mountain `-7` (Arizona is `-7` with DST off), Central `-6`, Eastern `-5`, Alaska `-9`, Hawaii `-10`.

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
