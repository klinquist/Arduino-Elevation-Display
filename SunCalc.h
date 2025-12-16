#pragma once

#include <Arduino.h>
#include <TimeLib.h>
#include <math.h>

// Civil twilight ≈ zenith 96° (sun 6° below horizon)

struct SunriseSunsetUtc {
  bool valid = false;
  bool polarDay = false;
  bool polarNight = false;
  uint16_t sunriseMin = 0; // minutes after 00:00 UTC
  uint16_t sunsetMin = 0;  // minutes after 00:00 UTC
};

static double normalizeDeg(double deg) {
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

static double degToRad(double deg) { return deg * 0.017453292519943295; }
static double radToDeg(double rad) { return rad * 57.29577951308232; }

static bool isLeapYear(int year) {
  if (year % 400 == 0) return true;
  if (year % 100 == 0) return false;
  return (year % 4) == 0;
}

static int dayOfYear(int year, int month, int day) {
  static const int daysBeforeMonth[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int doy = daysBeforeMonth[month - 1] + day;
  if (month > 2 && isLeapYear(year)) doy += 1;
  return doy;
}

static double calcSunEventUtcHours(
    bool sunrise,
    int year,
    int month,
    int day,
    double latDeg,
    double lonDeg,
    double zenithDeg,
    bool &polarDay,
    bool &polarNight) {
  polarDay = false;
  polarNight = false;

  int N = dayOfYear(year, month, day);
  double lngHour = lonDeg / 15.0;

  double t = sunrise ? (double)N + ((6.0 - lngHour) / 24.0) : (double)N + ((18.0 - lngHour) / 24.0);
  double M = (0.9856 * t) - 3.289;
  double L = M + (1.916 * sin(degToRad(M))) + (0.020 * sin(2.0 * degToRad(M))) + 282.634;
  L = normalizeDeg(L);

  double RA = radToDeg(atan(0.91764 * tan(degToRad(L))));
  RA = normalizeDeg(RA);

  double Lquadrant = floor(L / 90.0) * 90.0;
  double RAquadrant = floor(RA / 90.0) * 90.0;
  RA = RA + (Lquadrant - RAquadrant);
  RA /= 15.0;

  double sinDec = 0.39782 * sin(degToRad(L));
  double cosDec = cos(asin(sinDec));
  double cosH = (cos(degToRad(zenithDeg)) - (sinDec * sin(degToRad(latDeg)))) / (cosDec * cos(degToRad(latDeg)));

  if (cosH > 1.0) {
    polarNight = true;
    return 0.0;
  }
  if (cosH < -1.0) {
    polarDay = true;
    return 0.0;
  }

  double H = sunrise ? (360.0 - radToDeg(acos(cosH))) : radToDeg(acos(cosH));
  H /= 15.0;

  double T = H + RA - (0.06571 * t) - 6.622;
  double UT = T - lngHour;
  while (UT < 0.0) UT += 24.0;
  while (UT >= 24.0) UT -= 24.0;
  return UT;
}

static SunriseSunsetUtc calcSunriseSunsetUtc(time_t unixTimeUtc, float latDeg, float lonDeg, float zenithDeg) {
  SunriseSunsetUtc out;
  tmElements_t te;
  breakTime(unixTimeUtc, te);
  int year = 1970 + te.Year;

  bool polarDayRise = false, polarNightRise = false;
  bool polarDaySet = false, polarNightSet = false;
  double riseHr = calcSunEventUtcHours(true, year, te.Month, te.Day, latDeg, lonDeg, zenithDeg, polarDayRise, polarNightRise);
  double setHr = calcSunEventUtcHours(false, year, te.Month, te.Day, latDeg, lonDeg, zenithDeg, polarDaySet, polarNightSet);

  if (polarDayRise || polarDaySet) {
    out.valid = true;
    out.polarDay = true;
    return out;
  }
  if (polarNightRise || polarNightSet) {
    out.valid = true;
    out.polarNight = true;
    return out;
  }

  long riseMin = (long)(riseHr * 60.0 + 0.5);
  long setMin = (long)(setHr * 60.0 + 0.5);
  riseMin %= 1440;
  setMin %= 1440;
  if (riseMin < 0) riseMin += 1440;
  if (setMin < 0) setMin += 1440;
  out.sunriseMin = (uint16_t)riseMin;
  out.sunsetMin = (uint16_t)setMin;
  out.valid = true;
  return out;
}

