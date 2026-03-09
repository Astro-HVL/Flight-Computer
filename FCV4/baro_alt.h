#pragma once
#include <math.h>
#include "globals.h"

// Konverter trykk til høyde (standard baro-formel) med baseline p0
// Returnerer meter (0 ved baseline)
static inline float baroAltitudeFromPressure(float pPa, float p0Pa)
{
  if (p0Pa <= 10000.0f || pPa <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(pPa / p0Pa, 0.1903f));
}

// baroCompute: bruker målingene fra readSensors() (sensors.h)
// - bmp_ok: om målingene er gyldige nå
// - output: temperatureC, pressurePa, altitude_m
static inline void baroCompute(bool bmp_ok, float bmp_tempC, float bmp_pressPa, float &temperatureC, float &pressurePa, float &altitude_m)
{
  if (bmp_ok) {
    temperatureC = bmp_tempC;
    pressurePa   = bmp_pressPa;
    altitude_m   = baroAltitudeFromPressure(pressurePa, bmp_p0_Pa);
  } else {
    // Failsafe hvis baro ikke fungerer: sett til 0
    // Unngår NaN verdier
    // Bruke z_est?
    temperatureC = 0.0f;
    pressurePa   = 0.0f;
    altitude_m   = 0.0f;
  }
}
