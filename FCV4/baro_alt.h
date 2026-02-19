#pragma once
#include <math.h>
#include "globals.h"

// Konverter trykk til høyde (standard baro-formel) med baseline p0.
// Returnerer meter AGL (0 ved baseline).
static inline float baroAltitudeFromPressure(float pPa, float p0Pa)
{
  if (p0Pa <= 10000.0f || pPa <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(pPa / p0Pa, 0.1903f));
}

// Ryddig baroCompute: bruker målingene du allerede tok i readSensors().
// - bmp_ok: om målingene er gyldige nå
// - bmp_tempC / bmp_pressPa: snapshot fra sensoren
// - output: temperatureC, pressurePa, altitude_m
static inline void baroCompute(bool bmp_ok,
                              float bmp_tempC, float bmp_pressPa,
                              float &temperatureC, float &pressurePa, float &altitude_m)
{
  if (bmp_ok) {
    temperatureC = bmp_tempC + bmp_tOff;
    pressurePa   = bmp_pressPa + bmp_pOff;
    altitude_m   = baroAltitudeFromPressure(pressurePa, bmp_p0_Pa);
  } else {
    // Failsafe hvis baro er nede: hold “siste gode” eller sett 0
    // For nå: sett 0 slik at resten av pipeline ikke får NAN.
    // (Du kan evt bruke z_est her hvis du vil)
    temperatureC = 0.0f;
    pressurePa   = 0.0f;
    altitude_m   = 0.0f;
  }
}