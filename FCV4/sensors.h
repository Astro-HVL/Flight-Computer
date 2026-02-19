#pragma once
#include <Arduino.h>
#include "globals.h"

// Én "snapshot" av alle sensorer for én loop-iterasjon
struct SensorFrame {
  // Raw sensor events
  sensors_event_t adxl;
  sensors_event_t icm_acc;
  sensors_event_t icm_gyro;
  sensors_event_t icm_temp;

  // Baro snapshot
  bool  bmp_ok = false;
  float bmp_tempC = NAN;
  float bmp_pressPa = NAN;

  // Timing
  float dt = 0.001f;
  unsigned long nowMicros = 0;
};

static inline SensorFrame readSensors()
{
  SensorFrame s;

  // --- dt (mikrosekunder-basert) ---
  s.nowMicros = micros();
  s.dt = (s.nowMicros - lastMicros) / 1e6f;
  if (s.dt <= 0.0f) s.dt = 0.001f;
  lastMicros = s.nowMicros;

  // --- IMU & ADXL ---
  adxl.getEvent(&s.adxl);
  icm.getEvent(&s.icm_acc, &s.icm_gyro, &s.icm_temp);

  // --- BARO (les kun én gang per loop) ---
  s.bmp_ok = (bmp_present && bmp.performReading());
  if (s.bmp_ok) {
    s.bmp_tempC   = bmp.temperature;
    s.bmp_pressPa = bmp.pressure;
  }

  return s;
}