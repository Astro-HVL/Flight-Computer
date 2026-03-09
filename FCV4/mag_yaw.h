#pragma once
#include "globals.h"
#include "math_utils.h"
#include <math.h>

// MAGNETOMETER-FUNKSJONER
static inline bool computeMagYawDeg(float rollDeg, float pitchDeg, float& yawMagDeg){
  sensors_event_t magEv;
  lis3mdl.getEvent(&magEv);

  // Hvis ingen målinger, returner false
  if (magEv.magnetic.x == 0.0f && magEv.magnetic.y == 0.0f && magEv.magnetic.z == 0.0f) {
    return false;
  }

  // Kalibreringsverdier
  float mx = 0.0f, my = 0.0f, mz = 0.0f;
  mcal_apply(MCAL, magEv.magnetic.x, magEv.magnetic.y, magEv.magnetic.z, mx, my, mz);

  const float cr = cosf(deg2rad(rollDeg));
  const float sr = sinf(deg2rad(rollDeg));
  const float cp = cosf(deg2rad(pitchDeg));
  const float sp = sinf(deg2rad(pitchDeg));

  // Tilt-kompensere magnetometer
  const float Xh = mx * cp + my * sr * sp + mz * cr * sp;
  const float Yh = my * cr - mz * sr;

  float hdg = rad2deg(atan2f(-Yh, Xh));
  hdg += decl_deg;
  yawMagDeg = wrap180(hdg);
  return true;
}

static inline float magTrust(const float yawMagDeg, const float lastYaw, const float dt){
  (void)dt;
  float dy = fabsf(wrap180(yawMagDeg - lastYaw));
  if (dy <= 8.0f)  return 1.0f;   // nesten enig med gyro, stol på mag
  if (dy <= 20.0f) return 0.35f;  // delvis enig -> svak korreksjon
  return 0.0f;                    // stor uenighet -> ignorer mag denne runden
}
