#pragma once
#include "globals.h"
#include "math_utils.h"
#include <math.h>

// MAGNETOMETER-FUNKSJONER (robust mot void getEvent)
static inline bool computeMagYawDeg(float rollDeg, float pitchDeg, float& yawMagDeg){
  sensors_event_t magEv;
  lis3mdl.getEvent(&magEv);

  if (magEv.magnetic.x == 0.0f && magEv.magnetic.y == 0.0f && magEv.magnetic.z == 0.0f) {
    return false;
  }

  float mx = (magEv.magnetic.x - mag_xBias) * mag_xScale;
  float my = (magEv.magnetic.y - mag_yBias) * mag_yScale;
  float mz = (magEv.magnetic.z - mag_zBias) * mag_zScale;

  const float cr = cosf(deg2rad(rollDeg));
  const float sr = sinf(deg2rad(rollDeg));
  const float cp = cosf(deg2rad(pitchDeg));
  const float sp = sinf(deg2rad(pitchDeg));

  const float Xh = mx * cp + my * sr * sp + mz * cr * sp;
  const float Yh = my * cr - mz * sr;

  float hdg = rad2deg(atan2f(-Yh, Xh));
  hdg += decl_deg;
  yawMagDeg = wrap180(hdg);
  return true;
}

static inline float magTrust(const float yawMagDeg, const float lastYaw, const float dt){
  float dy = fabsf(wrap180(yawMagDeg - lastYaw));
  float speed_ok = (dy <= 30.0f * dt);
  return speed_ok ? 1.0f : 0.15f;
}
