#pragma once
#include <math.h>

static inline float rad2deg(float r){ return r * 57.2957795f; }
static inline float deg2rad(float d){ return d * 0.01745329252f; }

static inline float wrap180(float a){
  while (a > 180.f)  a -= 360.f;
  while (a < -180.f) a += 360.f;
  return a;
}

static inline float clampf(float x, float lo, float hi){
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Tilt-kompensasjon: rotasjonsmatrise body->world (ZYX: yaw->pitch->roll)
static inline void rotMatrixZYX(float rollDeg, float pitchDeg, float yawDeg, float R[3][3]) {
  const float cr = cosf(deg2rad(rollDeg));
  const float sr = sinf(deg2rad(rollDeg));
  const float cp = cosf(deg2rad(pitchDeg));
  const float sp = sinf(deg2rad(pitchDeg));
  const float cy = cosf(deg2rad(yawDeg));
  const float sy = sinf(deg2rad(yawDeg));

  R[0][0] = cy * cp;
  R[0][1] = cy * sp * sr - sy * cr;
  R[0][2] = cy * sp * cr + sy * sr;

  R[1][0] = sy * cp;
  R[1][1] = sy * sp * sr + cy * cr;
  R[1][2] = sy * sp * cr - cy * sr;

  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
}

// out = R * v
static inline void matVec3(const float R[3][3], float vx, float vy, float vz,
                           float& ox, float& oy, float& oz) {
  ox = R[0][0] * vx + R[0][1] * vy + R[0][2] * vz;
  oy = R[1][0] * vx + R[1][1] * vy + R[1][2] * vz;
  oz = R[2][0] * vx + R[2][1] * vy + R[2][2] * vz;
}
