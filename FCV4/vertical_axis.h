#pragma once
#include <math.h>
#include "math_utils.h"   // deg2rad, rotMatrixZYX, matVec3
#include <Adafruit_Sensor.h>

// Beregner tilt-kompensert vertikal aksel (world up) og netto-acc.
// Input:
//  - icmAcc: accelerometer event (m/s^2) i body frame
//  - rollDeg, pitchDeg, yawDeg: attitude i grader (samme yaw du bruker for R)
//  - g0: 9.80665
// Output:
//  - az_filt: filtrert vertikal aksel (m/s^2), positiv opp
//  - a_norm: |a| (m/s^2)
//  - a_net_filt: lavpass(|a|-g) for launch detect (m/s^2)
static inline void verticalAxisUpdate(const sensors_event_t& icmAcc,
                                      float rollDeg, float pitchDeg, float yawDeg,
                                      float g0,
                                      float& az_filt,
                                      float& a_norm,
                                      float& a_net_filt)
{
  // 1) Norm av akselerometer
  const float ax = icmAcc.acceleration.x;
  const float ay = icmAcc.acceleration.y;
  const float az = icmAcc.acceleration.z;

  a_norm = sqrtf(ax*ax + ay*ay + az*az);

  // 2) Roter til world (ZYX)
  float Rbw[3][3];
  rotMatrixZYX(rollDeg, pitchDeg, yawDeg, Rbw);

  float ax_w, ay_w, az_w;
  matVec3(Rbw, ax, ay, az, ax_w, ay_w, az_w);

  // 3) Lineær vertikal aksel: trekk fra g i world-up
  const float az_lin = az_w - g0;

  // 4) Mild LPF + deadzone (az_filt har intern tilstand)
  static float az_state = 0.0f;
  az_state = 0.25f * az_lin + 0.75f * az_state;
  if (fabsf(az_state) < 0.05f) az_state = 0.0f;

  az_filt = az_state;

  // 5) Netto-acc for launch detect: |a|-g (lavpass)
  const float a_net = fabsf(a_norm - g0);

  static float a_net_state = 0.0f;
  a_net_state = 0.2f * a_net + 0.8f * a_net_state;

  a_net_filt = a_net_state;
}
