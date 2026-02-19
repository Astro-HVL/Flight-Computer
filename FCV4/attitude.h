#pragma once
#include <math.h>

#include "globals.h"
#include "math_utils.h"
#include "mag_yaw.h"

// Oppdaterer attitude basert på ICM gyro+accel.
// - Returnerer gx/gy/gz i deg/s (nyttig for Kalman onPad-lock)
// - Returnerer yaw_out (yaw + yaw_offset, wrappet til [-180, 180])
static inline void attitudeUpdate(float dt,
                                  const sensors_event_t& icm_accelEv,
                                  const sensors_event_t& icm_gyroEv,
                                  float& gx_dps, float& gy_dps, float& gz_dps,
                                  float& yaw_out)
{
  // ---------- Gyro (rad/s -> deg/s) ----------
  gx_dps = (icm_gyroEv.gyro.x - gyro_offset_x) * 57.2957795f;
  gy_dps = (icm_gyroEv.gyro.y - gyro_offset_y) * 57.2957795f;
  gz_dps = (icm_gyroEv.gyro.z - gyro_offset_z) * 57.2957795f;

  // ---------- Roll/Pitch (komplementærfilter) ----------
  const float rollAcc = rad2deg(atan2f(icm_accelEv.acceleration.y,
                                       icm_accelEv.acceleration.z));

  const float pitchAcc = rad2deg(atan2f(-icm_accelEv.acceleration.x,
                                        sqrtf(icm_accelEv.acceleration.y * icm_accelEv.acceleration.y +
                                              icm_accelEv.acceleration.z * icm_accelEv.acceleration.z)));

  roll  = alpha_rp * (roll  + gx_dps * dt) + (1.0f - alpha_rp) * rollAcc;
  pitch = alpha_rp * (pitch + gy_dps * dt) + (1.0f - alpha_rp) * pitchAcc;

  // ---------- Yaw: gyro-integrasjon + magnetometer-korreksjon ----------
  const float yaw_gyro = wrap180(yaw + gz_dps * dt);

  float yawMagDeg = 0.0f;
  const bool haveMag = computeMagYawDeg(roll, pitch, yawMagDeg);

  if (haveMag) {
    const float trust = magTrust(yawMagDeg, yaw_gyro, dt);
    const float beta  = 0.02f * trust;  // 0..0.02
    yaw = wrap180((1.0f - beta) * yaw_gyro + beta * yawMagDeg);
  } else {
    yaw = yaw_gyro;
  }

  // Output yaw med offset slik at den starter nær 0°
  yaw_out = wrap180(yaw + yaw_offset);
}