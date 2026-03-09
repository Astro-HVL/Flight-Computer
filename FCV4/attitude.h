#pragma once
#include <math.h>

#include "globals.h"
#include "math_utils.h"
#include "mag_yaw.h"

// Oppdaterer attitude basert på ICM gyro + accel.
// - Returnerer gx/gy/gz i deg/s
// - Returnerer yaw_out (yaw + yaw_offset, wrappet til [-180, 180])
static inline void attitudeUpdate(float dt,
                                  const sensors_event_t& icm_accelEv,
                                  const sensors_event_t& icm_gyroEv,
                                  float& gx_dps, float& gy_dps, float& gz_dps,
                                  float& yaw_out)
{
  // Gyro (rad/s -> deg/s)
  gx_dps = (icm_gyroEv.gyro.x - gyro_offset_x) * 57.2957795f;
  gy_dps = (icm_gyroEv.gyro.y - gyro_offset_y) * 57.2957795f;
  gz_dps = (icm_gyroEv.gyro.z - gyro_offset_z) * 57.2957795f;

  // Roll/Pitch (komplementærfilter)
  const float rollAcc = rad2deg(atan2f(icm_accelEv.acceleration.y,
                                       icm_accelEv.acceleration.z));

  const float pitchAcc = rad2deg(atan2f(-icm_accelEv.acceleration.x,
                                        sqrtf(icm_accelEv.acceleration.y * icm_accelEv.acceleration.y +
                                              icm_accelEv.acceleration.z * icm_accelEv.acceleration.z)));

  roll  = alpha_rp * (roll  + gx_dps * dt) + (1.0f - alpha_rp) * rollAcc;
  pitch = alpha_rp * (pitch + gy_dps * dt) + (1.0f - alpha_rp) * pitchAcc;

  // Gyro-integrasjon (rask, men drifter over tid)
  const float yaw_gyro = wrap180(yaw + gz_dps * dt);

  float yawMagDeg = 0.0f;
  const bool haveMag = computeMagYawDeg(roll, pitch, yawMagDeg); // Magnetometer-yaw: gyldig verdi?

  // Bruk mag-korreksjon kun i kvasi
  // - akselerometer nær 1g (lite lineær akselerasjon)
  // - moderat rotasjonshastighet
  const float a_norm = sqrtf(icm_accelEv.acceleration.x * icm_accelEv.acceleration.x +
                             icm_accelEv.acceleration.y * icm_accelEv.acceleration.y +
                             icm_accelEv.acceleration.z * icm_accelEv.acceleration.z);
  const float acc_err = fabsf(a_norm - g0);
  const float max_rate_dps = fmaxf(fabsf(gx_dps), fmaxf(fabsf(gy_dps), fabsf(gz_dps)));
  const bool quasi_static = (acc_err < 1.2f) && (max_rate_dps < 35.0f);

  if (haveMag && quasi_static) {
    const float trust = magTrust(yawMagDeg, yaw_gyro, dt); // Returnerer 0.0, 0.35 eller 1.0
    const float beta  = 0.02f * trust;  // Korreksjon
    yaw = wrap180((1.0f - beta) * yaw_gyro + beta * yawMagDeg);
  } else {
    yaw = yaw_gyro;
  }

  // Output yaw med offset slik at den starter nær 0°
  yaw_out = wrap180(yaw + yaw_offset);
}




  // Roll & pitch fra akselerometer
  const float rollAcc  = rad2deg(atan2f(ay, az));
  const float pitchAcc = rad2deg(atan2f(-ax, sqrtf(ay*ay + az*az)));

  // Komplementærfilter: gyro (rask) + acc (absolutt)
  roll  = alpha_rp * (roll  + gx_dps * dt) + (1.0f - alpha_rp) * rollAcc;
  pitch = alpha_rp * (pitch + gy_dps * dt) + (1.0f - alpha_rp) * pitchAcc;



  // Feilen må wrappes fordi vinkel er periodisk (f.eks. 179° vs -179°)
  const float yaw_gyro = wrap180(yaw + gz_dps * dt); // Gyro-integrasjon (rask, men drifter over tid)

  float yawMagDeg = 0.0f;
  // Magnetometer-yaw: gyldig verdi?
  const bool haveMag = computeMagYawDeg(roll, pitch, yawMagDeg);

  // Bruk mag kun når systemet er kvasi-stasisk
  const float acc_err = fabsf(a_norm - g0);
  const float max_rate_dps = fmaxf(fabsf(gx_dps), fmaxf(fabsf(gy_dps), fabsf(gz_dps)));
  const bool quasi_static = (acc_err < 1.2f) && (max_rate_dps < 35.0f);

  if (haveMag && quasi_static) {
    const float trust = magTrust(yawMagDeg, yaw_gyro, dt);  // 0.0, 0.35 eller 1.0
    const float beta  = 0.02f * trust; // Korreksjons-gain (K)
    const float yaw_err = wrap180(yawMagDeg - yaw_gyro);
    yaw = wrap180(yaw_gyro + beta * yaw_err);
  } else {
    yaw = yaw_gyro;
  }
