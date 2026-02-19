#pragma once
#include "globals.h"
#include "math_utils.h"
#include <math.h>

// Oppdaterer z_est/v_est og P.. basert på:
//  - dt
//  - az_filt (lineær vertikal aksel)
//  - altitude_m (baro)
//  - bmp_ok
//  - state (for onPad logic)
static inline void kalmanVzUpdate(float dt,
                                  float az_filt,
                                  float altitude_m,
                                  bool bmp_ok,
                                  float gx_dps,
                                  float gy_dps,
                                  float gz_dps,
                                  float a_norm,
                                  float& altitude_est_m,
                                  float& vz_est_mps)
{
  // -------- System model --------
  const float F00 = 1.0f, F01 = dt;
  const float F10 = 0.0f, F11 = 1.0f;
  const float Bz0 = 0.5f * dt * dt;
  const float Bz1 = dt;

  const float q_aa = sigma_acc * sigma_acc;
  const float q_vv = sigma_model_v * sigma_model_v;

  const float Q00 = Bz0*Bz0 * q_aa;
  const float Q01 = Bz0*Bz1 * q_aa;
  const float Q10 = Q01;
  const float Q11 = Bz1*Bz1 * q_aa + q_vv;

  // -------- Predict --------
  const float z_pred = F00*z_est + F01*v_est + Bz0*az_filt;
  const float v_pred = F10*z_est + F11*v_est + Bz1*az_filt;

  // P pred
  const float P00p  = F00*P00 + F01*P10;
  const float P01p  = F00*P01 + F01*P11;
  const float P10p  = F10*P00 + F11*P10;
  const float P11p  = F10*P01 + F11*P11;

  const float P00pp = P00p*F00 + P01p*F01 + Q00;
  const float P01pp = P00p*F10 + P01p*F11 + Q01;
  const float P10pp = P10p*F00 + P11p*F01 + Q10;
  const float P11pp = P10p*F10 + P11p*F11 + Q11;

  // -------- Update w/ baro --------
  if (bmp_ok) {
    const float y = altitude_m - z_pred;
    const float S = P00pp + R_baro;
    const float K0 = P00pp / S;
    const float K1 = P10pp / S;

    z_est = z_pred + K0 * y;
    v_est = v_pred + K1 * y;

    const float P00n = (1 - K0)*P00pp;
    const float P01n = (1 - K0)*P01pp;
    const float P10n = -K1*P00pp + P10pp;
    const float P11n = -K1*P01pp + P11pp;

    P00 = P00n; P01 = P01n; P10 = P10n; P11 = P11n;
  } else {
    z_est = z_pred;
    v_est = v_pred;
    P00 = P00pp; P01 = P01pp; P10 = P10pp; P11 = P11pp;
  }

  // -------- OnPad-lock --------
  const bool gyro_quiet  = (fabsf(gx_dps) < 1.0f && fabsf(gy_dps) < 1.0f && fabsf(gz_dps) < 1.0f);
  const bool accel_quiet = (fabsf(a_norm - g0) < 0.08f*g0);
  const bool baro_quiet  = bmp_ok ? (fabsf(altitude_m - z_est) < 0.15f) : true;

  const bool onPad = (state == SYSTEM_CHECK || state == OPERATION_READY);
  const bool stationary = gyro_quiet && accel_quiet && baro_quiet;

  if (onPad && stationary) {
    z_est = 0.0f;
    v_est = 0.0f;
    P00 = 0.5f; P01 = 0.0f; P10 = 0.0f; P11 = 0.5f;
  } else if (onPad) {
    z_est *= 0.98f;
    v_est *= 0.98f;
  }

  // output clamp / deadzone
  altitude_est_m = z_est;
  vz_est_mps     = v_est;

  if (fabsf(vz_est_mps) < 0.05f) vz_est_mps = 0.0f;
  if (fabsf(altitude_est_m) < 0.05f) altitude_est_m = 0.0f;

  const float v_sigma  = sqrtf(fmaxf(P11, 1e-6f));
  const float v_thresh = 2.5f * v_sigma;
  if (fabsf(v_est) < v_thresh) v_est = 0.0f;
}