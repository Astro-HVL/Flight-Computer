// ========================
// main.ino (UPDATED)
// Fixes applied (from review):
//  1) Removed duplicated baro "auto-mode" block in setup (and the buggy one).
//  2) Made baro auto-mode robust (Welford online variance) INSIDE setup.
//  3) verticalAxisUpdate now uses physical yaw (without yaw_offset), not yaw_out.
//  4) Removed redundant cal_isValid+cal_setDefaults in setup (cal_load already defaults).
//     (Still prints whether EEPROM had valid calibration by peeking first.)
//  5) Minor: renamed bmp_ok in setup to bmp_read_ok for clarity.
// ========================

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

#include "globals.h"
#include "math_utils.h"
#include "mag_yaw.h"
#include "kalman_vz.h"
#include "fsm.h"
#include "telemetry.h"
#include "attitude.h"
#include "baro_alt.h"
#include "vertical_axis.h"
#include "sensors.h"

// ---------------------------------------------------------
// Robust baro stability measurement (online mean/variance)
// Returns true if enough valid samples were collected.
// Also outputs sigmaP (Pa).
// ---------------------------------------------------------
static bool baroMeasureSigmaPa_Welford(Adafruit_BMP3XX &bmp, int Np, float &sigmaP_out) {
  int   n    = 0;
  float mean = 0.0f;
  float M2   = 0.0f;

  for (int i = 0; i < Np; i++) {
    if (bmp.performReading()) {
      const float x = bmp.pressure;
      n++;
      const float delta  = x - mean;
      mean += delta / (float)n;
      const float delta2 = x - mean;
      M2 += delta * delta2;
    }
    delay(100);
  }

  // Require at least 80% valid
  if (n < (Np * 8) / 10) return false;

  const float var = (n > 1) ? (M2 / (float)n) : 0.0f;
  sigmaP_out = sqrtf(fmaxf(var, 0.0f));
  return true;
}

// ---------------------------------------------------------
// Apply your indoor/moderate/outdoor mode based on sigmaP.
// Also updates sigma_baro + R_baro (KF measurement variance).
// ---------------------------------------------------------
static void baroApplyAutoModeFromSigma(float sigmaP) {
  if (sigmaP < 1.5f) {
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_63);
    sigma_baro = 2.5f;
    Serial.printf("Innendørsmodus aktivert (σ = %.2f Pa)\n", sigmaP);
  } else if (sigmaP < 5.0f) {
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
    sigma_baro = 1.8f;
    Serial.printf("Moderat modus aktivert (σ = %.2f Pa)\n", sigmaP);
  } else {
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    sigma_baro = 1.2f;
    Serial.printf("Utendørsmodus aktivert (σ = %.2f Pa)\n", sigmaP);
  }
  R_baro = sigma_baro * sigma_baro;
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial && millis() < 6000);
  Serial.println("Flight firmware started");

  Wire.begin();
  Wire.setClock(400000);
  SPI.begin();

  // ---------- LAST KALIBRERINGSVERDIER ----------
  // NOTE: cal_load() already applies defaults if invalid.
  // If you want correct logging about whether EEPROM was valid, we peek first.
  CalibrationData tmpPeek;
  EEPROM.get(CAL_EEPROM_ADDR, tmpPeek);
  const bool hadValidCal = cal_isValid(tmpPeek);

  cal_load(CAL);
  Serial.println(hadValidCal
                   ? F("EEPROM: Kalibrering lastet")
                   : F("EEPROM: Ingen gyldig kalibrering funnet. Bruker standardverdier."));

  gyro_offset_x = CAL.gyroBias[0];
  gyro_offset_y = CAL.gyroBias[1];
  gyro_offset_z = CAL.gyroBias[2];

  adxl_xOff     = CAL.adxlBias[0] * g0;
  adxl_yOff     = CAL.adxlBias[1] * g0;
  adxl_zOff     = CAL.adxlBias[2] * g0;

  mag_xBias     = CAL.magOffset[0];
  mag_yBias     = CAL.magOffset[1];
  mag_zBias     = CAL.magOffset[2];
  mag_xScale    = (CAL.magScale[0] > 0) ? 1.0f / CAL.magScale[0] : 1.0f;
  mag_yScale    = (CAL.magScale[1] > 0) ? 1.0f / CAL.magScale[1] : 1.0f;
  mag_zScale    = (CAL.magScale[2] > 0) ? 1.0f / CAL.magScale[2] : 1.0f;

  Serial.println(F("------ EEPROM Data ------"));
  Serial.printf("Gyro bias [rad/s]: %+0.6f %+0.6f %+0.6f\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);
  Serial.printf("ADXL bias [m/s²]:  %+0.3f %+0.3f %+0.3f\n", adxl_xOff, adxl_yOff, adxl_zOff);
  Serial.printf("Mag offset [uT]:   %+0.2f %+0.2f %+0.2f\n", mag_xBias, mag_yBias, mag_zBias);
  Serial.printf("Mag scale inv:     %.3f %.3f %.3f\n", mag_xScale, mag_yScale, mag_zScale);
  Serial.println(F("-------------------------"));

  // ---------- SENSORINIT ----------
  if (!icm.begin_SPI(CS_ICM, &SPI)) {
    while (1) { Serial.println("ICM20649 init fail"); delay(500); }
  }
  icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  delay(10);

  if (!adxl.begin()) Serial.println("ADXL375 init fail");

  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL init fail");
  } else {
    lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  }

  // --- BMP init: present vs reading ok ---
  bmp_present = bmp.begin_I2C();

  if (bmp_present) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    delay(300);

    // Bekreft at vi får data
    const bool bmp_read_ok = bmp.performReading();

        if (bmp_read_ok) {

      // -------------------------------
      // (1) La sensoren stabilisere seg litt først
      // -------------------------------
      delay(800);

      // -------------------------------
      // (2) Auto indoor/moderate/outdoor basert på trykkstabilitet
      //     (velger endelig IIR + sigma_baro/R_baro)
      // -------------------------------
      Serial.println("Måler barotrykkstabilitet...");
      float sigmaP = 0.0f;
      if (baroMeasureSigmaPa_Welford(bmp, 50, sigmaP)) {
        baroApplyAutoModeFromSigma(sigmaP);
      } else {
        Serial.println("BMP: for mange ugyldige samples til stabilitetsmåling. Bruker default IIR=7.");
        // behold IIR=7 som du satte tidligere
        R_baro = sigma_baro * sigma_baro;
      }

      // -------------------------------
      // (3) La ny IIR "sette seg"
      // -------------------------------
      delay(800);

      // -------------------------------
      // (4) Sett baro-nullpunkt (baseline p0) ETTER auto-mode
      //     Flere samples + discard av de første
      // -------------------------------
      float p_sum = 0.0f;
      int   p_cnt = 0;
      const int N = 50;       // mer robust enn 10
      const int DISCARD = 10; // kast de første

      for (int i = 0; i < N; i++) {
        if (bmp.performReading()) {
          if (i >= DISCARD) {
            p_sum += bmp.pressure;
            p_cnt++;
          }
        }
        delay(50);
      }

      if (p_cnt >= 20) { // minst 20 gode
        bmp_p0_Pa = p_sum / (float)p_cnt;
        Serial.printf("Baro baseline satt: %.2f Pa (0 m)\n", bmp_p0_Pa);

        // Init KF
        z_est = 0.0f; v_est = 0.0f;
        P00 = 10; P01 = 0; P10 = 0; P11 = 10;
        R_baro = sigma_baro * sigma_baro;

      } else {
        Serial.println("BMP: for få gyldige samples til baseline. Fortsetter uten baro.");
        bmp_present = false; // "disable"
      }

    } else {
      Serial.println("BMP: performReading() feilet i setup. Fortsetter uten baro.");
      bmp_present = false; // "disable"
    }

  } else {
    Serial.println("BMP: ikke funnet (begin_I2C feilet). Fortsetter uten baro.");
  }

  // ---------- Første roll/pitch fra gravitasjon ----------
  sensors_event_t a0, g0ev, t0;
  icm.getEvent(&a0, &g0ev, &t0);
  roll  = rad2deg(atan2f(a0.acceleration.y, a0.acceleration.z));
  pitch = rad2deg(atan2f(-a0.acceleration.x,
                         sqrtf(a0.acceleration.y * a0.acceleration.y +
                               a0.acceleration.z * a0.acceleration.z)));

  // ---------- Initial yaw alignment (tilt-kompensert mag) ----------
  delay(200);
  float yawMag0 = 0.0f;
  if (computeMagYawDeg(roll, pitch, yawMag0)) {
    yaw = yawMag0;
    yaw_offset = -yawMag0;
    Serial.printf("Start yaw (mag tilt-comp) = %.1f°, yaw_offset = %.1f°\n", yawMag0, yaw_offset);
  } else {
    Serial.println("Magnetometer ikke tilgjengelig for yaw-init");
    yaw = 0.0f;
    yaw_offset = 0.0f;
  }

  stateStart = millis();
  lastMicros = micros();
}

void loop() {
  // Les sensorer (inkl bmp snapshot bare én gang pr loop)
  SensorFrame s = readSensors();

  // ---------- Gyro (rad/s -> deg/s) + yaw_out ----------
  float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
  float yaw_out = 0.0f;

  attitudeUpdate(s.dt, s.icm_acc, s.icm_gyro, gx_dps, gy_dps, gz_dps, yaw_out);

  // ---------- ADXL375 til CSV (i g) ----------
  const float ax_g = (s.adxl.acceleration.x - adxl_xOff) / g0;
  const float ay_g = (s.adxl.acceleration.y - adxl_yOff) / g0;
  const float az_g = (s.adxl.acceleration.z - adxl_zOff) / g0;

  // ---------- Temperatur/Trykk/Høyde ----------
  float temperatureC = 0.0f;
  float pressurePa   = 0.0f;
  float altitude_m   = 0.0f;
  baroCompute(s.bmp_ok, s.bmp_tempC, s.bmp_pressPa,
              temperatureC, pressurePa, altitude_m);

  // =========================================================
  //   Vertical axis
  // =========================================================
  float az_filt    = 0.0f;
  float a_norm     = 0.0f;
  float a_net_filt = 0.0f;

  // IMPORTANT FIX:
  // Use physical yaw (no offset) for rotation; yaw_out is for logging/UI.
  verticalAxisUpdate(s.icm_acc, roll, pitch, yaw, g0,
                     az_filt, a_norm, a_net_filt);

  // =========================================================
  //   Kalman VZ
  // =========================================================
  float altitude_est_m = 0.0f;
  float vz_est_mps     = 0.0f;

  kalmanVzUpdate(s.dt, az_filt, altitude_m, s.bmp_ok,
                 gx_dps, gy_dps, gz_dps,
                 a_norm,
                 altitude_est_m, vz_est_mps);

  // ---------- Flight-time ----------
  const float tSec = (state >= LIFT_OFF) ? (millis() - LIFT_OFFStart) / 1000.0f : 0.0f;

  // =========================================================
  //   FSM
  // =========================================================
  fsmUpdate(s.bmp_ok, altitude_m, a_net_filt);

  // ---------- CSV ----------
  const float vel_out   = vz_est_mps;
  const float press_out = (pressurePa > 0.0f) ? (pressurePa / 101325.0f) : 0.0f;
  const long  alt_out   = (long)lroundf(altitude_est_m);

  emitCsv(tSec, ax_g, ay_g, az_g,
          pitch, roll, yaw_out,
          temperatureC, vel_out, press_out,
          drogueFired ? 1 : 0, mainFired ? 1 : 0,
          alt_out, altitude_m, (int)state);

  // ---------- Delay ----------
  delayByState();
}
