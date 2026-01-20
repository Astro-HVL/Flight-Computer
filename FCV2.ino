#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>

#include <Adafruit_AHRS_Madgwick.h>
Adafruit_Madgwick filter;

#include "calibration_data.h"
CalibrationData CAL;


#define SPI_SCK   13
#define SPI_MISO  12
#define SPI_MOSI  11
#define CS_ADXL   10
#define CS_ICM    15


Adafruit_ADXL375   adxl(CS_ADXL, &SPI, 12345);
Adafruit_ICM20649  icm;
Adafruit_LIS3MDL   lis3mdl;
Adafruit_BMP3XX    bmp;

//     KALIBRERINGSVARIABLER (lastes fra EEPROM)
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float adxl_xOff = 0, adxl_yOff = 0, adxl_zOff = 0;
float mag_xBias = 0, mag_yBias = 0, mag_zBias = 0;
float mag_xScale = 1.0f, mag_yScale = 1.0f, mag_zScale = 1.0f;
float decl_deg = 0.0f;
float bmp_p0_Pa = 101325.0f;
float bmp_tOff = 0.0f;
float bmp_pOff = 0.0f;

//   TILSTAND & DYNAMIKK
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float yaw_offset = 0.0f;
float alpha_rp = 0.98f;
unsigned long lastMicros = 0;

unsigned long seq = 0;
long   alt_sim = 0;
float  vel_sim = 0;
float  a_up = 10.0f;
float  g0 = 9.80665f;
unsigned long LIFT_OFFStart = 0;


enum State {
  SYSTEM_CHECK = 1,
  OPERATION_READY,
  LIFT_OFF,
  APOGEE,
  PARACHUTE_DEPLOY
};
State state = SYSTEM_CHECK;
State lastState = SYSTEM_CHECK;
unsigned long stateStart = 0;

//      HJELPEFUNKSJONER
static inline float rad2deg(float r){ return r * 57.2957795f; }
static inline float deg2rad(float d){ return d * 0.01745329252f; }
static inline float wrap180(float a){
  while (a > 180.f)  a -= 360.f;
  while (a < -180.f) a += 360.f;
  return a;
}

//   FYSIKK-FUNKSJONER
float pressureAtHeight(float h) {
  if (h < 11000) return 101325 * pow(1 - 0.0065 * h / 288.15, 5.2561);
  else if (h < 20000) return 22632 * exp(-0.000157 * (h - 11000));
  else if (h < 32000) return 5474 * pow(1 + 0.001 * (h - 20000) / 216.65, -34.1632);
  else if (h < 47000) return 868 * pow(1 - 0.0028 * (h - 32000) / 228.65, 12.2016);
  else if (h < 51000) return 110 * exp(-0.000157 * (h - 47000));
  else if (h < 71000) return 66 * pow(1 - 0.0028 * (h - 51000) / 270.65, -12.2016);
  else return 0.12;
}
float temperatureAtHeight(float h) {
  if (h < 11000) return 15 - 0.0065 * h;
  else if (h < 20000) return -56.5;
  else if (h < 32000) return -56.5 + 0.001 * (h - 20000);
  else if (h < 47000) return -44.5 + 0.0028 * (h - 32000);
  else if (h < 51000) return -2.5;
  else if (h < 71000) return -2.5 - 0.0028 * (h - 51000);
  else return -58.5;
}

//   MAGNETOMETER-FUNKSJONER
bool computeMagYawDeg(float rollDeg, float pitchDeg, float& yawMagDeg){
  sensors_event_t magEv;
  if (!lis3mdl.getEvent(&magEv)) return false;

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

float magTrust(const float yawMagDeg, const float lastYaw, const float dt){
  float dy = fabsf(wrap180(yawMagDeg - lastYaw));
  float speed_ok = (dy <= 30.0f * dt);
  return speed_ok ? 1.0f : 0.15f;
}

// --- Kalman: z-v med aksel-input ---
static float z_est = 0.0f, v_est = 0.0f;     // tilstandsestimat
static float P00 = 1, P01 = 0, P10 = 0, P11 = 1; // kovarians

// Tuning (startforslag):
static float sigma_acc = 0.4f;   // m/s^2 (effektiv støy i az_lin etter filtrering)
static float sigma_model_v = 0.15f;  // m/s (uforutsigbarhet utover aksel-input)
static float sigma_baro = 1.5f;  // m (baro-høydestøy etter baseline & LPF)

// avledete kovarianser:
static float R_baro;   // = sigma_baro^2


void softReset() {
  SCB_AIRCR = 0x05FA0004;  // Myk programreset uten bootloader
}


//   CSV-EMITTER (til PC-dashboard)
inline void emitCsv(float tSec, float ax_g, float ay_g, float az_g,
                    float pitch, float roll, float yaw,
                    float temperature, float vel, float press,
                    long lat, long lon, long alt_m, int state)
{
  Serial.print(tSec, 1); Serial.print(',');
  Serial.print(seq++);   Serial.print(',');
  Serial.print(ax_g);    Serial.print(',');
  Serial.print(ay_g);    Serial.print(',');
  Serial.print(az_g, 1);    Serial.print(',');
  Serial.print(pitch, 1);   Serial.print(',');
  Serial.print(roll, 1);    Serial.print(',');
  Serial.print(yaw, 1);     Serial.print(',');
  Serial.print(temperature); Serial.print(',');
  Serial.print(vel);     Serial.print(',');
  Serial.print(press);   Serial.print(',');
  Serial.print(lat);     Serial.print(',');
  Serial.print(lon);     Serial.print(',');
  Serial.print(alt_m);   Serial.print(',');
  Serial.println((int)state);
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial && millis() < 6000);
  Serial.println("✅ Flight firmware started");

  filter.begin(100);  // sample rate ~ din loopfrekvens
  filter.setBeta(0.05f);  // mindre drift, roligere yaw
  Wire.begin();
  Wire.setClock(400000);
  SPI.begin();

  // ---------- LAST KALIBRERINGSVERDIER ----------
  cal_load(CAL);
  if (!cal_isValid(CAL)) {
    Serial.println(F("⚠️ EEPROM: Ingen gyldig kalibrering funnet. Bruker standardverdier."));
    cal_setDefaults(CAL);
  } else {
    Serial.println(F("✅ EEPROM: Kalibrering lastet"));
  }

  // Overfør til lokale variabler
  gyro_offset_x = CAL.gyroBias[0];
  gyro_offset_y = CAL.gyroBias[1];
  gyro_offset_z = CAL.gyroBias[2];

  adxl_xOff = CAL.adxlBias[0] * g0;
  adxl_yOff = CAL.adxlBias[1] * g0;
  adxl_zOff = CAL.adxlBias[2] * g0;

  mag_xBias  = CAL.magOffset[0];
  mag_yBias  = CAL.magOffset[1];
  mag_zBias  = CAL.magOffset[2];
  mag_xScale = (CAL.magScale[0] > 0) ? 1.0f / CAL.magScale[0] : 1.0f;
  mag_yScale = (CAL.magScale[1] > 0) ? 1.0f / CAL.magScale[1] : 1.0f;
  mag_zScale = (CAL.magScale[2] > 0) ? 1.0f / CAL.magScale[2] : 1.0f;

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

  // ---------- INITIAL YAW ALIGNMENT ----------
  delay(500);  // kort ventetid for å sikre stabile mag-data
  sensors_event_t m;
  if (lis3mdl.getEvent(&m)) {
    float yawMag0 = atan2f(m.magnetic.y, m.magnetic.x) * 57.2958f;
    yaw = yawMag0;          // behold magnetisk yaw som +-vinkel
    yaw_offset = -yawMag0;  // offset slik at nåværende retning blir 0°
    Serial.printf("🧭 Start yaw justert til %.1f° (mag)\n", yaw);
  } else {
    Serial.println("⚠️  Magnetometer ikke tilgjengelig for yaw-init");
  }

  bool bmp_ok = bmp.begin_I2C();
  if (bmp_ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    delay(300);

    // Sett baro-nullpunkt automatisk ---
    if (bmp.performReading()) {
      // Vent litt for å la sensoren stabilisere seg
      delay(1000);

      // Les gjennomsnitt av flere målinger for et mer stabilt nullpunkt
      float p_sum = 0;
      const int N = 10;
      for (int i = 0; i < N; i++) {
        bmp.performReading();
        p_sum += bmp.pressure;
        delay(100);
      }
      bmp_p0_Pa = p_sum / N;

      Serial.printf("🌍 Baro baseline satt: %.2f Pa (0 m)\n", bmp_p0_Pa);
    }
    z_est = 0.0f; v_est = 0.0f;
    P00 = 10; P01 = 0; P10 = 0; P11 = 10;   // litt usikker start
    R_baro = sigma_baro * sigma_baro;
  }

  // --- Automatisk innendørs/utendørs-modus basert på trykkvariasjon ---
  float p_samples[50];
  const int Np = 50;
  Serial.println("📡 Måler barotrykkstabilitet...");

  for (int i = 0; i < Np; i++) {
    bmp.performReading();
    p_samples[i] = bmp.pressure;
    delay(100);  // 50 sample → ~5 sek
  }

  // --- Juster første roll/pitch etter gravitasjon ---
  sensors_event_t a, g, t;
  icm.getEvent(&a, &g, &t);
  roll  = rad2deg(atan2f(a.acceleration.y, a.acceleration.z));
  pitch = rad2deg(atan2f(-a.acceleration.x,
              sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)));

  // Beregn gjennomsnitt og standardavvik
  float meanP = 0, varP = 0;
  for (int i = 0; i < Np; i++) meanP += p_samples[i];
  meanP /= Np;
  for (int i = 0; i < Np; i++) varP += (p_samples[i] - meanP)*(p_samples[i] - meanP);
  varP /= Np;
  float sigmaP = sqrtf(varP);

  if (sigmaP < 1.5f) {
    // Innendørsmodus – rolig luft, høyere støy og sterkere filter
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_63);
    sigma_baro = 2.5f;   // øk usikkerhet (mindre vekt på baro)
    Serial.printf("🏠 Innendørsmodus aktivert (σ=%.2f Pa)\n", sigmaP);
  } else if (sigmaP < 5.0f) {
    // Moderat miljø (f.eks. rolig utendørs)
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
    sigma_baro = 1.8f;
    Serial.printf("🌤️  Moderat modus aktivert (σ=%.2f Pa)\n", sigmaP);
  } else {
    // Utendørs / flyvning
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    sigma_baro = 1.2f;
    Serial.printf("🚀 Utendørsmodus aktivert (σ=%.2f Pa)\n", sigmaP);
  }
  R_baro = sigma_baro * sigma_baro;


  // ---------- STARTVARIABLER ----------
  stateStart = millis();
  lastMicros = micros();
}


void loop() {
  // ---------- Les sensorer ----------
  sensors_event_t adxlEv;
  adxl.getEvent(&adxlEv);  // m/s^2

  sensors_event_t icm_accelEv, icm_gyroEv, icm_tempEv;
  icm.getEvent(&icm_accelEv, &icm_gyroEv, &icm_tempEv);

  bool bmp_ok = bmp.performReading();   // oppdater BMP målinger

  // ---------- dt ----------
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // --- Gyro mapping for 3D-modell (ZYX rotasjonsrekkefølge)
  float gx =  icm_gyroEv.gyro.x - gyro_offset_x; // rad/s
  float gy =  icm_gyroEv.gyro.y - gyro_offset_y;
  float gz =  icm_gyroEv.gyro.z - gyro_offset_z;

  // til grader/s
  //gx *= 57.2958f;
  //gy *= 57.2958f;
  //gz *= 57.2958f;

  // ---------- Oppdater magnetometer før filter ---------- 
  sensors_event_t m;
  lis3mdl.getEvent(&m);

  // ---------- Madgwick update (gyro rad/s) ----------
  filter.update(
    icm_gyroEv.gyro.y - gyro_offset_y,    // yaw-akse (bytt rekkefølge)
    icm_gyroEv.gyro.x - gyro_offset_x,    // pitch
    -(icm_gyroEv.gyro.z - gyro_offset_z), // roll (negativ omvendt)
    icm_accelEv.acceleration.y,
    icm_accelEv.acceleration.x,
    -icm_accelEv.acceleration.z,
    m.magnetic.y, m.magnetic.x, -m.magnetic.z,
    dt
  );

  roll  = rad2deg(filter.getRoll());
  pitch = rad2deg(filter.getPitch());
  yaw   = rad2deg(filter.getYaw()) + yaw_offset;

  // ---------- Roll/Pitch via komplementært filter (ICM-accel + gyro) ----------
  //float rollAcc = rad2deg(atan2f(icm_accelEv.acceleration.y, icm_accelEv.acceleration.z));
  //float pitchAcc = rad2deg(atan2f(-icm_accelEv.acceleration.x,
   //                      sqrtf(icm_accelEv.acceleration.y*icm_accelEv.acceleration.y +
    //                           icm_accelEv.acceleration.z*icm_accelEv.acceleration.z)));

  //roll  = alpha_rp * (roll  + gx * dt) + (1.0f - alpha_rp) * rollAcc;
  //pitch = alpha_rp * (pitch + gy * dt) + (1.0f - alpha_rp) * pitchAcc;

  // ---------- ADXL375 akselerasjoner til CSV (i g) ----------
  // ADXL offsets er i m/s^2 slik at vi matcher getEvent()
  float ax_g = (adxlEv.acceleration.x - adxl_xOff) / g0;
  float ay_g = (adxlEv.acceleration.y - adxl_yOff) / g0;
  float az_g = (adxlEv.acceleration.z - adxl_zOff) / g0;

  // ---------- Temperatur/Trykk/Høyde ----------
  float temperatureC = bmp_ok ? (bmp.temperature + bmp_tOff) : temperatureAtHeight(alt_sim);
  float pressurePa   = bmp_ok ? (bmp.pressure + bmp_pOff)   : pressureAtHeight(alt_sim);
  float altitude_m   = 0.0f;

  if (bmp_ok && bmp_p0_Pa > 10000.0f) {
    // standard barometrisk formel
    altitude_m = 44330.0f * (1.0f - powf(pressurePa / bmp_p0_Pa, 0.1903f));
  } else {
    altitude_m = (float)alt_sim; // fallback til simulert alt
  }

  // ---------- NØYAKTIG HASTIGHETSBEREGNING MED KALMAN-FILTER ----------

  // --- 1) Grav-kompensert vertikal aksel ---
  float az = icm_accelEv.acceleration.z;
  float g_comp = g0 * cosf(deg2rad(pitch)) * cosf(deg2rad(roll));
  float az_lin = az - g_comp;

  // Mild filtrering + dead-zone
  static float az_filt = 0;
  az_filt = 0.25f * az_lin + 0.75f * az_filt;
  if (fabs(az_filt) < 0.05f) az_filt = 0.0f;

  // ---------- Yaw: gyro-integrasjon + magnetometer-korreksjon ----------
  //float yaw_gyro = wrap180(yaw + gz * dt);

  //float yawMagDeg;
 // bool haveMag = computeMagYawDeg(roll, pitch, yawMagDeg);

  //static float yawMagFilt = 0.0f;  // lavpass på magnetometeret
  //if (haveMag) {
   // yawMagFilt = 0.8f * yawMagFilt + 0.2f * yawMagDeg;

    // Beregn magnetvekt basert på gyroaktivitet (mindre vekt ved høy rotasjon)
    //float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    //float magWeight = constrain(1.0f - (gyro_mag / 100.0f), 0.05f, 0.3f);

    // Kombiner gyro og magnetometer
    //yaw = wrap180((1.0f - magWeight) * yaw_gyro + magWeight * yawMagFilt);
  //} else {
   // yaw = yaw_gyro;
  //}

  // --- 2) KF: PREDIKSJON ---
  float F00 = 1.0f, F01 = dt;
  float F10 = 0.0f, F11 = 1.0f;
  float Bz0  = 0.5f * dt * dt;
  float Bz1  = dt;

  // Prosess-støy Q
  float q_aa = sigma_acc * sigma_acc;
  float q_vv = sigma_model_v * sigma_model_v;
  float Q00 = Bz0*Bz0 * q_aa;
  float Q01 = Bz0*Bz1 * q_aa;
  float Q10 = Q01;
  float Q11 = Bz1*Bz1 * q_aa + q_vv;

  // Prediker tilstand
  float z_pred = F00*z_est + F01*v_est + Bz0*az_filt;
  float v_pred = F10*z_est + F11*v_est + Bz1*az_filt;

  // Prediker kovarians
  float P00p = F00*P00 + F01*P10;
  float P01p = F00*P01 + F01*P11;
  float P10p = F10*P00 + F11*P10;
  float P11p = F10*P01 + F11*P11;
  float P00pp = P00p*F00 + P01p*F01 + Q00;
  float P01pp = P00p*F10 + P01p*F11 + Q01;
  float P10pp = P10p*F00 + P11p*F01 + Q10;
  float P11pp = P10p*F10 + P11p*F11 + Q11;

  // --- 3) KF: OPPDATERING MED BARO-HØYDE ---
  if (bmp_ok) {
    float y  = altitude_m - z_pred;   // innovasjon
    float S  = P00pp + R_baro;        // innovasjonskovarians
    float K0 = P00pp / S;             // gain (z)
    float K1 = P10pp / S;             // gain (v)

    // Oppdater estimat
    z_est = z_pred + K0 * y;
    v_est = v_pred + K1 * y;

    // Oppdater kovarians
    float P00n = (1 - K0)*P00pp;
    float P01n = (1 - K0)*P01pp;
    float P10n = -K1*P00pp + P10pp;
    float P11n = -K1*P01pp + P11pp;
    P00 = P00n; P01 = P01n; P10 = P10n; P11 = P11n;
  } else {
    // Ingen baro → kun prediksjon
    z_est = z_pred; 
    v_est = v_pred;
    P00 = P00pp; P01 = P01pp; P10 = P10pp; P11 = P11pp;
  }

  // Raketten stått i ro i mer enn 2 sek? Sett hastighet og høyde lik 0
  static unsigned long stillTimer = 0;

  float gx_dps = (icm_gyroEv.gyro.x - gyro_offset_x) * 57.29578f;
  float gy_dps = (icm_gyroEv.gyro.y - gyro_offset_y) * 57.29578f;
  float gz_dps = (icm_gyroEv.gyro.z - gyro_offset_z) * 57.29578f;

  float a_norm = sqrtf(icm_accelEv.acceleration.x*icm_accelEv.acceleration.x +
                      icm_accelEv.acceleration.y*icm_accelEv.acceleration.y +
                      icm_accelEv.acceleration.z*icm_accelEv.acceleration.z);

  bool gyro_quiet = (fabsf(gx_dps) < 1.0f && fabsf(gy_dps) < 1.0f && fabsf(gz_dps) < 1.0f);
  bool accel_quiet = fabsf(a_norm - g0) < 0.08f*g0;     // |a|-norm nær 1 g
  bool baro_quiet  = fabsf(altitude_m - z_est) < 0.15f; // måling≈estimat

  // Korrigering av gyrodrift
  //if ((state == SYSTEM_CHECK || state == OPERATION_REATION) && gyro_quiet && accel_quiet && baro_quiet && fabsf(v_est) < 0.5f) {
  //   if (millis() - stillTimer > 2000) {
  //     v_est = 0.0f;
  //     z_est = 0.0f;

      // Langsom oppdatering av gyro-bias når raketten står stille
  //    gyro_offset_x = 0.995f * gyro_offset_x + 0.005f * icm_gyroEv.gyro.x;
  //    gyro_offset_y = 0.995f * gyro_offset_y + 0.005f * icm_gyroEv.gyro.y;
  //    gyro_offset_z = 0.995f * gyro_offset_z + 0.005f * icm_gyroEv.gyro.z;
  //  }
  //  } else {
  //   stillTimer = millis();
  // }

  // Nå er høyde og hastighet filtrert:
  float altitude_est_m = z_est;
  float vz_est_mps     = v_est;

  // Holde høyden på 0 m når ikke i flight modus
  bool onPad = (state == SYSTEM_CHECK || state == OPERATION_READY);

  if (onPad) {
    // bredere snap nær 0 når du er inne / på pad
    const float ALT_SNAP = 1.0f;   // meter
    const float VZ_SNAP  = 0.15f;  // m/s

    if (fabsf(altitude_est_m) < ALT_SNAP) {
      altitude_est_m = 0.0f;
      z_est = 0.0f;
    }
    if (fabsf(vz_est_mps) < VZ_SNAP) {
      vz_est_mps = 0.0f;
      v_est = 0.0f;
    }

    // aldri rapporter negativ høyde på pad
    if (altitude_est_m < 0.0f) {
      altitude_est_m = 0.0f;
      z_est = 0.0f;
    }
  }

  // Tiny output clamp (etter nulling)
  if (fabsf(vz_est_mps) < 0.05f) vz_est_mps = 0.0f;
  if (fabsf(altitude_est_m) < 0.05f) altitude_est_m = 0.0f;

  // --- Smart deadzone: bruk KF-usikkerhet ---
  float v_sigma = sqrtf(fmaxf(P11, 1e-6f));   // hastighetsstandardavvik
  float v_thresh = 2.5f * v_sigma;            // ~99% konfidens
  if (fabsf(v_est) < v_thresh) v_est = 0.0f;

  // ---------- Flight-time (sekunder) ----------
  float tSec = (state >= LIFT_OFF) ? (millis() - LIFT_OFFStart) / 1000.0f : 0.0f;

  // =============================
  //       TILSTANDSMASKIN
  // =============================
  switch (state) {
    case SYSTEM_CHECK:
      if (millis() - stateStart > 10000) {
        state = OPERATION_READY;
        stateStart = millis();
      }
      break;

    case OPERATION_READY: {
      // --- Robust launch detection ---
      static int accelCounter = 0;      // teller høy aksel-samples
      static int baroCounter  = 0;      // teller stigende høyde
      static unsigned long quietTimer = 0;

      // 1️⃣ Beregn total akselerasjonsnorm (uavhengig av orientering)
      float a_norm = sqrtf(
          icm_accelEv.acceleration.x * icm_accelEv.acceleration.x +
          icm_accelEv.acceleration.y * icm_accelEv.acceleration.y +
          icm_accelEv.acceleration.z * icm_accelEv.acceleration.z
      );

      // 2️⃣ Grav-kompensert vertikal aksel (for redundans)
      float az_lin_launch = icm_accelEv.acceleration.z - g0;

      // 3️⃣ Arming: må ha vært rolig minst 0.5 s
      bool stableBefore = (millis() - quietTimer > 500);
      if (fabsf(v_est) > 0.5f || fabsf(az_lin_launch) > 2.0f)
        quietTimer = millis();

      // 4️⃣ Aksel- og baro-kriterier
      if (a_norm > 15.0f || az_lin_launch > 8.0f) accelCounter++;   // 1.5 g eller 8 m/s²
      else accelCounter = 0;

      if (fabsf(v_est) > 1.0f || altitude_m > 1.5f) baroCounter++;
      else baroCounter = 0;

      // 5️⃣ Kombinert kriterium                                    // !!!!!!!!!!!!!!!!!!!!VIKTIG!!!!!!!!!!!!!!!!!!!!!!!!!!
      if (stableBefore && accelCounter >= 2 || baroCounter >= 2) { // !!!!!!!!!!MÅ NOK TWEAKES LITT FØR LAUNCH!!!!!!!!!!!!!
        state = LIFT_OFF;
        LIFT_OFFStart = millis();
        stateStart = millis();

        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
        sigma_baro = 1.2f;
        R_baro = sigma_baro * sigma_baro;
      }

      break;
    }

    case LIFT_OFF:
      // Bytt til APOGEE ved høyde > 4500 m (enten fra BMP eller simulert)
      if ((bmp_ok && altitude_m >= 4500.0f) || (!bmp_ok && alt_sim >= 4500)) {
        state = APOGEE;
        stateStart = millis();
      }
      break;

    case APOGEE:
      // Liten dwell før fallskjerm
      if (millis() - stateStart > 3000) {
        state = PARACHUTE_DEPLOY;
        stateStart = millis();
      }
      break;

    case PARACHUTE_DEPLOY:
      // Simulert fall – i ekte flight er det BMP som gir altitude/vel
      if (!bmp_ok) {
        vel_sim *= 0.9f;
        if (vel_sim < 50) vel_sim = 50;
        alt_sim -= 50;
        if (alt_sim < 0) alt_sim = 0;
      }
      break;
  }

  // ---------- JSON ved tilstandsendring ----------
  if (state != lastState) {
    Serial.print('{'); Serial.print("\"state\":"); Serial.print((int)state); Serial.println('}');
    Serial.flush();

    // Reset vinkler når systemet blir klart til launch
    //if (state == OPERATION_READY && lastState == SYSTEM_CHECK) {
      //roll = 0.0f;
      //pitch = 0.0f;
      //// yaw = 0.0f;
    //}

    lastState = state;
  }

  float vel_out   = vz_est_mps;  // hold som før; (valgfritt) legg inn barometrisk Vz-filter senere
  float press_out = pressurePa / 101325.0f; // app viser "atm" – leverer i atm
  long  alt_out   = (long)lroundf(altitude_est_m);

  // Fiktiv posisjon – beholdt fra original
  long lat = 6039290 + (long)(seq * 2);
  long lon = 532410 + (long)(seq * 2);

  emitCsv(tSec, ax_g, ay_g, az_g,
          pitch, roll, yaw,
          temperatureC, vel_out, press_out,
          lat, lon, alt_out, (int)state);

  // ---------- Oppdateringsfrekvens pr state ----------
  switch (state) {
    case SYSTEM_CHECK:      delay(100); break;
    case OPERATION_READY:   delay(50); break;
    case LIFT_OFF:          delay(5);  break; // raskere i flight for bedre respons
    case APOGEE:            delay(10);  break;
    case PARACHUTE_DEPLOY:  delay(10);  break;
  }
}
