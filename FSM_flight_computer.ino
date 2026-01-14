#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>

#include "calibration_data.h"
CalibrationData CAL;

// PIN-KONFIG
#define SPI_SCK   13
#define SPI_MISO  12
#define SPI_MOSI  11
#define CS_ADXL   10
#define CS_ICM    15
// #define fallskjermpin

// SENSOR-OBJEKTER
Adafruit_ADXL375   adxl(CS_ADXL, &SPI, 12345);
Adafruit_ICM20649  icm;
Adafruit_LIS3MDL   lis3mdl;
Adafruit_BMP3XX    bmp;

// Fallskjerm
static bool parachuteFired = false;

// KALIBRERINGSVARIABLER (lastes fra EEPROM)
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float adxl_xOff = 0, adxl_yOff = 0, adxl_zOff = 0;
float mag_xBias = 0, mag_yBias = 0, mag_zBias = 0;
float mag_xScale = 1.0f, mag_yScale = 1.0f, mag_zScale = 1.0f;
float decl_deg = 0.0f;
float bmp_p0_Pa = 101325.0f;
float bmp_tOff = 0.0f;
float bmp_pOff = 0.0f;

// TILSTAND & DYNAMIKK
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float alpha_rp = 0.98f;
unsigned long lastMicros = 0;

unsigned long seq = 0;
float  g0 = 9.80665f;
unsigned long LIFT_OFFStart = 0;

// VERTIKAL HASTIGHET (baro + accel)
float alt_baro_prev = 0.0f;   // For differanseberegning
float alt_baro_filt = 0.0f;   // Lavpasset høyde
float vz_baro = 0.0f;         // d(alt)/dt fra baro
float vz_acc  = 0.0f;         // Integrert akselerasjon (grav-kompensert)
float vz_filt = 0.0f;         // Flettet resultat av begge (endelig vertikal hastighet)


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

// Beregning av trykk og temperatur
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

// MAGNETOMETER-FUNKSJONER
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

void softReset() {
  SCB_AIRCR = 0x05FA0004;  // Myk programreset uten bootloader
}

//   CSV-EMITTER (til PC-dashboard)
inline void emitCsv(float tSec, float ax_g, float ay_g, float az_g,
                    float pitch, float roll, float yaw,
                    float temperature, float vel, float press,
                    long lat, long lon, long alt_m, int state)
{
  Serial.print(tSec, 3); Serial.print(',');
  Serial.print(seq++);   Serial.print(',');
  Serial.print(ax_g);    Serial.print(',');
  Serial.print(ay_g);    Serial.print(',');
  Serial.print(az_g);    Serial.print(',');
  Serial.print(pitch);   Serial.print(',');
  Serial.print(roll);    Serial.print(',');
  Serial.print(yaw);     Serial.print(',');
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
  while (!Serial && millis() < 4000);
  Serial.println("✅ Flight firmware started");

  Wire.begin();
  Wire.setClock(400000);
  SPI.begin();

  // LAST KALIBRERINGSVERDIER
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

  // SENSORINIT
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

  bool bmp_ok = bmp.begin_I2C();
  if (bmp_ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    delay(300);

    // Sett baro-nullpunkt automatisk
    if (bmp.performReading()) {
      // Vent for å la sensoren stabilisere seg
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

      Serial.printf("Baro baseline satt: %.2f Pa (0 m)\n", bmp_p0_Pa);
    }
  }

  // STARTVARIABLER
  stateStart = millis();
  lastMicros = micros();
  alt_baro_prev = 0;
  alt_baro_filt = 0;
  vz_baro = 0;
  vz_acc  = 0;
  vz_filt = 0;
}

void loop() {
  // Les sensorer
  sensors_event_t adxlEv;
  adxl.getEvent(&adxlEv);  // m/s^2

  sensors_event_t icm_accelEv, icm_gyroEv, icm_tempEv;
  icm.getEvent(&icm_accelEv, &icm_gyroEv, &icm_tempEv);

  bool bmp_ok = bmp.performReading();   // oppdater BMP målinger

  // dt
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // Gyro (rad/s) -> fjern offset -> deg/s
  float gx = (icm_gyroEv.gyro.x - gyro_offset_x) * 57.2957795f;
  float gy = (icm_gyroEv.gyro.y - gyro_offset_y) * 57.2957795f;
  float gz = (icm_gyroEv.gyro.z - gyro_offset_z) * 57.2957795f;

  // Roll/Pitch via komplementært filter (ICM-accel + gyro)
  float rollAcc = rad2deg(atan2f(icm_accelEv.acceleration.y, icm_accelEv.acceleration.z));
  float pitchAcc = rad2deg(atan2f(-icm_accelEv.acceleration.x,
                         sqrtf(icm_accelEv.acceleration.y*icm_accelEv.acceleration.y +
                               icm_accelEv.acceleration.z*icm_accelEv.acceleration.z)));

  roll  = alpha_rp * (roll  + gx * dt) + (1.0f - alpha_rp) * rollAcc;
  pitch = alpha_rp * (pitch + gy * dt) + (1.0f - alpha_rp) * pitchAcc;

  // Yaw: gyro-integrasjon + magnetometer-korreksjon
  float yaw_gyro = wrap180(yaw + gz * dt);

  float yawMagDeg;
  bool haveMag = computeMagYawDeg(roll, pitch, yawMagDeg);
  if (haveMag) {
    // adaptiv vekt (0..1)
    float trust = magTrust(yawMagDeg, yaw_gyro, dt);
    // kraftig gyro-vekting for snappy respons; la magnetometer trekke sakte inn
    const float beta = 0.02f * trust; // 0.0–0.02 → ~1–2% mag-vekting pr iterasjon
    yaw = wrap180((1.0f - beta) * yaw_gyro + beta * yawMagDeg);
  } else {
    yaw = yaw_gyro;
  }

  // ADXL375 akselerasjoner til CSV (i g)
  // ADXL offsets er i m/s^2 slik at vi matcher getEvent()
  float ax_g = (adxlEv.acceleration.x - adxl_xOff) / g0;
  float ay_g = (adxlEv.acceleration.y - adxl_yOff) / g0;
  float az_g = (adxlEv.acceleration.z - adxl_zOff) / g0;

  // Temperatur/Trykk/Høyde
  float temperatureC = bmp_ok ? (bmp.temperature + bmp_tOff) : temperatureAtHeight(alt_sim);
  float pressurePa   = bmp_ok ? (bmp.pressure + bmp_pOff)   : pressureAtHeight(alt_sim);
  float altitude_m   = 0.0f;

  if (bmp_ok && bmp_p0_Pa > 10000.0f) {
    // standard barometrisk formel
    altitude_m = 44330.0f * (1.0f - powf(pressurePa / bmp_p0_Pa, 0.1903f));
  } else {
    altitude_m = (float)alt_sim; // fallback til simulert alt
  }

// NØYAKTIG HASTIGHETSBEREGNING (baro + accel)

// Filtrer akselerasjon (fjern støy og gravitasjon) og integrer
static float az_filt = 0;
az_filt = 0.3f * ((icm_accelEv.acceleration.z) - g0) + 0.7f * az_filt;
vz_acc += az_filt * dt;

// Baro-basert vertikalhastighet (derivert høyde)
if (bmp_ok) {
  float alt_lp_in = altitude_m;
  if (alt_baro_filt == 0.0f) alt_baro_filt = alt_lp_in;  // init første måling
  alt_baro_filt = 0.15f * alt_lp_in + 0.85f * alt_baro_filt;

  float raw_vz_baro = (alt_baro_filt - alt_baro_prev) / max(dt, 1e-3f);
  alt_baro_prev = alt_baro_filt;

  // Lavpass for å fjerne vibrasjoner
  vz_baro = 0.25f * raw_vz_baro + 0.75f * vz_baro;
}

// Fusjonér baro og accel til endelig estimat
float vz_filt = 0.75f * vz_baro + 0.25f * vz_acc;

  // Flight-time (sekunder)
  float tSec = (state >= LIFT_OFF) ? (millis() - LIFT_OFFStart) / 1000.0f : 0.0f;

  //       TILSTANDSMASKIN
  switch (state) {
    case SYSTEM_CHECK:
      if (millis() - stateStart > 10000) {
        state = OPERATION_READY;
        stateStart = millis();
      }
      break;

    case OPERATION_READY:
    // MÅ NOK TWEAKES
      // Enkel launch-deteksjon: høy Z-acc (ADXL) eller BMP-stigning
      if (az_g >= 1.60f || (bmp_ok && altitude_m > 3.0f)) {
        state = LIFT_OFF;
        LIFT_OFFStart = millis();
        stateStart = millis();
      }
      break;

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

    static unsigned long landingCandidateSince = 0;
    
    case PARACHUTE_DEPLOY:
      if (!parachuteFired) {
        parachuteFired = true;
        // TODO: fyr pyro / aktiver servo / release mekanisme
        // digitalWrite(PYRO_PIN, HIGH); delay(150); digitalWrite(PYRO_PIN, LOW);
        Serial.println("PARACHUTE: fired");
      }
      break;

      vz_acc = 0.98f * vz_acc + 0.02f * vz_baro;

      const bool lowV = fabsf(vz_filt) < 1.0f;                 // m/s
      const bool stableAlt = fabsf(altitude_m - alt_baro_prev) < 0.3f; // m pr loop-ish (du kan gjøre bedre)
      
      if (lowV && stableAlt) {
        if (landingCandidateSince == 0) landingCandidateSince = millis();
        if (millis() - landingCandidateSince > 5000) {
          // TODO: gå til LANDED state hvis du legger til en ny state
          Serial.println("LANDED detected");
        }
      } else {
        landingCandidateSince = 0;
      }
      break;
  }

  // JSON ved tilstandsendring
  if (state != lastState) {
    Serial.print('{'); Serial.print("\"state\":"); Serial.print((int)state); Serial.println('}');
    Serial.flush();
    lastState = state;
  }

  // CSV
  float vel_out   = vz_filt;                // m/s (fusjonert baro + accel)
  float press_out = pressurePa / 101325.0f; // atm (som dashboard forventer)
  long  alt_out   = (long)altitude_m;       // meter fra BMP

  // Fiktiv posisjon – beholdt fra original
  long lat = 6039290 + (long)(seq * 2);
  long lon = 532410 + (long)(seq * 2);

  emitCsv(tSec, ax_g, ay_g, az_g,
          pitch, roll, yaw,
          temperatureC, vel_out, press_out,
          lat, lon, alt_out, (int)state);

  // Oppdateringsfrekvens pr state
  switch (state) {
    case SYSTEM_CHECK:      delay(250); break;
    case OPERATION_READY:   delay(100); break;
    case LIFT_OFF:          delay(10);  break; // raskere i flight for bedre respons
    case APOGEE:            delay(50);  break;
    case PARACHUTE_DEPLOY:  delay(50);  break;
  }
}
