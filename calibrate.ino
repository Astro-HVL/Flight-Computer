#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_LIS3MDL.h>
#include "calibration_data.h"

// ---------- Teensy 4.1 SPI pins ----------
#define SPI_SCK   13
#define SPI_MISO  12
#define SPI_MOSI  11
#define CS_ADXL   10
#define CS_ICM    15

Adafruit_ADXL375   adxl(CS_ADXL, &SPI, 12345);
Adafruit_ICM20649  icm;
Adafruit_LIS3MDL   mag;

CalibrationData CAL;

void waitMsg(const __FlashStringHelper* s) {
  Serial.println(s);
  while (!Serial.available()) { delay(10); }
  while (Serial.available()) (void)Serial.read();
}

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println(F("\n=== Calibration Utility (Teensy 4.1) ==="));

  Wire.begin();
  SPI.begin();

  if (!adxl.begin())   Serial.println(F("❌ ADXL375 not found"));
  if (!icm.begin_SPI(CS_ICM)) Serial.println(F("❌ ICM20649 not found"));
  if (!mag.begin_I2C()) Serial.println(F("❌ LIS3MDL not found"));

  // ---------- Gyro warm-up phase ----------
  Serial.println(F("\n⚙️  Oppvarming av gyro (60 sekunder for stabil temperatur)..."));
  float lastTemp = 0;
  unsigned long tStart = millis();

  while (millis() - tStart < 60000) {
    sensors_event_t a, g, t;
    icm.getEvent(&a, &g, &t);
    if (millis() % 5000 < 20) {  // skriv ut hvert 5. sekund
      Serial.printf("Tid: %lus  |  GyroTemp: %.2f °C\n", (millis() - tStart) / 1000, t.temperature);
    }
    delay(50);
  }

  Serial.println(F("✅ Gyro oppvarmet – klar for kalibrering.\n"));

  cal_load(CAL);

  Serial.println(F("\nTrykk ENTER for å starte kalibrering (enhet helt stille på plan flate)."));
  waitMsg(F(">>>"));

  // ---------- 1) Gyro-bias (ICM20649) ----------
  {
    const int N = 800;
    float gx = 0, gy = 0, gz = 0;
    Serial.println(F("Kalibrerer gyro... ikke rør enheten (ca 4 s)"));
    for (int i = 0; i < N; i++) {
      sensors_event_t a, g, t;
      icm.getEvent(&a, &g, &t);
      gx += g.gyro.x; gy += g.gyro.y; gz += g.gyro.z;
      delay(5);
    }
    CAL.gyroBias[0] = gx / N;
    CAL.gyroBias[1] = gy / N;
    CAL.gyroBias[2] = gz / N;
    Serial.printf("Gyro bias  [rad/s]:  %+0.6f  %+0.6f  %+0.6f\n",
                  CAL.gyroBias[0], CAL.gyroBias[1], CAL.gyroBias[2]);
  }

  // ---------- 2) Accel-bias (ADXL375) ----------
  {
    const int N = 800;
    float ax = 0, ay = 0, az = 0;
    Serial.println(F("Kalibrerer akselerometer (ADXL375)... enhet stille, Z opp (~+1g)"));
    for (int i = 0; i < N; i++) {
      sensors_event_t e; adxl.getEvent(&e);
      ax += e.acceleration.x / 9.80665f; // m/s² → g
      ay += e.acceleration.y / 9.80665f;
      az += e.acceleration.z / 9.80665f;
      delay(5);
    }
    ax /= N; ay /= N; az /= N;
    CAL.adxlBias[0] = ax - 0.0f;
    CAL.adxlBias[1] = ay - 0.0f;
    CAL.adxlBias[2] = az - 1.0f;
    Serial.printf("ADXL bias    [g]:    %+0.5f  %+0.5f  %+0.5f\n",
                  CAL.adxlBias[0], CAL.adxlBias[1], CAL.adxlBias[2]);
  }

  // ---------- 3) Magnetometer (LIS3MDL) ----------
  {
    Serial.println(F("\nMag-kalibrering (hard-iron + enkel scale). Beveg i 3D/‘figure-8’ i 60 s..."));
    float minX = 1e9, minY = 1e9, minZ = 1e9;
    float maxX = -1e9, maxY = -1e9, maxZ = -1e9;
    uint32_t t0 = millis();
    while (millis() - t0 < 60000) {
      sensors_event_t m; mag.getEvent(&m);
      minX = min(minX, m.magnetic.x); maxX = max(maxX, m.magnetic.x);
      minY = min(minY, m.magnetic.y); maxY = max(maxY, m.magnetic.y);
      minZ = min(minZ, m.magnetic.z); maxZ = max(maxZ, m.magnetic.z);
      delay(10);
    }
    CAL.magOffset[0] = (maxX + minX) * 0.5f;
    CAL.magOffset[1] = (maxY + minY) * 0.5f;
    CAL.magOffset[2] = (maxZ + minZ) * 0.5f;

    const float rx = (maxX - minX) * 0.5f;
    const float ry = (maxY - minY) * 0.5f;
    const float rz = (maxZ - minZ) * 0.5f;
    CAL.magScale[0] = (rx <= 0 ? 1.f : rx);
    CAL.magScale[1] = (ry <= 0 ? 1.f : ry);
    CAL.magScale[2] = (rz <= 0 ? 1.f : rz);

    Serial.printf("Mag offset   [uT]:   %+0.2f  %+0.2f  %+0.2f\n",
                  CAL.magOffset[0], CAL.magOffset[1], CAL.magOffset[2]);
    Serial.printf("Mag scale    [uT]:    %0.2f   %0.2f   %0.2f\n",
                  CAL.magScale[0], CAL.magScale[1], CAL.magScale[2]);
  }

  // ---------- Lagre ----------
  cal_save(CAL);
  Serial.println(F("\n✅ Kalibrering lagret til EEPROM.\nStart flight-firmware for normal drift."));
}

void loop() { /* tom */ }
