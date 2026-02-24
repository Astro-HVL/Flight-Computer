#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_LIS3MDL.h>

#include "calibration_data.h"
#include "motioncal_eeprom.h"

// ------------------ Sensorer ------------------
Adafruit_ICM20649 icm;
Adafruit_LIS3MDL  lis3mdl;

// ------------------ EEPROM ------------------
CalibrationData CAL;
MotionCalData   MCAL;

// ------------------ MotionCal RX ------------------
byte caldata[68];
byte calcount = 0;

static inline uint16_t crc16_update(uint16_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) {
    if (crc & 1) crc = (crc >> 1) ^ 0xA001;
    else         crc = (crc >> 1);
  }
  return crc;
}

void receiveCalibration() {
  uint16_t crc;
  byte b;

  while (Serial.available()) {
    b = Serial.read();

    if (calcount == 0 && b != 117) return;
    if (calcount == 1 && b != 84)  { calcount = 0; return; }

    caldata[calcount++] = b;
    if (calcount < 68) return;

    crc = 0xFFFF;
    for (int i = 0; i < 68; i++)
      crc = crc16_update(crc, caldata[i]);

    if (crc == 0) {
      float offsets[16];
      memcpy(offsets, caldata + 2, 16 * 4);

      // -------- HARD IRON --------
      MCAL.magHardIron[0] = offsets[6];
      MCAL.magHardIron[1] = offsets[7];
      MCAL.magHardIron[2] = offsets[8];

      // -------- FIELD --------
      MCAL.magField = offsets[9];

      // -------- SOFT IRON (Adafruit symmetrisk mapping) --------
      MCAL.magSoftIron[0] = offsets[10];
      MCAL.magSoftIron[4] = offsets[11];
      MCAL.magSoftIron[8] = offsets[12];

      MCAL.magSoftIron[1] = MCAL.magSoftIron[3] = offsets[13];
      MCAL.magSoftIron[2] = MCAL.magSoftIron[6] = offsets[14];
      MCAL.magSoftIron[5] = MCAL.magSoftIron[7] = offsets[15];

      // -------- Lagre full MCAL --------
      mcal_save(MCAL);

      // -------- Lagre full CAL  --------
      cal_save(CAL);

      Serial.println("\n✅ MotionCal lagret!");
      Serial.print("HardIron: ");
      Serial.print(MCAL.magHardIron[0], 3); Serial.print(", ");
      Serial.print(MCAL.magHardIron[1], 3); Serial.print(", ");
      Serial.println(MCAL.magHardIron[2], 3);

      Serial.print("Field: ");
      Serial.println(MCAL.magField, 3);

      Serial.println("SoftIron 3x3:");
      for (int r = 0; r < 3; r++) {
        Serial.print("  ");
        for (int c = 0; c < 3; c++) {
          Serial.print(MCAL.magSoftIron[r*3+c], 5);
          if (c < 2) Serial.print(", ");
        }
        Serial.println();
      }

      calcount = 0;
      return;
    }

    calcount = 0;
  }
}

void setup() {
  delay(1200);
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Serial.println("\n--- MotionCal (ICM20649 SPI + LIS3MDL I2C) ---");
  Serial.println("Lukk Serial Monitor før du bruker MotionCal.\n");

  Wire.begin();
  Wire.setClock(400000);

  SPI.begin();
  const int CS_ICM = 15;

  if (!icm.begin_SPI(CS_ICM, &SPI)) {
    Serial.println("❌ ICM SPI init feilet");
    while (1);
  }

  if (!lis3mdl.begin_I2C()) {
    Serial.println("❌ LIS3MDL init feilet");
    while (1);
  }

  icm.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);

  cal_load(CAL);
  mcal_load(MCAL);

  Serial.println("EEPROM: CAL + MCAL loaded.");
}

void loop() {
  sensors_event_t acc, gyro, temp;
  sensors_event_t mag;

  icm.getEvent(&acc, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  const float g0 = 9.80665f;

  int ax = (int)lroundf(acc.acceleration.x * 8192.0f / g0);
  int ay = (int)lroundf(acc.acceleration.y * 8192.0f / g0);
  int az = (int)lroundf(acc.acceleration.z * 8192.0f / g0);

  float gx_dps = gyro.gyro.x * 57.2957795f;
  float gy_dps = gyro.gyro.y * 57.2957795f;
  float gz_dps = gyro.gyro.z * 57.2957795f;

  int gx = (int)lroundf(gx_dps * 16.0f);
  int gy = (int)lroundf(gy_dps * 16.0f);
  int gz = (int)lroundf(gz_dps * 16.0f);

  int mx = (int)lroundf(mag.magnetic.x * 10.0f);
  int my = (int)lroundf(mag.magnetic.y * 10.0f);
  int mz = (int)lroundf(mag.magnetic.z * 10.0f);

  Serial.print("Raw:");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(mx); Serial.print(",");
  Serial.print(my); Serial.print(",");
  Serial.println(mz);

  receiveCalibration();

  delay(10);
}
