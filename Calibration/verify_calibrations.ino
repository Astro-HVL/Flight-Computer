#include <Arduino.h>
#include <EEPROM.h>

#include "calibration_data.h"
#include "motioncal_eeprom.h"

CalibrationData CAL;
MotionCalData   MCAL;

static void printCAL(const CalibrationData& c) {
  Serial.println("\n=== CAL (CalibrationData) @ EEPROM addr 0 ===");
  Serial.print("magic: 0x");   Serial.println(c.magic, HEX);
  Serial.print("version: 0x"); Serial.println(c.version, HEX);

  Serial.print("gyroBias: ");
  Serial.print(c.gyroBias[0], 6); Serial.print(", ");
  Serial.print(c.gyroBias[1], 6); Serial.print(", ");
  Serial.println(c.gyroBias[2], 6);

  Serial.print("adxlBias: ");
  Serial.print(c.adxlBias[0], 6); Serial.print(", ");
  Serial.print(c.adxlBias[1], 6); Serial.print(", ");
  Serial.println(c.adxlBias[2], 6);

  Serial.print("crc32 (not validated in your CAL code): 0x");
  Serial.println(c.crc32, HEX);
}

static void printMCAL(const MotionCalData& m) {
  Serial.println("\n=== MCAL (MotionCalData) @ EEPROM addr 128 ===");
  Serial.print("magic: 0x");   Serial.println(m.magic, HEX);
  Serial.print("version: 0x"); Serial.println(m.version, HEX);

  Serial.print("CRC valid: ");
  Serial.println(mcal_isValid(m) ? "YES" : "NO");

  Serial.print("magHardIron: ");
  Serial.print(m.magHardIron[0], 3); Serial.print(", ");
  Serial.print(m.magHardIron[1], 3); Serial.print(", ");
  Serial.println(m.magHardIron[2], 3);

  Serial.print("magField: ");
  Serial.println(m.magField, 3);

  Serial.println("magSoftIron 3x3 (row-major):");
  for (int r = 0; r < 3; r++) {
    Serial.print("  ");
    for (int c = 0; c < 3; c++) {
      Serial.print(m.magSoftIron[r*3 + c], 5);
      if (c < 2) Serial.print(", ");
    }
    Serial.println();
  }

  Serial.print("crc32 stored: 0x");
  Serial.println(m.crc32, HEX);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Serial.println("\n--- EEPROM VERIFY ---");

  // Les direkte for å se "rått" innhold
  EEPROM.get(CAL_EEPROM_ADDR, CAL);
  EEPROM.get(MCAL_EEPROM_ADDR, MCAL);

  // Print
  printCAL(CAL);
  printMCAL(MCAL);

  Serial.println("\nDone.");
}

void loop() {}
