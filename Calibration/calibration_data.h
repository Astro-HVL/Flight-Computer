#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#define CAL_MAGIC   0x42A1C0DEu
#define CAL_VERSION 0x0002u

struct CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  float gyroBias[3];
  float adxlBias[3];
  uint32_t crc32;
};

constexpr size_t CAL_EEPROM_ADDR = 0;

inline bool cal_isValid(const CalibrationData& c) {
  return (c.magic == CAL_MAGIC) && (c.version == CAL_VERSION);
}

inline void cal_setDefaults(CalibrationData& c) {
  memset(&c, 0, sizeof(c));
  c.magic   = CAL_MAGIC;
  c.version = CAL_VERSION;
}

inline void cal_load(CalibrationData& c) {
  EEPROM.get(CAL_EEPROM_ADDR, c);
  if (!cal_isValid(c)) cal_setDefaults(c);
}

inline void cal_save(const CalibrationData& c) {
  EEPROM.put(CAL_EEPROM_ADDR, c);
  // Teensy har ekte EEPROM – commit() trengs ikke
}
