#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#define MCAL_MAGIC   0x4D43414Cu
#define MCAL_VERSION 0x0001u

constexpr size_t MCAL_EEPROM_ADDR = 128;  // IKKE 0

struct MotionCalData {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;

  float magHardIron[3];
  float magSoftIron[9];
  float magField;

  uint32_t crc32;
};

static inline uint32_t mcal_crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++)
    crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
  return crc;
}

static inline uint32_t mcal_crc32_bytes(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++)
    crc = mcal_crc32_update(crc, data[i]);
  return ~crc;
}

static inline void mcal_setDefaults(MotionCalData& m) {
  memset(&m, 0, sizeof(m));
  m.magic = MCAL_MAGIC;
  m.version = MCAL_VERSION;
  m.magSoftIron[0] = m.magSoftIron[4] = m.magSoftIron[8] = 1.0f;
}

static inline bool mcal_isValid(const MotionCalData& m) {
  if (m.magic != MCAL_MAGIC || m.version != MCAL_VERSION) return false;
  MotionCalData tmp = m;
  uint32_t stored = tmp.crc32;
  tmp.crc32 = 0;
  return stored == mcal_crc32_bytes((uint8_t*)&tmp, sizeof(tmp));
}

static inline void mcal_load(MotionCalData& m) {
  EEPROM.get(MCAL_EEPROM_ADDR, m);
  if (!mcal_isValid(m)) mcal_setDefaults(m);
}

static inline void mcal_save(MotionCalData& m) {
  m.magic = MCAL_MAGIC;
  m.version = MCAL_VERSION;
  MotionCalData tmp = m;
  tmp.crc32 = 0;
  m.crc32 = mcal_crc32_bytes((uint8_t*)&tmp, sizeof(tmp));
  EEPROM.put(MCAL_EEPROM_ADDR, m);
}

static inline void mcal_apply(const MotionCalData& m,
                              float mx, float my, float mz,
                              float& mx_out, float& my_out, float& mz_out)
{
  float x = mx - m.magHardIron[0];
  float y = my - m.magHardIron[1];
  float z = mz - m.magHardIron[2];

  mx_out = m.magSoftIron[0]*x + m.magSoftIron[1]*y + m.magSoftIron[2]*z;
  my_out = m.magSoftIron[3]*x + m.magSoftIron[4]*y + m.magSoftIron[5]*z;
  mz_out = m.magSoftIron[6]*x + m.magSoftIron[7]*y + m.magSoftIron[8]*z;
}
