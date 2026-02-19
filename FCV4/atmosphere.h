#pragma once
#include <math.h>

static inline float pressureAtHeight(float h) {
  if (h < 11000) return 101325 * pow(1 - 0.0065 * h / 288.15, 5.2561);
  else if (h < 20000) return 22632 * exp(-0.000157 * (h - 11000));
  else if (h < 32000) return 5474 * pow(1 + 0.001 * (h - 20000) / 216.65, -34.1632);
  else if (h < 47000) return 868 * pow(1 - 0.0028 * (h - 32000) / 228.65, 12.2016);
  else if (h < 51000) return 110 * exp(-0.000157 * (h - 47000));
  else if (h < 71000) return 66 * pow(1 - 0.0028 * (h - 51000) / 270.65, -12.2016);
  else return 0.12;
}

static inline float temperatureAtHeight(float h) {
  if (h < 11000) return 15 - 0.0065 * h;
  else if (h < 20000) return -56.5;
  else if (h < 32000) return -56.5 + 0.001 * (h - 20000);
  else if (h < 47000) return -44.5 + 0.0028 * (h - 32000);
  else if (h < 51000) return -2.5;
  else if (h < 71000) return -2.5 - 0.0028 * (h - 51000);
  else return -58.5;
}