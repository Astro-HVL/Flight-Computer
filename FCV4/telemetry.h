#pragma once
#include <Arduino.h>
#include "globals.h"

static inline void emitCsv(float tSec, float ax_g, float ay_g, float az_g,
                           float pitch_, float roll_, float yaw_,
                           float temperature, float vel, float press,
                           int drogueFired_, int mainFired_, long alt_m, int state_)
{
  Serial.print(tSec, 1);     Serial.print(',');
  Serial.print(seq++);       Serial.print(',');
  Serial.print(ax_g);        Serial.print(',');
  Serial.print(ay_g);        Serial.print(',');
  Serial.print(az_g, 1);     Serial.print(',');
  Serial.print(pitch_, 1);   Serial.print(',');
  Serial.print(roll_, 1);    Serial.print(',');
  Serial.print(yaw_, 1);     Serial.print(',');
  Serial.print(temperature); Serial.print(',');
  Serial.print(vel);         Serial.print(',');
  Serial.print(press);       Serial.print(',');
  Serial.print(drogueFired_); Serial.print(',');
  Serial.print(mainFired_);   Serial.print(',');
  Serial.print(alt_m);       Serial.print(',');
  Serial.println(state_);
}

inline void delayByState() {
  switch (state) {
    case SYSTEM_CHECK:      delay(100); break;
    case OPERATION_READY:   delay(50);  break;
    case LIFT_OFF:          delay(5);   break;

    // Etter liftoff trenger du fortsatt høy rate, men ikke ekstremt
    case APOGEE:            delay(10);  break;

    // Drogue/main-states
    case DROGUE_DEPLOY:     delay(10);  break;
    case DROGUE_DESCENT:    delay(20);  break;  // litt roligere logging
    case MAIN_DEPLOY:       delay(10);  break;
    case MAIN_DESCENT:      delay(20);  break;

    default:                delay(20);  break;
  }
}