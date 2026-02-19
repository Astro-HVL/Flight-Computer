#pragma once
#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BMP3XX.h>

#include "calibration_data.h"



// ------------------- Hardware pins -------------------
extern const int           SPI_SCK;
extern const int           SPI_MISO;
extern const int           SPI_MOSI;
extern const int           CS_ADXL;
extern const int           CS_ICM;



// ------------------- Sensors -------------------
extern Adafruit_ADXL375    adxl;
extern Adafruit_ICM20649   icm;
extern Adafruit_LIS3MDL    lis3mdl;
extern Adafruit_BMP3XX     bmp;



// ------------------- Calibration -------------------
extern CalibrationData     CAL;

extern float               gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float               adxl_xOff, adxl_yOff, adxl_zOff;
extern float               mag_xBias, mag_yBias, mag_zBias;
extern float               mag_xScale, mag_yScale, mag_zScale;
extern float               decl_deg;
extern float               bmp_p0_Pa;
extern float               bmp_tOff;
extern float               bmp_pOff;
extern bool                bmp_present;


// ------------------- Attitude -------------------
extern float               roll, pitch, yaw;     // deg
extern float               yaw_offset;
extern float               alpha_rp;
extern unsigned long       lastMicros;



// ------------------- Misc -------------------
extern unsigned long       seq;
extern float               g0;
extern unsigned long       LIFT_OFFStart;

extern int                 lift_off_armed;
extern bool                drogueFired;
extern bool                mainFired;




// ------------------- Launch detect tuning -------------------
extern const float         LAUNCH_A_NORM;
extern const int           LAUNCH_N_NORM;

extern const float         LAUNCH_A_HARD;
extern const int           LAUNCH_N_HARD;

extern const unsigned long ARM_QUIET_MS;
extern const unsigned long ARM_TIMEOUT_MS;



// ------------------- Kalman VZ -------------------
extern float               z_est, v_est;
extern float               P00, P01, P10, P11;

extern float               sigma_acc;
extern float               sigma_model_v;
extern float               sigma_baro;
extern float               R_baro;



// ------------------- FSM -------------------
enum State {
  SYSTEM_CHECK = 1,
  OPERATION_READY,
  LIFT_OFF,
  APOGEE,
  DROGUE_DEPLOY,
  DROGUE_DESCENT,
  MAIN_DEPLOY,
  MAIN_DESCENT
};
extern                     State state;
extern                     State lastState;
extern unsigned long       stateStart;