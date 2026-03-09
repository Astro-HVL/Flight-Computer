#include "globals.h"
#include <SPI.h>

// PINS
const int SPI_SCK  =              13;
const int SPI_MISO =              12;
const int SPI_MOSI =              11;
const int CS_ADXL  =              10;
const int CS_ICM   =              15;

// SENSORS (OBS: ADXL bruker SPI objekt)
Adafruit_ADXL375                  adxl(CS_ADXL, &SPI, 12345);
Adafruit_ICM20649                 icm;
Adafruit_LIS3MDL                  lis3mdl;
Adafruit_BMP3XX                   bmp;

// CALIBRATION
CalibrationData CAL;
MotionCalData MCAL;

float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float adxl_xOff = 0, adxl_yOff = 0, adxl_zOff = 0;
float mag_xBias = 0, mag_yBias = 0, mag_zBias = 0;
float mag_xScale = 1.0f, mag_yScale = 1.0f, mag_zScale = 1.0f;
float decl_deg = 0.0f;
float bmp_p0_Pa = 101325.0f;
bool  bmp_present = false;

// ATTITUDE
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float yaw_offset = 0.0f;
float alpha_rp = 0.95f; // Gjøres adaptiv? Høy alpha under flight, lav på bakken. 0.95 = 95% gyro, 5% accel
unsigned long lastMicros = 0;

// MISC
unsigned long seq = 0;
float  g0 = 9.80665f;
unsigned long LIFT_OFFStart = 0;

int lift_off_armed = 0;

// LAUNCH DETECT TUNING
const float LAUNCH_A_NORM  = 2.5f;
const int   LAUNCH_N_NORM  = 10;

const float LAUNCH_A_HARD  = 7.0f;
const int   LAUNCH_N_HARD  = 4;

const unsigned long ARM_QUIET_MS    = 500;
const unsigned long ARM_TIMEOUT_MS  = 45000;

// PARACHUTE
bool drogueFired = false;
bool mainFired   = false;


// FSM
State state = SYSTEM_CHECK;
State lastState = SYSTEM_CHECK;
unsigned long stateStart = 0;

// KALMAN VZ
float z_est = 0.0f, v_est = 0.0f;
float P00 = 1, P01 = 0, P10 = 0, P11 = 1;

float sigma_acc = 0.4f;
float sigma_model_v = 0.15f;
float sigma_baro = 1.5f;
float R_baro = 1.5f * 1.5f;
