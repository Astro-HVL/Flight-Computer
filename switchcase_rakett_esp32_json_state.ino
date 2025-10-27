#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ICM20649.h>
#include <EEPROM.h>

// SPI pins
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  21
#define CS_ADXL   22
#define CS_ICM    15

// Sensorer
Adafruit_ADXL375 accel = Adafruit_ADXL375(CS_ADXL, &SPI, 12345);
Adafruit_ICM20649 icm;

// Kalibreringsdata
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
const int CAL_SAMPLES = 500;
float xOffset, yOffset, zOffset;

// Tilstandsvariabler
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float alpha = 0.98f;
unsigned long lastMicros = 0;
unsigned long seq = 0;
long alt = 0;
float vel = 0;
float a_up = 10.0;
float g0 = 9.81;
unsigned long LIFT_OFFStart = 0;

// Systemtilstand
enum State {
  SYSTEM_CHECK = 1,
  OPERATION_READY,
  LIFT_OFF,
  APOGEE,
  PARACHUTE_DEPLOY
};

State state = SYSTEM_CHECK;
State lastState = SYSTEM_CHECK; // Brukes for å sende JSON ved tilstandsendring
unsigned long stateStart = 0;

void calibrateGyro() {
  sensors_event_t accelEv, gyroEv, tempEv;
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    icm.getEvent(&accelEv, &gyroEv, &tempEv);
    gx_sum += gyroEv.gyro.x;
    gy_sum += gyroEv.gyro.y;
    gz_sum += gyroEv.gyro.z;
    delay(2);
  }
  gyro_offset_x = gx_sum / CAL_SAMPLES;
  gyro_offset_y = gy_sum / CAL_SAMPLES;
  gyro_offset_z = gz_sum / CAL_SAMPLES;
}

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

// Skriver ut én CSV-linje per måling
inline void emitCsv(float tSec, float ax, float ay, float az,
                    float pitch, float roll, float yaw,
                    float temperature, float vel, float press,
                    long lat, long lon, long alt, int state)
{
  Serial.print(tSec, 3); Serial.print(',');
  Serial.print(seq++);   Serial.print(',');
  Serial.print(ax);      Serial.print(',');
  Serial.print(ay);      Serial.print(',');
  Serial.print(az);      Serial.print(',');
  Serial.print(pitch);   Serial.print(',');
  Serial.print(roll);    Serial.print(',');
  Serial.print(yaw);     Serial.print(',');
  Serial.print(temperature); Serial.print(',');
  Serial.print(vel);     Serial.print(',');
  Serial.print(press);   Serial.print(',');
  Serial.print(lat);     Serial.print(',');
  Serial.print(lon);     Serial.print(',');
  Serial.print(alt);     Serial.print(',');
  Serial.println((int)state);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // ICM20649 init
  if (!icm.begin_SPI(CS_ICM, &SPI)) {
    while (1);
  }
  icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  calibrateGyro();

  // ADXL375 init
  accel.begin();

  // Last kalibreringsdata
  EEPROM.get(0, xOffset);
  EEPROM.get(sizeof(float), yOffset);
  EEPROM.get(2 * sizeof(float), zOffset);

  stateStart = millis();
  lastMicros = micros();
}

void loop() {
  sensors_event_t adxlEvent, accelEv, gyroEv, tempEv;
  accel.getEvent(&adxlEvent);
  icm.getEvent(&accelEv, &gyroEv, &tempEv);

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // Fjern gyro-offset
  float gx = gyroEv.gyro.x - gyro_offset_x;
  float gy = gyroEv.gyro.y - gyro_offset_y;
  float gz = gyroEv.gyro.z - gyro_offset_z;
  gx *= 57.2958f; gy *= 57.2958f; gz *= 57.2958f;

  // Gyrofilter
  float rollAcc = atan2(accelEv.acceleration.y, accelEv.acceleration.z) * 57.2958f;
  float pitchAcc = atan2(-accelEv.acceleration.x,
                         sqrt(accelEv.acceleration.y * accelEv.acceleration.y +
                              accelEv.acceleration.z * accelEv.acceleration.z)) * 57.2958f;

  roll = alpha * (roll + gx * dt) + (1.0f - alpha) * rollAcc;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitchAcc;
  yaw += gz * dt;
  if (yaw > 180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;

  // Korrigert akselerometerdata
  float ax = adxlEvent.acceleration.x - xOffset;
  float ay = adxlEvent.acceleration.y - yOffset;
  float az = adxlEvent.acceleration.z - zOffset;

  // Flight-time (sekunder) – 0 før liftoff, fortsetter gjennom hele flighten
    float tSec = (state >= LIFT_OFF) 
               ? (millis() - LIFT_OFFStart) / 1000.0f 
               : 0.0f;

  // Tilstandsmaskin
  // Bare for test, tilstander må forandres og gjøres mer robuste
  switch (state) {
    case SYSTEM_CHECK:
      if (millis() - stateStart > 10000) {
        state = OPERATION_READY;
        stateStart = millis();
      }
      break;

    case OPERATION_READY:
      if (az >= 15) {
        state = LIFT_OFF;
        LIFT_OFFStart = millis();
        stateStart = millis();
      }
      break;

    case LIFT_OFF:
      vel += a_up * 0.15f;
      if (vel > 2000) vel = 2000;
      alt += 30;
      if (alt > 100000) alt = 100000;

      if (alt >= 4500) {
        state = APOGEE;
        stateStart = millis();
      }
      break;

    case APOGEE:
      // Simuler toppen av banen (stabil høyde en liten stund)
      delay(500);
      // Etter kort tid utløses fallskjerm
      if (millis() - stateStart > 3000) { // 3 sek etter apogee
        state = PARACHUTE_DEPLOY;
        stateStart = millis();
      }
      break;

    case PARACHUTE_DEPLOY:
      vel *= 0.9f;
      if (vel < 50) vel = 50;
      alt -= 50;
      if (alt < 0) alt = 0;
      break;
  }

  // Send JSON ved tilstandsendring
  if (state != lastState) {
    Serial.print('{');
    Serial.print("\"state\":");
    Serial.print((int)state);
    Serial.println('}');   // avslutt nøyaktig med }
    Serial.flush();
    lastState = state;
  }


  // Beregn og send CSV
  float press = pressureAtHeight(alt);
  float temperature = temperatureAtHeight(alt);
  long lat = 6039290 + (long)(seq * 2);
  long lon = 532410 + (long)(seq * 2);

  emitCsv(tSec, ax, ay, az, pitch, roll, yaw, temperature, vel, press, lat, lon, alt, (int)state);

  // Liten pause per state for å gi jevn oppdatering
  switch (state) {
    case SYSTEM_CHECK: delay(250); break;
    case OPERATION_READY: delay(100); break;
    case LIFT_OFF: delay(50); break;
    case PARACHUTE_DEPLOY: delay(200); break;
  }
}
