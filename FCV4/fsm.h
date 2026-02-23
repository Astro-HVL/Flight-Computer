#pragma once
#include <math.h>
#include "globals.h"

// ==========================================================
//                 TUNING: APOGEE DETECT
// ==========================================================
// Sikringer:
static const unsigned long APOGEE_MIN_TIME_MS   = 5000;   // 4 km rakett: ikke apogee før 5s etter liftoff
static const float         APOGEE_MIN_ALT_M     = 100.0f; // må være >100 m før apogee kan trigges

// Toppunkt-detektering:
static const float         APOGEE_DROP_M        = 10.0f;  // må falle minst 10 m fra maks
static const float         APOGEE_VNEG_MPS      = 2.0f;   // v_est < -2.0 m/s
static const unsigned long APOGEE_VNEG_MS       = 400;    // må være negativ i 400 ms

// Failsafe: hvis alt annet feiler (tilpass rakettprofil)
static const unsigned long APOGEE_MAX_TIME_MS   = 25000;  // 25s etter liftoff: gå videre uansett (robust)

// ==========================================================
//                 TUNING: DROGUE / MAIN
// ==========================================================
// Drogue: typisk kort hold etter apogee før pyro
static const unsigned long DROGUE_HOLD_MS       = 600;    // 0.6s etter apogee-detektering
static const float         DROGUE_MIN_ALT_M     = 200.0f; // ekstra sperre (for 4km kan dette være 0–200)

// Main: deploy-høyde AGL (bruk KF z_est).
static const float         MAIN_DEPLOY_ALT_M    = 600.0f; 

// Stabilitet: må være under terskel en liten stund før main
static const unsigned long MAIN_HOLD_MS         = 300;    // 300 ms under terskel

// Failsafe for main: hvis du aldri kommer under terskel (sensorfeil/estimatfeil)
// (Typisk: 60–120s etter drogue, avhengig av drogue synk)
static const unsigned long MAIN_FAILSAFE_MS     = 90000;  // 90s etter drogue-descent start

// ==========================================================
//                 Helper: safe comparisons
// ==========================================================
static inline bool isFinitef(float x) { return !(isnan(x) || isinf(x)); }

// ==========================================================
//                 FSM UPDATE
// ==========================================================
static inline void fsmUpdate(bool bmp_ok, float altitude_m, float a_net_filt)
{
  (void)bmp_ok;      // vi bruker primært z_est/v_est, men lar param stå (ryddig API)
  (void)altitude_m;  // kan brukes senere for ekstra sanity-check hvis du vil

  switch (state) {

    // ---------------------------
    // SYSTEM_CHECK -> READY
    // ---------------------------
    case SYSTEM_CHECK:
      if (millis() - stateStart > 10000) {
        state = OPERATION_READY;
        stateStart = millis();
      }
      break;

    // ---------------------------
    // READY: launch detect + arming
    // ---------------------------
    case OPERATION_READY: {
      static int normCnt = 0;
      static int hardCnt = 0;
      static unsigned long quietTimer = 0;

      // “Quiet” måles på v_est + a_net_filt
      if (fabsf(v_est) > 0.3f || a_net_filt > 0.8f) {
        quietTimer = millis();
      }

      const bool stableBefore = (millis() - quietTimer > ARM_QUIET_MS);
      const bool armed = stableBefore || (millis() - stateStart > ARM_TIMEOUT_MS);

      // launch counters
      if (a_net_filt > LAUNCH_A_NORM) normCnt++; else normCnt = 0;
      if (a_net_filt > LAUNCH_A_HARD) hardCnt++; else hardCnt = 0;

      const bool liftoff = (armed && normCnt >= LAUNCH_N_NORM) || (hardCnt >= LAUNCH_N_HARD);

      if (liftoff) {
        state = LIFT_OFF;
        LIFT_OFFStart = millis();
        stateStart = millis();

        // flight-mode
        if (bmp_present) {
          bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
          sigma_baro = 1.2f;
          R_baro = sigma_baro * sigma_baro;
        }

        // reset
        normCnt = 0;
        hardCnt = 0;

        // reset deploy flags for ny flight
        drogueFired = false;
        mainFired   = false;
      }

      lift_off_armed = armed ? 1 : 0;
      break;
    }

    // ---------------------------
    // LIFT_OFF: robust apogee detection
    // ---------------------------
    case LIFT_OFF: {
      static float maxZ = -1e9f;
      static unsigned long negVStart = 0;

      const unsigned long tSinceLift = millis() - LIFT_OFFStart;

      // init ved entry
      if (maxZ < -1e8f) {
        maxZ = z_est;
        negVStart = 0;
      }

      // Oppdater peak på z_est (KF-estimat)
      if (z_est > maxZ) maxZ = z_est;

      // Sikringer
      const bool timeOk = (tSinceLift >= APOGEE_MIN_TIME_MS);
      const bool altOk  = (maxZ >= APOGEE_MIN_ALT_M);

      // Falt fra peak?
      const bool droppedFromPeak = (z_est <= (maxZ - APOGEE_DROP_M));

      // Negativ v over tid
      if (v_est < -APOGEE_VNEG_MPS) {
        if (negVStart == 0) negVStart = millis();
      } else {
        negVStart = 0;
      }
      const bool vNegLongEnough = (negVStart != 0) && (millis() - negVStart >= APOGEE_VNEG_MS);

      const bool apogeeDetected = timeOk && altOk && droppedFromPeak && vNegLongEnough;

      // Timeout (ikke bindet til altOk, ellers kan du bli stuck hvis maxZ aldri blir “stor nok” pga feil)
      const bool apogeeTimeout  = (tSinceLift >= APOGEE_MAX_TIME_MS) && timeOk;

      if (apogeeDetected || apogeeTimeout) {
        state = APOGEE;
        stateStart = millis();

        // reset trackers
        maxZ = -1e9f;
        negVStart = 0;
      }
      break;
    }

    // ---------------------------
    // APOGEE: short hold, then drogue deploy
    // ---------------------------
    case APOGEE: {
      const unsigned long tInApogee = millis() - stateStart;

      // Bruk z_est (robust). Hvis den blir NaN av en eller annen grunn, ikke blokker alt.
      const float alt = z_est;
      const bool altOk = !isFinitef(alt) ? true : (alt >= DROGUE_MIN_ALT_M);

      if (tInApogee >= DROGUE_HOLD_MS && altOk) {
        state = DROGUE_DEPLOY;
        stateStart = millis();
      }
      break;
    }

    // ---------------------------
    // DROGUE_DEPLOY: fire drogue once
    // ---------------------------
    case DROGUE_DEPLOY:
      if (!drogueFired) {
        drogueFired = true;
        Serial.println("DROGUE: fired");
        // TODO: Sett din pyro-pin HIGH her (drogue), evt. med pulse/timer
      }
      // Gå videre direkte
      state = DROGUE_DESCENT;
      stateStart = millis();
      break;

    // ---------------------------
    // DROGUE_DESCENT: wait until main altitude, then main deploy
    // ---------------------------
    case DROGUE_DESCENT: {
      static unsigned long belowMainStart = 0;
      const unsigned long tSinceDrogue = millis() - stateStart;

      const float alt = z_est;

      // Hvis z_est er NaN/inf: bruk kun tid-failsafe
      const bool haveAlt = isFinitef(alt);

      if (haveAlt) {
        const bool belowMain = (alt <= MAIN_DEPLOY_ALT_M);

        if (belowMain) {
          if (belowMainStart == 0) belowMainStart = millis();
        } else {
          belowMainStart = 0;
        }

        const bool belowLongEnough = (belowMainStart != 0) && (millis() - belowMainStart >= MAIN_HOLD_MS);

        if (belowLongEnough) {
          state = MAIN_DEPLOY;
          stateStart = millis();
          belowMainStart = 0;
          break;
        }
      }

      // Failsafe: deploy main uansett etter en stund på drogue
      if (tSinceDrogue >= MAIN_FAILSAFE_MS) {
        state = MAIN_DEPLOY;
        stateStart = millis();
        belowMainStart = 0;
      }
      break;
    }

    // ---------------------------
    // MAIN_DEPLOY: fire main once
    // ---------------------------
    case MAIN_DEPLOY:
      if (!mainFired) {
        mainFired = true;
        Serial.println("MAIN: fired");
        // TODO: Sett din pyro-pin HIGH her (main), evt. med pulse/timer
      }
      state = MAIN_DESCENT;
      stateStart = millis();
      break;

    // ---------------------------
    // MAIN_DESCENT: end state
    // ---------------------------
    case MAIN_DESCENT:
      // Her kan du evt. legge inn “landing detect” og logging.
      break;

    default:
      // Hvis state blir korrupt, gå til safe:
      state = MAIN_DESCENT;
      stateStart = millis();
      break;
  }

  // For logging: etter liftoff regnes den som “armed”
  if (state >= LIFT_OFF) lift_off_armed = 1;
}
