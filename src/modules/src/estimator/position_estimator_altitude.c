/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_estimator_altitude.c: Altitude-only position estimator
 *
 * Two height estimators run in parallel from the same baro and acc:
 *  - the original Bitcraze filter (IIR on the baro, posEstAlt)
 *  - a linear Kalman filter (heightKF), states z, vz, acc bias, baro bias,
 *    which can also use the GPS vertical velocity and the GPS height
 * posEstAlt.useKF selects which one goes to stateEstimate.z / vz.
 * Both outputs are relative to the baro ASL at power-on (asl0, mean of the
 * first second with baro data).
 */

#include <math.h>

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "num.h"
#include "position_estimator.h"
#include "physicalConstants.h"

#define G 9.81f;

struct selfState_s {
  float estimatedZ; // The current Z estimate, has same offset as asl
  float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlphaZrange;
  float estAlphaAsl;
  float velocityFactor;
  float vAccDeadband; // Vertical acceleration deadband
  float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
  float estimatedVZ;
  float asl0;       // ASL at start-up, mean of the first ASL0_SAMPLES calls with baro data
  uint16_t asl0Count;
  float zRel;       // estimatedZ - asl0 (m)
};

// start-up average for asl0: 100 calls at 100 Hz = 1 s (50 baro samples)
#define ASL0_SAMPLES 100

static uint8_t useKF = 1;  // 1: Kalman filter to stateEstimate, 0: Bitcraze filter

static struct selfState_s state = {
  .estimatedZ = 0.0f,
  .velocityZ = 0.0f,
  .estAlphaZrange = 0.90f,
  .estAlphaAsl = 0.90f,
  .velocityFactor = 1.0f,
  .vAccDeadband = 0.04f,
  .velZAlpha = 0.995f,
  .estimatedVZ = 0.0f,
  .asl0 = 0.0f,
  .asl0Count = 0,
  .zRel = 0.0f,
};

/* ---------------------------------------------------------------------------
 * Linear Kalman filter for the height, x = [z, vz, ba, bb]
 *   z' = vz,  vz' = a - ba + w_a,  ba' = w_b,  bb' = w_bb
 *   baro = z + bb + n
 *   GPS velD = -vz + n          (heightKF.useGps bit 0)
 *   GPS hMSL - gpsOff = z + n   (heightKF.useGps bit 1)
 * a: vertical acc without gravity (sensfusion6GetAccZWithoutGravity, m/s^2)
 * prediction at 250 Hz (positionUpdateVelocity), baro update on every new
 * baro sample (50 Hz), GPS update on every solution. z is ASL on the baro
 * scale inside, the output is z - asl0.
 *
 * bb is the slow error of the baro (temperature, wind, weather). It can only
 * be told apart from a real height change with the GPS height, so it is
 * frozen (variance 0, the filter is then exactly the three-state filter)
 * until the GPS height is in use, and again when the GPS height is lost.
 * gpsOff is taken when the GPS height joins, so the output does not jump.
 * -------------------------------------------------------------------------*/
#define KF_N 4
#define KF_USE_GPS_VEL 0x01
#define KF_USE_GPS_Z   0x02
// GPS height not accepted for this long: baro bias frozen, new gpsOff when it is back
#define KF_GPS_Z_TIMEOUT M2T(2000)

struct heightKF_s {
  bool init;
  float x[KF_N];    // z (m, ASL), vz (m/s), ba (m/s^2), bb (m)
  float P[KF_N][KF_N];
  float lastAsl;    // to detect a new baro sample
  float zRel;       // log: z - asl0
  float innov;      // log: baro - z (m)
  float sigZ;       // log: sqrt(P_zz) (m)
  uint32_t nRej;    // log: rejected baro samples
  uint16_t rejCount;// consecutive rejected baro samples
  uint32_t nReset;  // log: resets to the baro after rejTimeout rejected samples
  // parameters
  float sigBaro;    // baro noise (m)
  float sigAcc;     // acc noise per 250 Hz sample (m/s^2)
  float sigBias;    // acc bias random walk (m/s^2/sqrt(s))
  float gate;       // innovation gate (sigma)
  uint16_t rejTimeout; // reset to the baro after this many consecutive rejected samples
  uint16_t initDelay;  // predictions (250 Hz) after power-on during which the KF is held at the baro
  uint32_t steps;      // predictions since power-on
  // GPS
  bool gpsZActive;     // GPS height in use, baro bias free
  float gpsOff;        // hMSL - z when the GPS height joined (m)
  uint32_t lastGpsZTick;
  float innovGz;       // log: GPS height - z (m)
  float innovGv;       // log: GPS velD + vz (m/s)
  uint32_t nGps;       // log: GPS solutions used
  uint32_t nGpsRej;    // log: GPS height or velocity rejected by the gate
  uint8_t useGps;      // bit 0: vertical velocity, bit 1: height (3: both, 0: baro only)
  float sigBaroBias;   // baro bias random walk (m/sqrt(s))
  float sigGpsZ;       // lower limit of the GPS height noise (m)
  float sigGpsVz;      // lower limit of the GPS vertical velocity noise (m/s)
  float maxVAcc;       // GPS solutions with vAcc above this are not used (m)
};

static struct heightKF_s kf = {
  .init = false,
  .sigBaro = 1.0f,
  .sigAcc = 1.0f,
  .sigBias = 0.005f,
  .gate = 5.0f,
  .rejTimeout = 50,  // 1 s of baro at 50 Hz, as the EKF3 height reset
  .initDelay = 2500, // 10 s: the attitude (and so the vertical acc) settles first
  // Velocity only. With the GPS height (2026-11-16, MG-902) the estimate rose
  // by 0.85 m in 24 s of flight and by 7 m on the ground while the receiver
  // was still settling, and the GPS height missed a real 2.5 m ramp that the
  // baro and the GPS velocity both saw.
  .useGps = KF_USE_GPS_VEL,
  .sigBaroBias = 0.05f,
  .sigGpsZ = 3.0f,   // GPS height error is slow (metres over minutes), not white
  .sigGpsVz = 0.15f, // 0.05 m/s measured at rest, but it can be off by several cm/s for a while
  .maxVAcc = 5.0f,
};

static void heightKFInit(float asl)
{
  kf.x[0] = asl;
  kf.x[1] = 0.0f;
  kf.x[2] = 0.0f;
  kf.x[3] = 0.0f;
  for (int i = 0; i < KF_N; i++) {
    for (int j = 0; j < KF_N; j++) {
      kf.P[i][j] = 0.0f;
    }
  }
  kf.P[0][0] = kf.sigBaro * kf.sigBaro;
  kf.P[1][1] = 0.1f * 0.1f;  // still at power-on
  kf.P[2][2] = 0.5f * 0.5f;
  kf.gpsZActive = false;     // baro bias frozen at 0 (P[3][3] = 0)
  kf.lastAsl = asl;
  kf.rejCount = 0;
  kf.init = true;
}

// Baro bias known exactly from now on: it keeps its value and the filter
// behaves as the three-state filter
static void heightKFFreezeBaroBias(void)
{
  for (int i = 0; i < KF_N; i++) {
    kf.P[i][3] = 0.0f;
    kf.P[3][i] = 0.0f;
  }
  kf.gpsZActive = false;
}

// x = F x + B a,  P = F P F' + Q,  F = [1 dt -dt^2/2 0; 0 1 -dt 0; 0 0 1 0; 0 0 0 1]
static void heightKFPredict(float a, float dt)
{
  kf.steps++;
  if (!kf.init) {
    return;
  }
  float dt2 = 0.5f * dt * dt;
  float acc = a - kf.x[2];
  kf.x[0] += kf.x[1] * dt + acc * dt2;
  kf.x[1] += acc * dt;

  float F[KF_N][KF_N] = {{1.0f, dt, -dt2, 0.0f}, {0.0f, 1.0f, -dt, 0.0f},
                         {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
  float FP[KF_N][KF_N];
  for (int i = 0; i < KF_N; i++) {
    for (int j = 0; j < KF_N; j++) {
      FP[i][j] = 0.0f;
      for (int k = 0; k < KF_N; k++) {
        FP[i][j] += F[i][k] * kf.P[k][j];
      }
    }
  }
  for (int i = 0; i < KF_N; i++) {
    for (int j = 0; j < KF_N; j++) {
      kf.P[i][j] = 0.0f;
      for (int k = 0; k < KF_N; k++) {
        kf.P[i][j] += FP[i][k] * F[j][k];
      }
    }
  }
  // Q: acc noise enters through B = [dt^2/2, dt, 0, 0], bias random walks
  float qa = kf.sigAcc * kf.sigAcc;
  kf.P[0][0] += qa * dt2 * dt2;
  kf.P[0][1] += qa * dt2 * dt;
  kf.P[1][0] += qa * dt2 * dt;
  kf.P[1][1] += qa * dt * dt;
  kf.P[2][2] += kf.sigBias * kf.sigBias * dt;

  if (kf.gpsZActive) {
    if (!(kf.useGps & KF_USE_GPS_Z) ||
        xTaskGetTickCount() - kf.lastGpsZTick > KF_GPS_Z_TIMEOUT) {
      heightKFFreezeBaroBias();
    } else {
      kf.P[3][3] += kf.sigBaroBias * kf.sigBaroBias * dt;
    }
  }
}

// One scalar measurement with innovation e, measurement row H and variance r.
// Returns false, without changing the filter, if the innovation gate rejects it.
static bool heightKFMeasure(const float H[KF_N], float e, float r)
{
  float PH[KF_N];  // P H'
  float S = r;
  for (int i = 0; i < KF_N; i++) {
    PH[i] = 0.0f;
    for (int k = 0; k < KF_N; k++) {
      PH[i] += kf.P[i][k] * H[k];
    }
  }
  for (int k = 0; k < KF_N; k++) {
    S += H[k] * PH[k];
  }
  if (e * e > kf.gate * kf.gate * S) {
    return false;
  }
  // x += K e,  P = (I - K H) P with K = P H' / S, then keep it symmetric
  for (int i = 0; i < KF_N; i++) {
    kf.x[i] += PH[i] / S * e;
  }
  for (int i = 0; i < KF_N; i++) {
    for (int j = 0; j < KF_N; j++) {
      kf.P[i][j] -= PH[i] * PH[j] / S;
    }
  }
  for (int i = 0; i < KF_N; i++) {
    for (int j = i + 1; j < KF_N; j++) {
      float m = 0.5f * (kf.P[i][j] + kf.P[j][i]);
      kf.P[i][j] = m;
      kf.P[j][i] = m;
    }
  }
  return true;
}

// baro update, baro = z + bb
static void heightKFUpdate(float asl)
{
  static const float H[KF_N] = {1.0f, 0.0f, 0.0f, 1.0f};
  float e = asl - kf.x[0] - kf.x[3];
  kf.innov = e;
  if (!heightKFMeasure(H, e, kf.sigBaro * kf.sigBaro)) {
    kf.nRej++;
    kf.rejCount++;
    if (kf.rejCount >= kf.rejTimeout) {
      // the filter ran away from the baro (e.g. wrong acc while the attitude
      // is still converging at start-up): start again from the baro
      heightKFInit(asl);
      kf.nReset++;
    }
    return;
  }
  kf.rejCount = 0;
}

// GPS update: vertical velocity and height, see heightKF.useGps
void positionUpdateGpsHeight(const gpsMeasurement_t* gps)
{
  if (!kf.init || kf.steps < kf.initDelay || !kf.useGps) {
    return;
  }
  if (!gps->fixOk || gps->fixType < 3 || gps->vAcc > kf.maxVAcc) {
    return;
  }
  bool rejected = false;

  if (kf.useGps & KF_USE_GPS_VEL) {
    static const float H[KF_N] = {0.0f, -1.0f, 0.0f, 0.0f};  // velD is positive down
    float sig = fmaxf(gps->sAcc, kf.sigGpsVz);
    kf.innovGv = gps->velD + kf.x[1];
    rejected |= !heightKFMeasure(H, kf.innovGv, sig * sig);
  }

  if (kf.useGps & KF_USE_GPS_Z) {
    if (!kf.gpsZActive) {
      // the GPS height joins (or comes back) at the current estimate
      kf.gpsOff = gps->hMSL - kf.x[0];
      kf.gpsZActive = true;
      kf.lastGpsZTick = xTaskGetTickCount();
      kf.innovGz = 0.0f;
    } else {
      static const float H[KF_N] = {1.0f, 0.0f, 0.0f, 0.0f};
      float sig = fmaxf(gps->vAcc, kf.sigGpsZ);
      kf.innovGz = gps->hMSL - kf.gpsOff - kf.x[0];
      if (heightKFMeasure(H, kf.innovGz, sig * sig)) {
        kf.lastGpsZTick = xTaskGetTickCount();
      } else {
        rejected = true;
      }
    }
  }

  if (rejected) {
    kf.nGpsRej++;
  } else {
    kf.nGps++;
  }
}

static void positionEstimateInternal(state_t* estimate, const baro_t* baro, const tofMeasurement_t* tofMeasurement, float dt, stabilizerStep_t stabilizerStep, struct selfState_s* state);
static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state);

void positionEstimate(state_t* estimate, const baro_t* baro, const tofMeasurement_t* tofMeasurement, float dt, stabilizerStep_t stabilizerStep) {
  positionEstimateInternal(estimate, baro, tofMeasurement, dt, stabilizerStep, &state);
}

void positionUpdateVelocity(float accWZ, float dt) {
  positionUpdateVelocityInternal(accWZ, dt, &state);
}

static void positionEstimateInternal(state_t* estimate, const baro_t* baro, const tofMeasurement_t* tofMeasurement, float dt, stabilizerStep_t stabilizerStep, struct selfState_s* state) {
  float filteredZ;
  static float prev_estimatedZ = 0;
  static bool surfaceFollowingMode = false;

  const uint32_t MAX_SAMPLE_AGE = M2T(50);

  uint32_t now = xTaskGetTickCount();
  bool isSampleUseful = ((now - tofMeasurement->timestamp) <= MAX_SAMPLE_AGE);

  if (isSampleUseful) {
    surfaceFollowingMode = true;
  }

  if (surfaceFollowingMode) {
    if (isSampleUseful) {
      // IIR filter zrange
      filteredZ = (state->estAlphaZrange       ) * state->estimatedZ +
                  (1.0f - state->estAlphaZrange) * tofMeasurement->distance;
      // Use zrange as base and add velocity changes.
      state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);
    }
  } else {
    // FIXME: A bit of an hack to init IIR filter
    if (state->estimatedZ == 0.0f) {
      filteredZ = baro->asl;
    } else {
      // IIR filter asl
      filteredZ = (state->estAlphaAsl       ) * state->estimatedZ +
                  (1.0f - state->estAlphaAsl) * baro->asl;
    }
    #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
      state->estimatedZ = filteredZ;
    #else
      // Use asl as base and add velocity changes.
      state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);
    #endif

    // start-up ASL: running mean over the first second with baro data, then fixed
    if (baro->asl != 0.0f && state->asl0Count < ASL0_SAMPLES) {
      state->asl0Count++;
      state->asl0 += (baro->asl - state->asl0) / state->asl0Count;
    }

    // Kalman filter: init at the first baro data, update on every new baro sample
    if (baro->asl != 0.0f) {
      if (!kf.init || kf.steps < kf.initDelay) {
        // start-up: held at the baro (z = baro, vz = 0, bias = 0) while the
        // attitude converges, otherwise the KF learns a wrong acc bias
        heightKFInit(baro->asl);
      } else if (baro->asl != kf.lastAsl) {
        kf.lastAsl = baro->asl;
        heightKFUpdate(baro->asl);
      }
    }
  }

  // Bitcraze filter output (relative to asl0 in baro mode)
  float zBitcraze = surfaceFollowingMode ? state->estimatedZ : state->estimatedZ - state->asl0;
  float vzBitcraze = (state->estimatedZ - prev_estimatedZ) / dt;
  state->zRel = zBitcraze;
  state->estimatedVZ = vzBitcraze;
  prev_estimatedZ = state->estimatedZ;

  // Kalman filter output
  kf.zRel = kf.x[0] - state->asl0;
  kf.sigZ = sqrtf(kf.P[0][0]);

  // position.x / y: position_estimator_horizontal.c
  if (useKF && kf.init && !surfaceFollowingMode) {
    estimate->position.z = kf.zRel;
    estimate->velocity.z = kf.x[1];
  } else {
    estimate->position.z = zBitcraze;
    estimate->velocity.z = vzBitcraze;
  }
}

static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state) {
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * G;
  state->velocityZ *= state->velZAlpha;

  // Kalman filter prediction, accWZ in g without gravity
  heightKFPredict(accWZ * GRAVITY_MAGNITUDE, dt);
}

LOG_GROUP_START(posEstAlt)
LOG_ADD(LOG_FLOAT, estimatedZ, &state.estimatedZ)
LOG_ADD(LOG_FLOAT, estVZ, &state.estimatedVZ)
LOG_ADD(LOG_FLOAT, velocityZ, &state.velocityZ)
LOG_ADD(LOG_FLOAT, asl0, &state.asl0)
LOG_ADD(LOG_FLOAT, z, &state.zRel)
LOG_GROUP_STOP(posEstAlt)

LOG_GROUP_START(heightKF)
LOG_ADD(LOG_FLOAT, z, &kf.zRel)
LOG_ADD(LOG_FLOAT, vz, &kf.x[1])
LOG_ADD(LOG_FLOAT, ba, &kf.x[2])
/**
 * @brief Baro bias [m], the part of the baro height the filter attributes to
 *        the baro itself. Only moves while the GPS height is in use.
 */
LOG_ADD(LOG_FLOAT, bb, &kf.x[3])
LOG_ADD(LOG_FLOAT, innGz, &kf.innovGz)
LOG_ADD(LOG_FLOAT, innGv, &kf.innovGv)
LOG_ADD(LOG_UINT8, gpsZ, &kf.gpsZActive)
LOG_ADD(LOG_UINT32, nGps, &kf.nGps)
LOG_ADD(LOG_UINT32, nGpsRej, &kf.nGpsRej)
LOG_ADD(LOG_FLOAT, innov, &kf.innov)
LOG_ADD(LOG_FLOAT, sigZ, &kf.sigZ)
LOG_ADD(LOG_UINT32, nRej, &kf.nRej)
LOG_ADD(LOG_UINT32, nReset, &kf.nReset)
LOG_GROUP_STOP(heightKF)

/**
 * Tuning setttings for the altitude estimator/filtering
 */
PARAM_GROUP_START(posEstAlt)
/**
 * @brief parameter alpha IIR Filter above sea level/true altitude (baro)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, estAlphaAsl, &state.estAlphaAsl)
/**
 * @brief parameter alpha IIR Filter Height  (zranger)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, estAlphaZr, &state.estAlphaZrange)
/**
 * @brief Multiplying factor for adding velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, velFactor, &state.velocityFactor)
/**
 * @brief Blendning factor to avoid accumulate error
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, velZAlpha, &state.velZAlpha)
/**
 * @brief Vertical acceleration deadband
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, vAccDeadband, &state.vAccDeadband)
/**
 * @brief 1: Kalman filter (heightKF) to stateEstimate.z/vz, 0: Bitcraze filter
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, useKF, &useKF)
PARAM_GROUP_STOP(posEstAlt)

PARAM_GROUP_START(heightKF)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigBaro, &kf.sigBaro)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigAcc, &kf.sigAcc)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigBias, &kf.sigBias)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, gate, &kf.gate)
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, rejTimeout, &kf.rejTimeout)
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, initDelay, &kf.initDelay)
/**
 * @brief GPS in the height filter: bit 0 vertical velocity, bit 1 height
 *        (0: baro and acc only, 3: both)
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, useGps, &kf.useGps)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigBaroBias, &kf.sigBaroBias)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigGpsZ, &kf.sigGpsZ)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigGpsVz, &kf.sigGpsVz)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, maxVAcc, &kf.maxVAcc)
PARAM_GROUP_STOP(heightKF)
