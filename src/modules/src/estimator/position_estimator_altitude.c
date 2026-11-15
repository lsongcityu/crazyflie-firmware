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
 *  - a linear Kalman filter (heightKF), states z, vz, acc bias
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
 * Linear Kalman filter for the height, x = [z, vz, ba]
 *   z' = vz,  vz' = a - ba + w_a,  ba' = w_b,   baro = z + n
 * a: vertical acc without gravity (sensfusion6GetAccZWithoutGravity, m/s^2)
 * prediction at 250 Hz (positionUpdateVelocity), baro update on every new
 * baro sample (50 Hz). z is ASL inside, the output is z - asl0.
 * -------------------------------------------------------------------------*/
struct heightKF_s {
  bool init;
  float x[3];       // z (m, ASL), vz (m/s), ba (m/s^2)
  float P[3][3];
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
};

static struct heightKF_s kf = {
  .init = false,
  .sigBaro = 1.0f,
  .sigAcc = 1.0f,
  .sigBias = 0.005f,
  .gate = 5.0f,
  .rejTimeout = 50,  // 1 s of baro at 50 Hz, as the EKF3 height reset
  .initDelay = 2500, // 10 s: the attitude (and so the vertical acc) settles first
};

static void heightKFInit(float asl)
{
  kf.x[0] = asl;
  kf.x[1] = 0.0f;
  kf.x[2] = 0.0f;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      kf.P[i][j] = 0.0f;
    }
  }
  kf.P[0][0] = kf.sigBaro * kf.sigBaro;
  kf.P[1][1] = 0.1f * 0.1f;  // still at power-on
  kf.P[2][2] = 0.5f * 0.5f;
  kf.lastAsl = asl;
  kf.rejCount = 0;
  kf.init = true;
}

// x = F x + B a,  P = F P F' + Q,  F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1]
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

  float F[3][3] = {{1.0f, dt, -dt2}, {0.0f, 1.0f, -dt}, {0.0f, 0.0f, 1.0f}};
  float FP[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      FP[i][j] = F[i][0] * kf.P[0][j] + F[i][1] * kf.P[1][j] + F[i][2] * kf.P[2][j];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      kf.P[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
    }
  }
  // Q: acc noise enters through B = [dt^2/2, dt, 0], bias random walk
  float qa = kf.sigAcc * kf.sigAcc;
  kf.P[0][0] += qa * dt2 * dt2;
  kf.P[0][1] += qa * dt2 * dt;
  kf.P[1][0] += qa * dt2 * dt;
  kf.P[1][1] += qa * dt * dt;
  kf.P[2][2] += kf.sigBias * kf.sigBias * dt;
}

// baro update, H = [1 0 0]
static void heightKFUpdate(float asl)
{
  float e = asl - kf.x[0];
  float S = kf.P[0][0] + kf.sigBaro * kf.sigBaro;
  kf.innov = e;
  if (e * e > kf.gate * kf.gate * S) {
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
  float K[3] = {kf.P[0][0] / S, kf.P[1][0] / S, kf.P[2][0] / S};
  for (int i = 0; i < 3; i++) {
    kf.x[i] += K[i] * e;
  }
  // P = (I - K H) P, then keep it symmetric
  float P0[3] = {kf.P[0][0], kf.P[0][1], kf.P[0][2]};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      kf.P[i][j] -= K[i] * P0[j];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 3; j++) {
      float m = 0.5f * (kf.P[i][j] + kf.P[j][i]);
      kf.P[i][j] = m;
      kf.P[j][i] = m;
    }
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

  estimate->position.x = 0.0f;
  estimate->position.y = 0.0f;
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
PARAM_GROUP_STOP(heightKF)
