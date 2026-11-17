/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 * position_estimator_horizontal.c: horizontal position estimator, GPS + acc
 *
 * Same structure as the height Kalman filter in position_estimator_altitude.c,
 * one linear Kalman filter per axis, x = [p, v, ba]:
 *   p' = v,  v' = a - ba + w_a,  ba' = w_b
 *   GPS position = p + n_p,  GPS velocity = v + n_v
 * a: horizontal acc in the world frame (m/s^2), prediction at 250 Hz.
 * ba: acc bias in the world frame. It mostly absorbs the roll/pitch error of
 *     the attitude filter (1 deg of tilt error looks like 0.17 m/s^2).
 * GPS update on every UBX-NAV-PVT solution, noise from the accuracy the
 * receiver reports (hAcc, sAcc) with a lower limit.
 *
 * World frame: ENU, x east, y north, the same frame as the compass yaw of
 * sensfusion6 (yaw counterclockwise from east). The origin is the first
 * usable GPS position; posEstXY.reset = 1 moves it to the current position.
 *
 * East and north have the same model and the same noise, and a GPS solution
 * is accepted or rejected for both, so they share one covariance matrix.
 *
 * GPS delay: a GPS solution describes the robot about 0.1 s ago. In the logs
 * of 2026-11-16 the GPS velocity innovation is smallest for 0.08 s (ArduPilot
 * uses 0.12 s for a u-blox M9). The position and velocity estimates of the last
 * 0.25 s are kept, and a GPS solution is compared with the estimate of
 * posEstXY.gpsDelay ago instead of the current one. The correction found
 * that way is applied to the current estimate and to the stored ones (they
 * all move together). gpsDelay = 0 compares with the current estimate.
 */

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "position_estimator.h"
#include "sensfusion6.h"

// one unit of lat/lon (1e-7 deg) in m along a meridian
#define EARTH_RADIUS 6378137.0f
#define LSB_TO_M (1e-7f * (float)M_PI / 180.0f * EARTH_RADIUS)

// stored estimates for the GPS delay, one per prediction (250 Hz): 0.256 s
#define XY_HIST_LEN 64

enum {
  XY_STARTUP = 0,    // start-up delay, the attitude is still settling
  XY_NO_YAW,         // the compass has not set the yaw, the world frame is not ENU
  XY_NO_GPS,         // waiting for the first usable GPS solution
  XY_OK,             // running with GPS
  XY_ACC_ONLY,       // running, but no accepted GPS for longer than timeout: do not use
};

struct xyKF_s {
  bool init;
  float p[2];       // east, north (m, from the origin)
  float v[2];       // east, north (m/s)
  float b[2];       // acc bias east, north (m/s^2)
  float P[3][3];    // covariance of [p, v, ba], the same for both axes
  bool haveOrigin;
  int32_t lat0;     // origin (1e-7 deg)
  int32_t lon0;
  float lonToM;     // LSB_TO_M * cos(lat0)
  uint32_t lastAcceptTick;
  uint32_t steps;   // predictions since power-on
  float histP[2][XY_HIST_LEN];  // p and v after each of the last predictions
  float histV[2][XY_HIST_LEN];
  uint8_t histIdx;  // newest entry
  float dt;         // prediction step (s)
  // log
  float gps[2];     // GPS position east, north (m, from the origin)
  float innovP[2];  // GPS position - estimate (m)
  float innovV[2];  // GPS velocity - estimate (m/s)
  float sigP;       // sqrt(P_pp) (m)
  float sigV;       // sqrt(P_vv) (m/s)
  uint8_t status;
  uint32_t nGps;      // accepted GPS solutions
  uint32_t nRej;      // rejected by the innovation gate
  uint32_t nUnusable; // no 3D fix, or hAcc above maxHAcc
  uint32_t nReset;    // restarts from the GPS after timeout
  // parameters
  float sigAcc;     // acc noise per 250 Hz sample (m/s^2)
  float sigBias;    // acc bias random walk (m/s^2/sqrt(s))
  float sigPosMin;  // lower limit of the GPS position noise (m)
  float sigVelMin;  // lower limit of the GPS velocity noise (m/s)
  float gate;       // innovation gate (sigma)
  float timeout;    // s without accepted GPS: status acc only, restart from the next usable GPS
  float maxHAcc;    // GPS solutions with hAcc above this are not used (m)
  uint16_t initDelay; // predictions (250 Hz) after power-on before the filter may start
  float gpsDelay;   // age of a GPS solution when it arrives (s), 0: no compensation
  uint8_t reset;    // set to 1: new origin at the current GPS position
};

static struct xyKF_s kf = {
  .init = false,
  .haveOrigin = false,
  .status = XY_STARTUP,
  .sigAcc = 1.0f,     // as the height filter
  .sigBias = 0.05f,   // tilt error changes within seconds, faster than a sensor bias
  .sigPosMin = 1.0f,  // GPS position error is slow, not white: do not follow it closely
  .sigVelMin = 0.2f,  // 0.2 m/s sample to sample measured at 25 Hz (2026-11-15)
  .gate = 5.0f,
  .timeout = 1.0f,
  .maxHAcc = 5.0f,
  .initDelay = 2500,  // 10 s, after the initial compass yaw (8.2 s)
  .gpsDelay = 0.08f,   // smallest GPS velocity innovation in the logs of 2026-11-16
  .dt = 0.004f,
  .reset = 0,
};

static void xyKFInit(const float p[2], const float v[2], float sigP, float sigV)
{
  for (int a = 0; a < 2; a++) {
    kf.p[a] = p[a];
    kf.v[a] = v[a];
    kf.b[a] = 0.0f;
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      kf.P[i][j] = 0.0f;
    }
  }
  kf.P[0][0] = sigP * sigP;
  kf.P[1][1] = sigV * sigV;
  kf.P[2][2] = 0.5f * 0.5f;
  for (int a = 0; a < 2; a++) {
    for (int i = 0; i < XY_HIST_LEN; i++) {
      kf.histP[a][i] = p[a];
      kf.histV[a][i] = v[a];
    }
  }
  kf.lastAcceptTick = xTaskGetTickCount();
  kf.init = true;
}

// x = F x + B a,  P = F P F' + Q,  F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1]
void positionXYPredict(float accE, float accN, float dt)
{
  kf.steps++;
  if (!kf.init) {
    if (kf.steps < kf.initDelay) {
      kf.status = XY_STARTUP;
    } else if (!sensfusion6HasCompassYaw()) {
      kf.status = XY_NO_YAW;
    } else {
      kf.status = XY_NO_GPS;
    }
    return;
  }

  const float a[2] = {accE, accN};
  float dt2 = 0.5f * dt * dt;
  for (int i = 0; i < 2; i++) {
    float acc = a[i] - kf.b[i];
    kf.p[i] += kf.v[i] * dt + acc * dt2;
    kf.v[i] += acc * dt;
  }

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

  kf.sigP = sqrtf(kf.P[0][0]);
  kf.sigV = sqrtf(kf.P[1][1]);

  // keep the estimate for the GPS delay
  kf.dt = dt;
  kf.histIdx = (kf.histIdx + 1) % XY_HIST_LEN;
  for (int i = 0; i < 2; i++) {
    kf.histP[i][kf.histIdx] = kf.p[i];
    kf.histV[i][kf.histIdx] = kf.v[i];
  }

  if (xTaskGetTickCount() - kf.lastAcceptTick > M2T(1000.0f * kf.timeout)) {
    kf.status = XY_ACC_ONLY;
  } else {
    kf.status = XY_OK;
  }
}

// Measurement of state h (0: position, 1: velocity) for both axes,
// e: innovation east, north, r: measurement variance
static void xyKFUpdate(int h, const float e[2], float r)
{
  float S = kf.P[h][h] + r;
  float K[3] = {kf.P[0][h] / S, kf.P[1][h] / S, kf.P[2][h] / S};
  for (int a = 0; a < 2; a++) {
    kf.p[a] += K[0] * e[a];
    kf.v[a] += K[1] * e[a];
    kf.b[a] += K[2] * e[a];
    // the stored estimates get the same correction
    for (int i = 0; i < XY_HIST_LEN; i++) {
      kf.histP[a][i] += K[0] * e[a];
      kf.histV[a][i] += K[1] * e[a];
    }
  }
  // P = (I - K H) P, then keep it symmetric
  float Ph[3] = {kf.P[h][0], kf.P[h][1], kf.P[h][2]};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      kf.P[i][j] -= K[i] * Ph[j];
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

void positionXYUpdateGps(const gpsMeasurement_t* gps)
{
  if (!gps->fixOk || gps->fixType < 3 || gps->hAcc > kf.maxHAcc) {
    kf.nUnusable++;
    return;
  }
  // The acc is rotated with the attitude, so the filter only makes sense
  // once the attitude has settled and its yaw is referenced to north
  if (kf.steps < kf.initDelay || !sensfusion6HasCompassYaw()) {
    return;
  }

  const float zv[2] = {gps->velE, gps->velN};
  float sigP = fmaxf(gps->hAcc, kf.sigPosMin);
  float sigV = fmaxf(gps->sAcc, kf.sigVelMin);

  if (!kf.haveOrigin || kf.reset) {
    kf.lat0 = gps->lat;
    kf.lon0 = gps->lon;
    kf.lonToM = LSB_TO_M * cosf(gps->lat * 1e-7f * (float)M_PI / 180.0f);
    kf.haveOrigin = true;
    kf.reset = 0;
    const float zero[2] = {0.0f, 0.0f};
    kf.gps[0] = 0.0f;
    kf.gps[1] = 0.0f;
    xyKFInit(zero, zv, sigP, sigV);
    return;
  }

  // The difference is taken in integers: a float cannot hold lat/lon itself
  const float zp[2] = {(float)(gps->lon - kf.lon0) * kf.lonToM,
                       (float)(gps->lat - kf.lat0) * LSB_TO_M};
  kf.gps[0] = zp[0];
  kf.gps[1] = zp[1];

  if (xTaskGetTickCount() - kf.lastAcceptTick > M2T(1000.0f * kf.timeout)) {
    // no GPS, or every solution rejected, for longer than timeout: the acc
    // alone drifts by metres within seconds, so start again from the GPS
    // (moved forward by the distance travelled during the GPS delay)
    const float pNow[2] = {zp[0] + zv[0] * kf.gpsDelay, zp[1] + zv[1] * kf.gpsDelay};
    xyKFInit(pNow, zv, sigP, sigV);
    kf.nReset++;
    return;
  }

  // the estimate at the time the GPS solution is valid for
  int back = (int)(kf.gpsDelay / kf.dt + 0.5f);
  if (back < 0) {
    back = 0;
  } else if (back > XY_HIST_LEN - 1) {
    back = XY_HIST_LEN - 1;
  }
  const int then = (kf.histIdx + XY_HIST_LEN - back) % XY_HIST_LEN;

  float eP[2], eV[2];
  for (int a = 0; a < 2; a++) {
    eP[a] = zp[a] - kf.histP[a][then];
    eV[a] = zv[a] - kf.histV[a][then];
    kf.innovP[a] = eP[a];
    kf.innovV[a] = eV[a];
  }
  float g2 = kf.gate * kf.gate;
  if (eP[0] * eP[0] + eP[1] * eP[1] > g2 * (kf.P[0][0] + sigP * sigP) ||
      eV[0] * eV[0] + eV[1] * eV[1] > g2 * (kf.P[1][1] + sigV * sigV)) {
    kf.nRej++;
    return;
  }

  xyKFUpdate(0, eP, sigP * sigP);
  // velocity innovation with the state after the position update
  for (int a = 0; a < 2; a++) {
    eV[a] = zv[a] - kf.histV[a][then];
  }
  xyKFUpdate(1, eV, sigV * sigV);

  kf.lastAcceptTick = xTaskGetTickCount();
  kf.nGps++;
}

void positionXYGetState(state_t* estimate)
{
  estimate->position.x = kf.p[0];
  estimate->position.y = kf.p[1];
  estimate->velocity.x = kf.v[0];
  estimate->velocity.y = kf.v[1];
}

/**
 * Horizontal position estimator (GPS + acc), world frame ENU: x east, y north.
 * The estimate itself is stateEstimate.x / y / vx / vy.
 */
LOG_GROUP_START(posEstXY)
LOG_ADD(LOG_FLOAT, x, &kf.p[0])
LOG_ADD(LOG_FLOAT, y, &kf.p[1])
LOG_ADD(LOG_FLOAT, vx, &kf.v[0])
LOG_ADD(LOG_FLOAT, vy, &kf.v[1])
LOG_ADD(LOG_FLOAT, bx, &kf.b[0])
LOG_ADD(LOG_FLOAT, by, &kf.b[1])
/**
 * @brief GPS position in the same frame [m], to compare with x / y
 */
LOG_ADD(LOG_FLOAT, gpsX, &kf.gps[0])
LOG_ADD(LOG_FLOAT, gpsY, &kf.gps[1])
LOG_ADD(LOG_FLOAT, innPx, &kf.innovP[0])
LOG_ADD(LOG_FLOAT, innPy, &kf.innovP[1])
LOG_ADD(LOG_FLOAT, innVx, &kf.innovV[0])
LOG_ADD(LOG_FLOAT, innVy, &kf.innovV[1])
LOG_ADD(LOG_FLOAT, sigP, &kf.sigP)
LOG_ADD(LOG_FLOAT, sigV, &kf.sigV)
/**
 * @brief 0 start-up delay, 1 no compass yaw, 2 no usable GPS yet, 3 running,
 *        4 no accepted GPS for longer than timeout (acc only, do not use)
 */
LOG_ADD(LOG_UINT8, status, &kf.status)
LOG_ADD(LOG_UINT32, nGps, &kf.nGps)
LOG_ADD(LOG_UINT32, nRej, &kf.nRej)
LOG_ADD(LOG_UINT32, nUnusable, &kf.nUnusable)
LOG_ADD(LOG_UINT32, nReset, &kf.nReset)
LOG_ADD(LOG_INT32, lat0, &kf.lat0)
LOG_ADD(LOG_INT32, lon0, &kf.lon0)
LOG_GROUP_STOP(posEstXY)

PARAM_GROUP_START(posEstXY)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigAcc, &kf.sigAcc)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigBias, &kf.sigBias)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigPosMin, &kf.sigPosMin)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, sigVelMin, &kf.sigVelMin)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, gate, &kf.gate)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, timeout, &kf.timeout)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, maxHAcc, &kf.maxHAcc)
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, initDelay, &kf.initDelay)
/**
 * @brief Age of a GPS solution when it arrives [s], at most 0.25. 0: not compensated
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, gpsDelay, &kf.gpsDelay)
/**
 * @brief Set to 1 to move the origin to the current GPS position (goes back
 *        to 0 by itself). The position jumps, so not while position control is on.
 */
PARAM_ADD(PARAM_UINT8, reset, &kf.reset)
PARAM_GROUP_STOP(posEstXY)
