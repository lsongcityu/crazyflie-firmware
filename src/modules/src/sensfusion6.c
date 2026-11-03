/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 */
#include <math.h>

#include "sensfusion6.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"

#include "autoconf.h"

static uint8_t gravity_correction = 1;

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
  #define BETA_DEF     0.01f    // 2 * proportional gain
#else // MAHONY_QUATERNION_IMU
    #define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
    #define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain
#endif

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
  float beta = BETA_DEF;     // 2 * proportional gain (Kp)
#else // MAHONY_QUATERNION_IMU
  float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
  float integralFBx = 0.0f;
  float integralFBy = 0.0f;
  float integralFBz = 0.0f;  // integral error terms scaled by Ki
#endif

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

void quaternion_multiply(
  float q1w, float q1x, float q1y, float q1z,
  float q2w, float q2x, float q2y, float q2z,
  float *q3w, float *q3x, float *q3y, float *q3z)
{
*q3w = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z;
*q3x = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y;
*q3y = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x;
*q3z = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w;
}

void setquat(float w, float x, float y, float z)
{
qw = w;
qx = x;
qy = y;
qz = z;
}

void applyquat(float w, float x, float y, float z) // Relta_R * R
{
quaternion_multiply(
    w, x, y, z,
    qw, qx, qy, qz,
    &qw, &qx, &qy, &qz);
}

void applyquat_body(float w, float x, float y, float z) //  R * Relta_R
{
quaternion_multiply(
    qw, qx, qy, qz,
    w, x, y, z,
    &qw, &qx, &qy, &qz);
}


static float gravX, gravY, gravZ; // Unit vector in the estimated gravity direction

// The acc in Z for static position (g)
// Set on first update, assuming we are in a static position since the sensors were just calibrates.
// This value will be better the more level the copter is at calibration time
static float baseZacc = 1.0;

static bool isInit;

static bool isCalibrated = false;

static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static float sensfusion6GetAccZ(const float ax, const float ay, const float az);
static void estimatedGravityDirection(float* gx, float* gy, float* gz);

// TODO: Make math util file
static float invSqrt(float x);

void sensfusion6Init()
{
  if(isInit)
    return;

  isInit = true;
}

bool sensfusion6Test(void)
{
  return isInit;
}


/* ---------------------------------------------------------------------------
 * Compass yaw correction (MG-902 deck), simplified from ArduPilot AP_AHRS_DCM
 * yaw_error_compass() / drift_correction_yaw():
 *  - the compass is rotated into the world frame and only its horizontal part
 *    is compared with magnetic north, so the compass only corrects yaw,
 *  - on an accepted sample the gyro gets an extra rotation rate kp*innov
 *    about the world vertical, on a rejected sample nothing,
 *  - after initDelay steps the first good sample sets the yaw directly.
 * Gates: |mag| in magMin..magMax, field-up angle in angRef +- angTol and
 * |innovation| < gate. If the innovation gate rejects for longer than gateT
 * (field gates still passing), it is bypassed so a wrong yaw is pulled back.
 *
 * World frame: ENU (x east, y north, z up), yaw counterclockwise from east.
 * -------------------------------------------------------------------------*/
#define DEG2RAD (M_PI_F / 180.0f)
#define RAD2DEG (180.0f / M_PI_F)

enum {
  MAGYAW_NO_DATA = 0,
  MAGYAW_ACCEPTED,
  MAGYAW_REJECT_NORM,      // |mag| outside magMin..magMax
  MAGYAW_REJECT_ANGLE,     // field-up angle outside angRef +- angTol
  MAGYAW_REJECT_INNOV,     // |innovation| > gate
  MAGYAW_ACCEPTED_TIMEOUT, // innovation gate bypassed after gateT
  MAGYAW_INITIAL,          // initial yaw set from the compass
  MAGYAW_DISABLED,
};

// parameters
static uint8_t magYawEnable = 1;
static float magDecl = -10.0f;       // deg, east positive (Toronto -10)
static float magKp = 0.18f;          // 1/s, DCM AHRS_YAW_P 0.2 * 0.9 at hover
static float magMin = 0.185f;        // gauss
static float magMax = 0.875f;        // gauss
static float magAngRef = 157.0f;     // deg, angle between field and world up
static float magAngTol = 25.0f;      // deg
static float magGate = 30.0f;        // deg, innovation gate
static float magGateT = 5.0f;        // s, bypass the innovation gate after this
static uint16_t magInitDelay = 2050; // sensfusion steps (250 Hz) before the initial yaw

// input
static float magBx, magBy, magBz;
static bool magNew = false;

// state
static bool haveInitialYaw = false;
static float yawCorr = 0.0f;     // rad/s, correction about world z
static float magDt = 0.0f;       // s since the last compass sample
static float innovBadT = 0.0f;   // s of continuous innovation rejection
static uint32_t ufSteps = 0;

// log
static float logHeadMag;   // deg, compass heading, clockwise from true north
static float logHeadEst;   // deg, estimator heading, clockwise from true north
static float logHeadGyro;  // deg, gyro-only heading from the initial yaw
static float logInnov;     // deg, compass - estimate
static float logNorm;      // gauss
static float logAngle;     // deg, field-up angle
static float logCorr;      // deg/s, yaw correction
static uint8_t magYawStatus = MAGYAW_NO_DATA;
static uint32_t nAccept, nRejNorm, nRejAngle, nRejInnov, nTimeout;

void sensfusion6UpdateMag(float mx, float my, float mz)
{
  magBx = mx;
  magBy = my;
  magBz = mz;
  magNew = true;
}

static float wrap360(float a)
{
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

// gx, gy, gz in deg/s; returns the yaw correction about world z in rad/s
static float magYawUpdate(float gx, float gy, float gz, float dt)
{
  ufSteps++;
  magDt += dt;

  // body -> world rotation matrix of the current estimate
  float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
  float r12 = 2.0f * (qx * qy - qw * qz);
  float r13 = 2.0f * (qx * qz + qw * qy);
  float r21 = 2.0f * (qx * qy + qw * qz);
  float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
  float r23 = 2.0f * (qy * qz - qw * qx);
  float r31 = 2.0f * (qx * qz - qw * qy);
  float r32 = 2.0f * (qy * qz + qw * qx);
  float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

  float yawEst = atan2f(r21, r11);
  logHeadEst = wrap360(90.0f - yawEst * RAD2DEG);
  // gyro-only heading: world z rate = (third row of R) . gyro
  logHeadGyro = wrap360(logHeadGyro - (r31 * gx + r32 * gy + r33 * gz) * dt);

  if (!magYawEnable) {
    magYawStatus = MAGYAW_DISABLED;
    yawCorr = 0.0f;
    return 0.0f;
  }
  if (!magNew) {
    return yawCorr;  // hold the last correction until the next sample
  }
  magNew = false;
  float dtm = magDt;
  magDt = 0.0f;
  yawCorr = 0.0f;  // stays 0 if this sample is rejected

  // field in the world frame
  float mwx = r11 * magBx + r12 * magBy + r13 * magBz;
  float mwy = r21 * magBx + r22 * magBy + r23 * magBz;
  float mwz = r31 * magBx + r32 * magBy + r33 * magBz;
  float norm = sqrtf(mwx * mwx + mwy * mwy + mwz * mwz);
  logNorm = norm;
  logAngle = (norm > 0.0f) ? acosf(mwz / norm) * RAD2DEG : 0.0f;

  // innovation: angle from the measured horizontal field to magnetic north,
  // magnetic north in ENU = (sin decl, cos decl)
  float nx = sinf(magDecl * DEG2RAD);
  float ny = cosf(magDecl * DEG2RAD);
  float innov = atan2f(mwx * ny - mwy * nx, mwx * nx + mwy * ny);
  logInnov = innov * RAD2DEG;
  logHeadMag = wrap360(90.0f - (yawEst + innov) * RAD2DEG);

  if (norm < magMin || norm > magMax) {
    magYawStatus = MAGYAW_REJECT_NORM;
    nRejNorm++;
    return 0.0f;
  }
  if (fabsf(logAngle - magAngRef) > magAngTol) {
    magYawStatus = MAGYAW_REJECT_ANGLE;
    nRejAngle++;
    return 0.0f;
  }

  if (!haveInitialYaw) {
    if (ufSteps < magInitDelay) {
      return 0.0f;  // let roll and pitch settle first
    }
    applyquat(cosf(0.5f * innov), 0.0f, 0.0f, sinf(0.5f * innov));
    haveInitialYaw = true;
    logHeadGyro = logHeadMag;
    magYawStatus = MAGYAW_INITIAL;
    return 0.0f;
  }

  if (fabsf(logInnov) > magGate) {
    innovBadT += dtm;
    if (innovBadT < magGateT) {
      magYawStatus = MAGYAW_REJECT_INNOV;
      nRejInnov++;
      return 0.0f;
    }
    magYawStatus = MAGYAW_ACCEPTED_TIMEOUT;
    nTimeout++;
  } else {
    innovBadT = 0.0f;
    magYawStatus = MAGYAW_ACCEPTED;
    nAccept++;
  }

  yawCorr = magKp * innov;
  return yawCorr;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  // compass yaw correction as a rotation rate about the world vertical,
  // added to the gyro in the body frame: w_b += corr * (R^T e_z)
  float corr = magYawUpdate(gx, gy, gz, dt) * RAD2DEG;  // deg/s
  logCorr = corr;
  gx += corr * 2.0f * (qx * qz - qw * qy);
  gy += corr * 2.0f * (qy * qz + qw * qx);
  gz += corr * (1.0f - 2.0f * (qx * qx + qy * qy));

  sensfusion6UpdateQImpl(gx, gy, gz, ax, ay, az, dt);
  estimatedGravityDirection(&gravX, &gravY, &gravZ);

  if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}

#ifdef CONFIG_IMU_MADGWICK_QUATERNION
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy ,_8qx, _8qy, qwqw, qxqx, qyqy, qzqz;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz);
  qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy);
  qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx);
  qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (gravity_correction)
  {
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2qw = 2.0f * qw;
    _2qx = 2.0f * qx;
    _2qy = 2.0f * qy;
    _2qz = 2.0f * qz;
    _4qw = 4.0f * qw;
    _4qx = 4.0f * qx;
    _4qy = 4.0f * qy;
    _8qx = 8.0f * qx;
    _8qy = 8.0f * qy;
    qwqw = qw * qw;
    qxqx = qx * qx;
    qyqy = qy * qy;
    qzqz = qz * qz;

    // Gradient decent algorithm corrective step
    s0 = _4qw * qyqy + _2qy * ax + _4qw * qxqx - _2qx * ay;
    s1 = _4qx * qzqz - _2qz * ax + 4.0f * qwqw * qx - _2qw * ay - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * az;
    s2 = 4.0f * qwqw * qy + _2qw * ax + _4qy * qzqz - _2qz * ay - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * az;
    s3 = 4.0f * qxqx * qz - _2qx * ax + 4.0f * qyqy * qz - _2qy * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }
  }
  // Integrate rate of change of quaternion to yield quaternion
  qw += qDot1 * dt;
  qx += qDot2 * dt;
  qy += qDot3 * dt;
  qz += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}
#else
// Madgwick's implementation of Mahony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * M_PI_F / 180;
  gy = gy * M_PI_F / 180;
  gz = gz * M_PI_F / 180;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (gravity_correction)
  {
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = qx * qz - qw * qy;
    halfvy = qw * qx + qy * qz;
    halfvz = qw * qw - 0.5f + qz * qz;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
}
  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = qw;
  qb = qx;
  qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz);
  qx += (qa * gx + qc * gz - qz * gy);
  qy += (qa * gy - qb * gz + qz * gx);
  qz += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}
#endif

void sensfusion6GetQuaternion(float* q_x, float* q_y, float* q_z, float* q_w)
{
  *q_x = qx;
  *q_y = qy;
  *q_z = qz;
  *q_w = qw;
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx = gravX;
  float gy = gravY;
  float gz = gravZ;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  *yaw = atan2f(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 180 / M_PI_F;
  *pitch = asinf(gx) * 180 / M_PI_F; //Pitch seems to be inverted
  *roll = atan2f(gy, gz) * 180 / M_PI_F;
}

float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

static float sensfusion6GetAccZ(const float ax, const float ay, const float az)
{
  // return vertical acceleration
  // (A dot G) / |G|,  (|G| = 1) -> (A dot G)
  return (ax * gravX + ay * gravY + az * gravZ);
}

static void estimatedGravityDirection(float* gx, float* gy, float* gz)
{
  *gx = 2 * (qx * qz - qw * qy);
  *gy = 2 * (qw * qx + qy * qz);
  *gz = qw * qw - qx * qx - qy * qy + qz * qz;
}

/**
 * Sensor fusion is the process of combining sensory data or data derived from
 * disparate sources such that the resulting information has less uncertainty
 * than would be possible when these sources were used individually.
 *
 * The sensfusion6 module uses an 3 axis accelerometer and a 3 axis gyro to get
 * accurate attitude measurements.
 */
LOG_GROUP_START(sensfusion6)
/**
 * @brief W quaternion
 */
  LOG_ADD(LOG_FLOAT, qw, &qw)
/**
 * @brief X quaternion
 */
  LOG_ADD(LOG_FLOAT, qx, &qx)
/**
 * @brief y quaternion
 */
  LOG_ADD(LOG_FLOAT, qy, &qy)
/**
 * @brief z quaternion
 */
  LOG_ADD(LOG_FLOAT, qz, &qz)
/**
 * @brief Gravity vector X
 */
  LOG_ADD(LOG_FLOAT, gravityX, &gravX)
/**
 * @brief Gravity vector Y
 */
  LOG_ADD(LOG_FLOAT, gravityY, &gravY)
/**
 * @brief Gravity vector Z
 */
  LOG_ADD(LOG_FLOAT, gravityZ, &gravZ)
/**
 * @brief Gravity scale factor after calibration
 */
  LOG_ADD(LOG_FLOAT, accZbase, &baseZacc)
/**
 * @brief Nonzero if complimentary filter been initialized
 */
  LOG_ADD(LOG_UINT8, isInit, &isInit)
/**
 * @brief Nonzero if gravity scale been calibrated
 */
  LOG_ADD(LOG_UINT8, isCalibrated, &isCalibrated)
LOG_GROUP_STOP(sensfusion6)

/**
 * Sensor fusion is the process of combining sensory data or data derived from
 * disparate sources such that the resulting information has less uncertainty
 * than would be possible when these sources were used individually.
 *
 * The sensfusion6 module uses an 3 axis accelerometer and a 3 axis gyro to get
 * accurate attitude measurements.
 */
PARAM_GROUP_START(sensfusion6)
#ifdef CONFIG_IMU_MADGWICK_QUATERNION
PARAM_ADD(PARAM_FLOAT, beta, &beta)
#else // MAHONY_QUATERNION_IMU
/**
 * @brief Integral gain (default: 0.002)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp, &twoKp)

/**
 * @brief Propotional gain (default: 0.8)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki, &twoKi)
#endif
PARAM_ADD(PARAM_FLOAT, baseZacc, &baseZacc)
PARAM_ADD(PARAM_UINT8, gc, &gravity_correction)
PARAM_GROUP_STOP(sensfusion6)

/**
 * Compass yaw correction (ArduPilot DCM style), see magYawUpdate().
 * World frame ENU, headings clockwise from true north.
 */
PARAM_GROUP_START(magYaw)
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, enable, &magYawEnable)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, decl, &magDecl)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, kp, &magKp)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magMin, &magMin)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magMax, &magMax)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, angRef, &magAngRef)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, angTol, &magAngTol)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, gate, &magGate)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, gateT, &magGateT)
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, initDelay, &magInitDelay)
PARAM_GROUP_STOP(magYaw)

LOG_GROUP_START(magYaw)
LOG_ADD(LOG_FLOAT, headMag, &logHeadMag)
LOG_ADD(LOG_FLOAT, headEst, &logHeadEst)
LOG_ADD(LOG_FLOAT, headGyro, &logHeadGyro)
LOG_ADD(LOG_FLOAT, innov, &logInnov)
LOG_ADD(LOG_FLOAT, norm, &logNorm)
LOG_ADD(LOG_FLOAT, angle, &logAngle)
LOG_ADD(LOG_FLOAT, corr, &logCorr)
LOG_ADD(LOG_UINT8, status, &magYawStatus)
LOG_ADD(LOG_UINT8, init, &haveInitialYaw)
LOG_ADD(LOG_UINT32, nAccept, &nAccept)
LOG_ADD(LOG_UINT32, nRejNorm, &nRejNorm)
LOG_ADD(LOG_UINT32, nRejAngle, &nRejAngle)
LOG_ADD(LOG_UINT32, nRejInnov, &nRejInnov)
LOG_ADD(LOG_UINT32, nTimeout, &nTimeout)
LOG_GROUP_STOP(magYaw)
