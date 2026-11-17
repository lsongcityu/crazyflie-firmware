/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */


#include "power_distribution.h"

#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
    #error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static float armLength = ARM_LENGTH; // m
static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
//    where PWM is normalized (range 0...1)
//          thrust is in Newtons (per rotor)
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;
static uint16_t min_thrust = 3000;

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
    DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
  }
  #endif
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

#ifndef CONFIG_MOTORS_REVERSIBLE
static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}
#endif

#ifdef CONFIG_MOTORS_REVERSIBLE
// [hopping] Reversible motors (DSHOT 3D mode). Local addition, not in upstream.
//
// This follows the arithmetic of the bidirectional_hopping branch (OneShot 3D):
// all values are distances from the thrust midpoint 32750, in thrust setpoint
// units, where 32750 would be the full range of one direction.
//
//   motor = (thrust - 32750) + min_thrust + attitude correction
//
// control->thrust is signed here: 0 = motors stopped, positive = forward,
// negative = reverse. All four motors always share one direction.
//
// The only new part is the last step. A OneShot 3D ESC takes 32750 +- motor.
// DSHOT 3D mode instead has two halves of 1000 steps: 1048..2047 is the normal
// direction and 48..1047 the reversed one, slowest first.
#define DSHOT_3D_FORWARD_MIN  1048
#define DSHOT_3D_REVERSE_MIN  48
#define DSHOT_3D_STEPS        999

// Thrust convention of the reversible build, used for the radio thrust setpoint
// and for the motorPowerSet override.
#define REVERSIBLE_THRUST_MID         32750.0f
#define REVERSIBLE_THRUST_DEADBAND    10.0f
// The radio commander limits the thrust setpoint to 60000 and treats values
// below 1000 as stop. Every motor is kept between the minimum spin and this
// distance from the midpoint, in both directions (60000 forward, 5500 reverse).
#define REVERSIBLE_THRUST_MAX_OFFSET  (60000.0f - REVERSIBLE_THRUST_MID)
#define REVERSIBLE_THRUST_STOP_BELOW  1000

// Set to 1 if forward and reverse turn out to be swapped on the robot
static uint8_t swapDirection3d = 0;

static void powerDistributionLegacyReversible(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  const int32_t thrust = (int32_t)control->thrust;

  if (thrust == 0)
  {
    for (int mi = 0; mi < STABILIZER_NR_OF_MOTORS; mi++)
    {
      motorThrustUncapped->list[mi] = 0;
    }
    return;
  }

  // A reversed propeller flips both its thrust and its yaw torque, so in the
  // reversed direction the attitude terms change sign relative to the motor speed.
  const int32_t direction = (thrust > 0) ? 1 : -1;

  int32_t att[4];
  att[0] = direction * (-control->roll + control->pitch + control->yaw);
  att[1] = direction * (-control->roll - control->pitch - control->yaw);
  att[2] = direction * (+control->roll - control->pitch + control->yaw);
  att[3] = direction * (+control->roll + control->pitch - control->yaw);

  int32_t min = att[0];
  for (int mi = 1; mi < 4; mi++)
  {
    if (att[mi] < min)
      min = att[mi];
  }

  // Keep every motor on the same side of the midpoint, then add the minimum spin
  int32_t speed = direction * thrust;
  if (speed < -min)
    speed = -min;
  speed = speed + min_thrust;

  for (int mi = 0; mi < 4; mi++)
  {
    motorThrustUncapped->list[mi] = direction * (att[mi] + speed);
  }
}

// Convert a signed distance from the midpoint to the 16 bit ratio that the
// DSHOT driver turns into a 3D mode throttle value (the driver sends ratio >> 5).
static uint16_t reversibleEncodeRatio(int32_t value)
{
  if (value == 0)
  {
    return 0; // DSHOT 0 = motor stop
  }

  const bool forward = (value > 0) != (swapDirection3d != 0);
  const int32_t speed = (value > 0) ? value : -value;

  // 32750 is the full range of one direction, DSHOT 3D has 1000 steps for it
  int32_t steps = speed >> 5;
  if (steps > DSHOT_3D_STEPS)
    steps = DSHOT_3D_STEPS;

  return (uint16_t)(((forward ? DSHOT_3D_FORWARD_MIN : DSHOT_3D_REVERSE_MIN) + steps) << 5);
}

float powerDistributionReversibleThrust(float setpointThrust)
{
  float offset = setpointThrust - REVERSIBLE_THRUST_MID;
  if (fabsf(offset) <= REVERSIBLE_THRUST_DEADBAND)
  {
    return 1.0f; // smallest forward value: the motors turn at the minimum spin
  }

  if (offset > REVERSIBLE_THRUST_MAX_OFFSET)
    offset = REVERSIBLE_THRUST_MAX_OFFSET;
  if (offset < -REVERSIBLE_THRUST_MAX_OFFSET)
    offset = -REVERSIBLE_THRUST_MAX_OFFSET;

  return offset;
}

uint16_t powerDistributionReversibleRatio(uint16_t value)
{
  if (value < REVERSIBLE_THRUST_STOP_BELOW)
  {
    return 0; // motor stop
  }

  // Unlike the thrust setpoint, the override has no minimum spin: 32750 is
  // "motor stopped", like the midpoint of a OneShot 3D ESC, and the speed
  // grows from the ESC minimum on either side.
  float offset = (float)value - REVERSIBLE_THRUST_MID;
  if (fabsf(offset) <= REVERSIBLE_THRUST_DEADBAND)
  {
    return 0; // motor stop
  }

  if (offset > REVERSIBLE_THRUST_MAX_OFFSET)
    offset = REVERSIBLE_THRUST_MAX_OFFSET;
  if (offset < -REVERSIBLE_THRUST_MAX_OFFSET)
    offset = -REVERSIBLE_THRUST_MAX_OFFSET;

  return reversibleEncodeRatio((int32_t)offset);
}

#else // CONFIG_MOTORS_REVERSIBLE

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  // int16_t r = control->roll / 2.0f;
  // int16_t p = control->pitch / 2.0f;

  int32_t att[4];
  att[0] = -control->roll + control->pitch + control->yaw;
  att[1] = -control->roll - control->pitch - control->yaw;
  att[2] = +control->roll - control->pitch + control->yaw;
  att[3] = +control->roll + control->pitch - control->yaw;

  // for x-config quadcopter
  int32_t min = att[0];
  for (int mi = 1; mi < 4; mi++)
  {
    if (att[mi] < min)
      min = att[mi];
  }
  int32_t thrust;
  if (control->thrust < -min)
    thrust = -min;
  else
    thrust = (uint32_t)control->thrust;

  if (thrust > 100)
    thrust = thrust + min_thrust;

  att[0] = att[0] + thrust;
  att[1] = att[1] + thrust;
  att[2] = att[2] + thrust;
  att[3] = att[3] + thrust;

  motorThrustUncapped->motors.m1 = limitUint16(att[0]);
  motorThrustUncapped->motors.m2 = limitUint16(att[1]);
  motorThrustUncapped->motors.m3 = limitUint16(att[2]);
  motorThrustUncapped->motors.m4 = limitUint16(att[3]);

  // motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  // motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  // motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  // motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}
#endif // CONFIG_MOTORS_REVERSIBLE

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * armLength;
  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / thrustToTorque;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    float motor_pwm = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * motorForce)) / (2.0f * pwmToThrustA);
    motorThrustUncapped->list[motorIndex] = motor_pwm * UINT16_MAX;
  }
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  // Not implemented yet
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
#ifdef CONFIG_MOTORS_REVERSIBLE
      powerDistributionLegacyReversible(control, motorThrustUncapped);
#else
      powerDistributionLegacy(control, motorThrustUncapped);
#endif
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = UINT16_MAX;
  bool isCapped = false;

#ifdef CONFIG_MOTORS_REVERSIBLE
  // [hopping] The values are signed distances from the midpoint. Keep every
  // motor between the minimum spin and the maximum distance (32750+3000..60000
  // forward, 32750-3000..5500 reverse). A common reduction is tried first, as in
  // the one-direction code below, so the attitude differences are kept when possible.
  // powerDist.idleThrust is not used.
  (void)maxAllowedThrust;
  const int32_t maxSpeed = (int32_t)REVERSIBLE_THRUST_MAX_OFFSET;
  const int32_t minSpeed = min_thrust;

  int32_t highestSpeedFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t value = motorThrustBatCompUncapped->list[motorIndex];
    const int32_t speed = (value > 0) ? value : -value;
    if (speed > highestSpeedFound)
    {
      highestSpeedFound = speed;
    }
  }

  int32_t speedReduction = 0;
  if (highestSpeedFound > maxSpeed)
  {
    speedReduction = highestSpeedFound - maxSpeed;
    isCapped = true;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t value = motorThrustBatCompUncapped->list[motorIndex];
    if (value == 0)
    {
      motorPwm->list[motorIndex] = 0; // motor stop
      continue;
    }

    int32_t speed = ((value > 0) ? value : -value) - speedReduction;
    if (speed < minSpeed)
    {
      speed = minSpeed; // never below the minimum spin, so the motor does not stall
    }
    if (speed > maxSpeed)
    {
      speed = maxSpeed;
    }
    motorPwm->list[motorIndex] = reversibleEncodeRatio((value > 0) ? speed : -speed);
  }

  return isCapped;
#else

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
    isCapped = true;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, powerDistributionGetIdleThrust());
  }

  return isCapped;
#endif // CONFIG_MOTORS_REVERSIBLE
}

uint32_t powerDistributionGetIdleThrust()
{
  int32_t thrust = idleThrust;
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
  #endif
  return thrust;
}

float powerDistributionGetMaxThrust() {
  // max thrust per rotor occurs if normalized PWM is 1
  // pwmToThrustA * pwm * pwm + pwmToThrustB * pwm = pwmToThrustA + pwmToThrustB
  return STABILIZER_NR_OF_MOTORS * (pwmToThrustA + pwmToThrustB);
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
#ifdef CONFIG_MOTORS_REVERSIBLE
/**
 * @brief [hopping] Reversible build only: set to 1 if forward and reverse thrust are swapped (default: 0)
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, swap3d, &swapDirection3d)
#endif
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
PARAM_GROUP_START(quadSysId)

PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_GROUP_STOP(quadSysId)
