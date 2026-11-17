/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * power_distribution.h - Interface to stabilizer power distribution
 */
#ifndef __POWER_DISTRIBUTION_H__
#define __POWER_DISTRIBUTION_H__

#include "stabilizer_types.h"


void powerDistributionInit(void);
bool powerDistributionTest(void);

/**
 * @brief Calculate the power (thrust) of each motor based on the output from the controller
 *
 * @param control Data from the controller
 * @param motorThrustUncapped The desired thrust
 */
void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped);

/**
 * @brief Cap the thrust for the motors when out side of the valid range [0 - UINT16_MAX]. The platform specific
 * implementation can chose to cap the trust in a way that provides graceful degradation, for instance prioritizing
 * attitude over thrust.
 *
 * @param motorThrustBatCompUncapped The desired thrust for the motors
 * @param motorPwm The capped thrust
 * @return true   Thrust was capped
 * @return false  Thrust was unchanged and not capped
 */
bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm);

/**
 * Returns a 1 when motor 'id' gives thrust, returns 0 otherwise
 */
int powerDistributionMotorType(uint32_t id);

/**
 * Returns the stop ratio of the motor 'id'
 */
uint16_t powerDistributionStopRatio(uint32_t id);

/**
 * @brief Get the current setting for idle thrust
 *
 * @return uint32_t The idle thrust
 */
uint32_t powerDistributionGetIdleThrust();

/**
 * @brief Compute the maximum thrust
 *
 * @return float The maximum thrust of the robot [N]
 */
float powerDistributionGetMaxThrust();

#ifdef CONFIG_MOTORS_REVERSIBLE
// [hopping] Reversible motors (DSHOT 3D mode). Local addition, not in upstream.

/**
 * @brief Decode a thrust value that uses the reversible convention.
 *
 * 32750 = zero thrust (motors turn forward at minimum speed), above = forward,
 * below = reverse, growing with the distance from 32750. Forward and reverse
 * are limited to the same size (60000 on the forward side, 5500 on the reverse side).
 *
 * @param setpointThrust Thrust in the reversible convention. Must not be the "stop" value 0.
 * @return Signed distance from 32750, positive = forward (32750 would be the
 *         full range of one direction). Never 0.
 */
float powerDistributionReversibleThrust(float setpointThrust);

/**
 * @brief Convert a motor value in the reversible convention straight to the
 * ratio for the DSHOT driver. Used for the motorPowerSet override.
 *
 * Unlike the thrust setpoint there is no minimum spin: below 1000 and
 * 32750 +-10 are "motor stopped", above = forward, below = reverse.
 *
 * @param value Motor value in the reversible convention
 * @return Ratio for motorsSetRatio()
 */
uint16_t powerDistributionReversibleRatio(uint16_t value);
#endif

#endif //__POWER_DISTRIBUTION_H__
