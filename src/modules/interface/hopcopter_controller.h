/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
/* Shared Hopcopter control path extracted from stabilizer.c (c0791643).
 * No estimator, RTOS, hardware I/O or wall clock. Call once per 1 ms.
 */
#ifndef HOPCOPTER_CONTROLLER_H
#define HOPCOPTER_CONTROLLER_H
#include "stabilizer_types.h"
typedef struct {
  float idle_thrust;
  float attitude_control_limit;
  float kp_xy;
  float kp_xy_temp;
  float kp_z;
  float kp_z_temp;
  float norm_tau_omega_limit;
  float kd_xy;
  float kd_z;
  float tau_x_offset;
  float tau_y_offset;
  float tau_z_offset;
  float tau_x;
  float tau_y;
  float tau_z;
  float omega_x;
  float omega_y;
  float omega_z;
  float qw_desired;
  float qx_desired;
  float qy_desired;
  float qz_desired;
  float qw_desired_delay;
  float qx_desired_delay;
  float qy_desired_delay;
  float qz_desired_delay;
  uint32_t timestamp_setpoint;
  float external_loop_freq;
  int16_t flip_roll;
  float flip_thrust;
  float switch_angle;
} hopcopterController_t;
#define HOPCOPTER_CONTROLLER_INITIALIZER { \
  .idle_thrust = 1500.0f, \
  .attitude_control_limit = 1300.0f, \
  .kp_xy = 6000, \
  .kp_xy_temp = 6000, \
  .kp_z = 6000, \
  .kp_z_temp = 6000, \
  .norm_tau_omega_limit = 100.0f, \
  .kd_xy = 10, \
  .kd_z = 10, \
  .tau_x_offset = 0.0f, \
  .tau_y_offset = 0.0f, \
  .tau_z_offset = 0.0f, \
  .tau_x = 0.0f, \
  .tau_y = 0.0f, \
  .tau_z = 0.0f, \
  .omega_x = 0.0f, \
  .omega_y = 0.0f, \
  .omega_z = 0.0f, \
  .qw_desired = 1.0f, \
  .qx_desired = 0.0f, \
  .qy_desired = 0.0f, \
  .qz_desired = 0.0f, \
  .qw_desired_delay = 1.0f, \
  .qx_desired_delay = 0.0f, \
  .qy_desired_delay = 0.0f, \
  .qz_desired_delay = 0.0f, \
  .timestamp_setpoint = 0, \
  .external_loop_freq = 100.0f, \
  .flip_roll = 10000, \
  .flip_thrust = 1380, \
  .switch_angle = 130, \
}
void hopcopterControllerInit(hopcopterController_t *context);
void hopcopterControllerStep(hopcopterController_t *context, control_t *control,
  const setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state);
#endif
