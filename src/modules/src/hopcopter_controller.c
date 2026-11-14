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
/* Extracted without changing arithmetic or branch order from stabilizer.c.
 * kp_z_temp is intentionally retained although the baseline yaw law uses kp_z.
 */
#include <math.h>
#include "hopcopter_controller.h"
static float limint16(float in)
{
  if (in > 32000.0f)
    return 32000.0f;
  else if (in < -32000.0f)
    return -32000.0f;
  else
    return in;
}

static float lim_num(float in, float num)
{
  if (in > num)
    return num;
  else if (in < -num)
    return -num;
  else
    return in;
}

// Function to determine if two quaternions represent the same attitude
static bool same_attitude(float w, float x, float y, float z, float w_d, float x_d, float y_d, float z_d)
{
  // Calculate the dot product of the two quaternions
  float dot_product = w * w_d + x * x_d + y * y_d + z * z_d;
  // Check if the absolute value of the dot product is close to 1
  return fabsf(dot_product) > 0.999999f;
}

static void pcontrol(float w, float x, float y, float z, float w_d, float x_d,
  float y_d, float z_d, float *tau_x, float *tau_y, float *tau_z)
{

if (same_attitude(w, x, y, z, w_d, x_d, y_d, z_d))
{
*tau_x = 0.0F;
*tau_y = 0.0F;
*tau_z = 0.0F;
}
else
{
static float axang1;
static float axang2;
static float axang3;
static float axang4;
static float rot1;
static float temp_1;

temp_1 = ((w * w_d + x * x_d) + y * y_d) + z * z_d;

if (temp_1 == 1.0F)
{
/*  axang = [0, 0, 1, 0]; */
axang1 = 0.0F;
axang2 = 0.0F;
axang3 = 1.0F;
axang4 = 0.0F;
}
else
{

axang4 = sqrtf(1.0F - temp_1 * temp_1);
axang1 = (((w * x_d - w_d * x) + y * z_d) - y_d * z) / axang4;
axang2 = (((w * y_d - w_d * y) - x * z_d) + x_d * z) / axang4;
axang3 = (((w * z_d - w_d * z) + x * y_d) - x_d * y) / axang4;
axang4 = 2.0F * acosf(temp_1);
}

if (axang4 > 3.1415926535897931f)
{
axang4 = 6.28318548F - axang4;
axang1 = -axang1;
axang2 = -axang2;
axang3 = -axang3;
}
else if (axang4 < -3.1415926535897931f)
{
axang4 += 6.28318548F + axang4;
axang1 = -axang1;
axang2 = -axang2;
axang3 = -axang3;
}

rot1 = axang1 * axang4;
axang1 = axang2 * axang4;
temp_1 = axang3 * axang4;

axang4 = (rot1 * x + axang1 * y) + temp_1 * z;
axang2 = (rot1 * w - temp_1 * y) + axang1 * z;
axang3 = (axang1 * w + temp_1 * x) - rot1 * z;
temp_1 = (temp_1 * w - axang1 * x) + rot1 * y;

*tau_x = ((w * axang2 - y * temp_1) + x * axang4) + z * axang3;
*tau_y = ((w * axang3 + x * temp_1) + y * axang4) - z * axang2;
*tau_z = ((w * temp_1 - x * axang3) + y * axang2) + z * axang4;
}
}

static void eul2quat_my(float yaw, float pitch, float roll,
  float *w, float *x, float *y, float *z)
{
static float c1c2;
static float c_idx_0;
static float c_idx_1;
static float c_idx_2;
static float s1s2;
static float s_idx_0;
static float s_idx_1;
static float s_idx_2;
s_idx_0 = yaw / 2.0F;
s_idx_1 = pitch / 2.0F;
s_idx_2 = roll / 2.0F;
c_idx_0 = cosf(s_idx_0);
s_idx_0 = sinf(s_idx_0);
c_idx_1 = cosf(s_idx_1);
s_idx_1 = sinf(s_idx_1);
c_idx_2 = cosf(s_idx_2);
s_idx_2 = sinf(s_idx_2);
c1c2 = c_idx_0 * c_idx_1;
s1s2 = s_idx_0 * s_idx_1;
c_idx_0 *= s_idx_1;
s_idx_1 = s_idx_0 * c_idx_1;
*w = c1c2 * c_idx_2 + s1s2 * s_idx_2;
*x = c1c2 * s_idx_2 - s1s2 * c_idx_2;
*y = c_idx_0 * c_idx_2 + s_idx_1 * s_idx_2;
*z = s_idx_1 * c_idx_2 - c_idx_0 * s_idx_2;
}



void hopcopterControllerInit(hopcopterController_t *context) {
  *context = (hopcopterController_t)HOPCOPTER_CONTROLLER_INITIALIZER;
}
void hopcopterControllerStep(hopcopterController_t *context, control_t *control,
  const setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state) {
  float tau_omega_x = 0.0f, tau_omega_y = 0.0f, norm_tau_omega = 0.0f;
        // disable P controller when thrust is equal to context->attitude_control_limit
        if (fabsf(setpoint->thrust - context->attitude_control_limit) < 10.0f)
        {
          context->kp_xy_temp = 0.0f;
          context->kp_z_temp = 0.0f;
        }
        else
        {
          context->kp_xy_temp = context->kp_xy;
          context->kp_z_temp = context->kp_z;
        }

        if (context->timestamp_setpoint == setpoint->timestamp)
        {
          // no control input is received
          ;
        }
        else
        {
          // control input is received
          if (setpoint->timestamp > context->timestamp_setpoint)
            // time_gap_setpoint = setpoint->timestamp - context->timestamp_setpoint;

            context->timestamp_setpoint = setpoint->timestamp;

          context->qw_desired_delay = context->qw_desired;
          context->qx_desired_delay = context->qx_desired;
          context->qy_desired_delay = context->qy_desired;
          context->qz_desired_delay = context->qz_desired;

          // compute desired quat
          eul2quat_my(setpoint->attitudeRate.yaw * -0.0174532925199433f,
                      setpoint->attitude.pitch * -0.0174532925199433f,
                      setpoint->attitude.roll * 0.0174532925199433f,
                      &context->qw_desired,
                      &context->qx_desired,
                      &context->qy_desired,
                      &context->qz_desired);

          pcontrol(context->qw_desired_delay,
                   context->qx_desired_delay,
                   context->qy_desired_delay,
                   context->qz_desired_delay,
                   context->qw_desired,
                   context->qx_desired,
                   context->qy_desired,
                   context->qz_desired,
                   &context->omega_x, &context->omega_y, &context->omega_z);

          // // desired angular rate in degrees
          context->omega_x = context->omega_x * 57.2957795130823f * context->external_loop_freq;
          context->omega_y = context->omega_y * 57.2957795130823f * context->external_loop_freq;
          context->omega_z = context->omega_z * 57.2957795130823f * context->external_loop_freq;

          context->omega_x = lim_num(context->omega_x, 300);
          context->omega_y = lim_num(context->omega_y, 300);
          context->omega_z = lim_num(context->omega_z, 300);
        }

        if (fabsf(setpoint->thrust - context->idle_thrust) < 10.0f)
        {
          control->thrust = 500.0f;
          control->roll = 0.0f;
          control->pitch = 0.0f;
          control->yaw = 0.0f;
        }
        else if (setpoint->thrust >= 10.0f)
        {

          pcontrol(state->attitudeQuaternion.w,
                   state->attitudeQuaternion.x,
                   state->attitudeQuaternion.y,
                   state->attitudeQuaternion.z,
                   context->qw_desired,
                   context->qx_desired,
                   context->qy_desired,
                   context->qz_desired,
                   &context->tau_x, &context->tau_y, &context->tau_z);

          // float angle_error = sqrtf(context->tau_x * context->tau_x + context->tau_y * context->tau_y + context->tau_z * context->tau_z);

          // if (angle_error > angle_error_threshold)
          // {
          //   context->omega_x = (context->tau_x/angle_error) * angle_error_velocity;
          //   context->omega_y = (context->tau_y/angle_error) * angle_error_velocity;
          //   context->omega_z = (context->tau_z/angle_error) * angle_error_velocity;
          // }
          // else
          // {
          //   context->omega_x = 0.0f;
          //   context->omega_y = 0.0f;
          //   context->omega_z = 0.0f;
          // }

          context->tau_x = context->tau_x + context->tau_x_offset;
          context->tau_y = context->tau_y + context->tau_y_offset;
          context->tau_z = context->tau_z + context->tau_z_offset;

          tau_omega_x = context->omega_x - sensorData->gyro.x;
          tau_omega_y = context->omega_y - sensorData->gyro.y;
          norm_tau_omega = sqrtf(tau_omega_x * tau_omega_x + tau_omega_y * tau_omega_y);

          if (norm_tau_omega > context->norm_tau_omega_limit)
          {
            tau_omega_x = tau_omega_x / norm_tau_omega * context->norm_tau_omega_limit;
            tau_omega_y = tau_omega_y / norm_tau_omega * context->norm_tau_omega_limit;
          }

          control->thrust = setpoint->thrust;
          control->roll = (int16_t)limint16(context->tau_x * context->kp_xy_temp + tau_omega_x * context->kd_xy);
          control->pitch = -(int16_t)limint16(context->tau_y * context->kp_xy_temp + tau_omega_y * context->kd_xy);
          control->yaw = -(int16_t)limint16(context->tau_z * context->kp_z + (context->omega_z - sensorData->gyro.z) * context->kd_z);
          if (fabsf(context->flip_thrust - setpoint->thrust) < 2.0f)
          {
            if (state->attitude.roll > -30 && state->attitude.roll < context->switch_angle)
              control->roll = context->flip_roll;
            else
              control->roll = -context->flip_roll;

            control->thrust = 1000.0f;
            control->pitch = 0;
            control->yaw = 0;
          }
        }
        else
        {
          control->thrust = 0.0f;
          control->roll = 0.0f;
          control->pitch = 0.0f;
          control->yaw = 0.0f;
        }

}
