/*
 * Adapted from Qiqi team's chassis_task.c:
 * chassic_rudder_preliminary_A_S_solution,
 * Rudder_motor_relative_angle_control and chassis_speed_control_set.
 * Copyright (c) 2026 Rudder-Swerve-Control contributors
 * SPDX-License-Identifier: MIT
 * See docs/licenses/qiqi-swerve-MIT.txt for the full original license.
 */
#include "alg_swerve.h"

#include <math.h>

static const float kSwervePi = 3.14159265358979323846f;

static float SwerveWrapPi(float angle) {
  while (angle > kSwervePi) {
    angle -= 2.0f * kSwervePi;
  }
  while (angle < -kSwervePi) {
    angle += 2.0f * kSwervePi;
  }
  return angle;
}

void SwerveResolve(float vx, float vy, float omega, float half_width_m,
                   float half_length_m,
                   SwerveWheelVector wheels[kSwerveWheelCount]) {
  /* Same translation + rotational-component method as the source.
   * Keep target geometry and SI units instead of its implicit 45-degree gain.
   */
  /* FL=(+L,+W), FR=(+L,-W), BL=(-L,+W), BR=(-L,-W).
   * Each wheel sees v + omega cross r: (vx - omega*y, vy + omega*x).
   */
  const float rotational_x = omega * half_width_m;
  const float rotational_y = omega * half_length_m;
  const float wheel_vx[kSwerveWheelCount] = {
      vx - rotational_x, vx + rotational_x,
      vx - rotational_x, vx + rotational_x};
  const float wheel_vy[kSwerveWheelCount] = {
      vy + rotational_y, vy + rotational_y,
      vy - rotational_y, vy - rotational_y};

  for (int i = 0; i < kSwerveWheelCount; ++i) {
    wheels[i].vx = wheel_vx[i];
    wheels[i].vy = wheel_vy[i];
    wheels[i].speed_mps =
        sqrtf(wheel_vx[i] * wheel_vx[i] + wheel_vy[i] * wheel_vy[i]);
    /* A stationary vector has no heading; the application holds its target. */
    wheels[i].angle_from_forward = wheels[i].speed_mps > 0.0f
        ? atan2f(wheel_vy[i], wheel_vx[i])
        : 0.0f;
  }
}

SwerveWheelTarget SwerveOptimizeWheel(float desired_angle_rad,
                                     float measured_angle_rad,
                                     float speed_mps,
                                     float cosine_deadband_rad) {
  SwerveWheelTarget target = {0};
  const float half_pi = kSwervePi * 0.5f;
  float error = SwerveWrapPi(desired_angle_rad - measured_angle_rad);
  target.direction = 1.0f;

  /* Source boundary convention: +90 stays forward; -90 reverses. */
  if (error > half_pi) {
    error -= kSwervePi;
    target.direction = -1.0f;
  } else if (error <= -half_pi) {
    error += kSwervePi;
    target.direction = -1.0f;
  }

  target.error_rad = error;
  target.angle_rad = SwerveWrapPi(measured_angle_rad + error);
  /* The deadband only affects speed compensation, never the position error. */
  float compensation_error = fabsf(error);
  if (compensation_error < cosine_deadband_rad) {
    compensation_error = 0.0f;
  }
  const float cosine = fmaxf(cosf(compensation_error), 0.0f);
  target.cosine_scale = cosine * cosine * cosine;
  target.drive_speed_mps = speed_mps * target.direction * target.cosine_scale;
  return target;
}

float SwerveLimitWheelSpeeds(float speeds[kSwerveWheelCount],
                            float max_speed_mps) {
  float maximum = 0.0f;
  for (int i = 0; i < kSwerveWheelCount; ++i) {
    maximum = fmaxf(maximum, fabsf(speeds[i]));
  }
  const float limit = fmaxf(max_speed_mps, 0.0f);
  const float scale = maximum > limit ? limit / maximum : 1.0f;
  for (int i = 0; i < kSwerveWheelCount; ++i) {
    speeds[i] *= scale;
  }
  return scale;
}
