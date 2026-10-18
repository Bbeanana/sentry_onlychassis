#ifndef ALG_SWERVE_H
#define ALG_SWERVE_H

/*
 * Four-wheel port of Qiqi team's rudder shortest-path / cos^3 algorithm.
 * Copyright (c) 2026 Rudder-Swerve-Control contributors
 * Original license: docs/licenses/qiqi-swerve-MIT.txt
 */

enum { kSwerveWheelCount = 4 };

/* Index order: front left, front right, back left, back right.
 * Body frame: +X forward, +Y left, +omega counterclockwise.
 */
typedef struct {
  float vx;                 /* Wheel velocity along body X, m/s. */
  float vy;                 /* Wheel velocity along body Y, m/s. */
  float speed_mps;           /* Unsigned wheel ground speed. */
  float angle_from_forward; /* Counterclockwise from +X, rad. */
} SwerveWheelVector;

typedef struct {
  float angle_rad;          /* Optimized absolute motor angle, rad. */
  float error_rad;          /* Shortest motor error, within +/-pi/2. */
  float drive_speed_mps;     /* Signed wheel speed after cos^3 compensation. */
  float direction;           /* +1 or -1, before installation sign. */
  float cosine_scale;
} SwerveWheelTarget;

/* vx/vy: m/s; omega: rad/s; half width/length: metres. */
void SwerveResolve(float vx, float vy, float omega, float half_width_m,
                   float half_length_m,
                   SwerveWheelVector wheels[kSwerveWheelCount]);

/* Angles already include encoder zero and steering installation direction.
 * A zero cosine_deadband_rad keeps cos^3 compensation during rotation.
 */
SwerveWheelTarget SwerveOptimizeWheel(float desired_angle_rad,
                                     float measured_angle_rad,
                                     float speed_mps,
                                     float cosine_deadband_rad);

/* Scale all signed speeds by one factor, preserving their ratios. */
float SwerveLimitWheelSpeeds(float speeds[kSwerveWheelCount],
                            float max_speed_mps);

#endif
