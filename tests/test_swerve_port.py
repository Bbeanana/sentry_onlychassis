"""Run the ported solver and the application's real kinematics on the host."""

from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Skywalker/Application/Src/app_robot.c").read_text(encoding="utf-8")


def function(name):
    match = re.search(
        r"^static [^\n]*\b" + name + r"\([^{}]*\) \{.*?^\}",
        source, re.M | re.S,
    )
    if not match:
        raise RuntimeError(f"Cannot locate {name}")
    return match.group()


def constant(name):
    match = re.search(r"^static const float " + name + r"\b.*?;", source, re.M | re.S)
    if not match:
        raise RuntimeError(f"Cannot locate {name}")
    return match.group()


constants = "\n".join(constant(name) for name in (
    "kRobotChassisHalfLengthM", "kRobotChassisHalfWidthM", "kRobotWheelRadiusM",
    "kRobotDriveReductionRatio", "kRobotChassisTranslationDeadband",
    "kRobotChassisTranslationStartDeadband", "kRobotSteeringForwardAngleRad",
    "kRobotSteeringDirection", "kRobotDriveDirection",
    "kRobotMaxVxMps",
    "kRobotMaxVyMps", "kRobotMaxOmegaRadps", "kRobotChassisMaxWheelSpeedMps",
    "kRobotSteeringCosineDeadbandRad", "kRobotChassisWheelMaxOmegaRadps",
    "kRobotChassisYawRateFeedbackKp", "kRobotChassisAutoSpinOmegaRadps",
    "kRobotChassisYawWheelDeadband", "kRobotSteeringVectorHoldSpeedMps",
    "kRobotSteeringVectorResumeSpeedMps",
    "kRobotChassisSpinYawCompensationS", "kRobotChassisSpinLateralScale",
    "kRobotChassisSpinForwardScale", "kRobotChassisSpinCommandEpsilon",
))
functions = "\n".join(function(name) for name in (
    "RobotClamp", "RobotRcNormalize", "RobotApplyDeadband", "RobotAutoSpinIsEnabled",
    "RobotWrapPi", "RobotChassisStickActive", "RobotSteeringTargetAngle",
    "RobotSteeringVectorActive", "RobotChassisYawRateCommand",
    "RobotChassisRemoteVelocity", "RobotChassisSpinTranslation", "RobotChassisKinematics",
))

harness = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "alg_swerve.h"
#include "dev_dji_motor.h"
#include "app_host_protocol.h"
#include "app_gimbal_link.h"
#ifndef TEST_REAL_GIMBAL_LINK
static float test_gimbal_yaw;
static bool test_gimbal_valid = true;
bool GimbalLinkGetYaw(GimbalYawFeedback *feedback) {
  *feedback = (GimbalYawFeedback){.relative_yaw_rad = test_gimbal_yaw,
                                 .valid = test_gimbal_valid};
  return test_gimbal_valid;
}
#endif
enum { kRobotChassisMotorCount = 4U, kRobotSteeringIdleOverridesYaw = 1U,
       kRobotChassisSpinTranslationEnabled = 1U,
       kRobotChassisStandaloneMode = 0U };
static struct {
  struct { int16_t ch2, ch3, wheel; uint8_t sw1; } rocker;
  struct { bool w, a, s, d, q, e; } key;
} g_dt7_object;
static bool SwitchIsUp(uint8_t value) { return value == 1U; }
typedef struct { float vx, vy, omega_z; } RobotVelocity;
static struct {
  DjiMotor chassis_motor_3508[4], chassis_motor_6020[4];
  bool chassis_steering_vector_active[4];
  float chassis_steering_requested_angle[4];
  struct {
    RobotVelocity desired_velocity, actual_velocity;
    bool translation_stick_active, steering_idle, imu_ready;
    float target_yaw_angle, yaw_angle, yaw_rate;
    bool spin_translation_active;
    bool spin_drive_tracking;
    float spin_translation_reference_yaw;
    float spin_translation_relative_yaw;
  } chassis;
} g_robot;
""" + constants + "\n" + functions + r"""

static void near(float actual, float expected, float tolerance) {
  assert(fabsf(actual - expected) < tolerance);
}

static void reset_robot(void) {
#ifndef TEST_REAL_GIMBAL_LINK
  test_gimbal_yaw = 0;
  test_gimbal_valid = true;
#endif
  memset(&g_robot, 0, sizeof(g_robot));
  memset(&g_dt7_object, 0, sizeof(g_dt7_object));
  g_dt7_object.rocker.sw1 = 2U;
  for (int i = 0; i < 4; ++i) {
    g_robot.chassis_motor_6020[i].state.angle = kRobotSteeringForwardAngleRad[i];
  }
}

static void test_vectors(void) {
  SwerveWheelVector wheels[4];
  SwerveResolve(2.0f, 0.0f, 0.0f, 0.2f, 0.3f, wheels);
  for (int i = 0; i < 4; ++i) {
    near(wheels[i].speed_mps, 2.0f, 0.00001f);
    near(wheels[i].angle_from_forward, 0.0f, 0.00001f);
  }
  SwerveResolve(0.0f, 2.0f, 0.0f, 0.2f, 0.3f, wheels);
  for (int i = 0; i < 4; ++i) {
    near(wheels[i].angle_from_forward, (float)M_PI / 2.0f, 0.00001f);
  }
  /* A non-square chassis checks wheel ordering and both lever arms. */
  const float x[4] = {0.3f, 0.3f, -0.3f, -0.3f};
  const float y[4] = {0.2f, -0.2f, 0.2f, -0.2f};
  for (int sign = -1; sign <= 1; sign += 2) {
    SwerveResolve(0.0f, 0.0f, (float)sign, 0.2f, 0.3f, wheels);
    for (int i = 0; i < 4; ++i) {
      /* Pure rotation must be tangential, with the requested yaw sign. */
      near(x[i] * wheels[i].vx + y[i] * wheels[i].vy, 0.0f, 0.00001f);
      near(x[i] * wheels[i].vy - y[i] * wheels[i].vx,
           (float)sign * 0.13f, 0.00001f);
    }
  }
  /* Independently check a mixed command and angle origin on all four wheels. */
  SwerveResolve(1.2f, -0.7f, 2.0f, 0.2f, 0.3f, wheels);
  const float expected_vx[4] = {0.8f, 1.6f, 0.8f, 1.6f};
  const float expected_vy[4] = {-0.1f, -0.1f, -1.3f, -1.3f};
  for (int i = 0; i < 4; ++i) {
    near(wheels[i].vx, expected_vx[i], 0.00001f);
    near(wheels[i].vy, expected_vy[i], 0.00001f);
    near(cosf(wheels[i].angle_from_forward) * wheels[i].speed_mps,
         expected_vx[i], 0.00001f);
    near(sinf(wheels[i].angle_from_forward) * wheels[i].speed_mps,
         expected_vy[i], 0.00001f);
  }
  /* One wheel can cancel to zero while the other three keep moving. */
  SwerveResolve(0.2f, -0.3f, 1.0f, 0.2f, 0.3f, wheels);
  near(wheels[0].speed_mps, 0.0f, 0.00001f);
  assert(wheels[1].speed_mps > 0.0f && wheels[2].speed_mps > 0.0f &&
         wheels[3].speed_mps > 0.0f);
}

static void test_optimization(void) {
  SwerveWheelTarget target = SwerveOptimizeWheel((float)M_PI, 0.0f, 2.0f, 0.0f);
  near(target.error_rad, 0.0f, 0.00001f);
  near(target.drive_speed_mps, -2.0f, 0.00001f);
  target = SwerveOptimizeWheel(0.0f, 0.0f, 2.0f, 0.0f);
  near(target.drive_speed_mps, 2.0f, 0.00001f);
  target = SwerveOptimizeWheel((float)M_PI / 3.0f, 0.0f, 2.0f, 0.0f);
  near(target.cosine_scale, 0.125f, 0.00001f);
  near(target.drive_speed_mps, 0.25f, 0.00001f);
  target = SwerveOptimizeWheel((float)M_PI / 2.0f, 0.0f, 2.0f, 0.0f);
  near(target.drive_speed_mps, 0.0f, 0.00001f);
  near(target.direction, 1.0f, 0.00001f);
  target = SwerveOptimizeWheel(-(float)M_PI / 2.0f, 0.0f, 2.0f, 0.0f);
  near(target.direction, -1.0f, 0.00001f);
  target = SwerveOptimizeWheel(0.03f, 0.0f, 2.0f, 0.05f);
  near(target.cosine_scale, 1.0f, 0.00001f);
  near(target.error_rad, 0.03f, 0.00001f);
  target = SwerveOptimizeWheel(0.03f, 0.0f, 2.0f, 0.0f);
  assert(target.cosine_scale < 1.0f);
  const float turn = 2.0f * (float)M_PI;
  target = SwerveOptimizeWheel(1.0f / 8192.0f * turn,
                               8191.0f / 8192.0f * turn, 1.0f, 0.0f);
  near(target.error_rad, 2.0f / 8192.0f * turn, 0.00001f);

  /* Every encoder heading must preserve the requested rolling vector after
   * a possible half-turn and drive reversal; error must never exceed 90 deg.
   */
  for (int desired = -180; desired <= 180; desired += 7) {
    for (int measured = 0; measured < 360; measured += 11) {
      float angle = (float)desired * (float)M_PI / 180.0f;
      float feedback = (float)measured * (float)M_PI / 180.0f;
      target = SwerveOptimizeWheel(angle, feedback, 1.0f, 0.0f);
      assert(fabsf(target.error_rad) <= (float)M_PI / 2.0f + 0.00001f);
      near(target.direction * cosf(target.angle_rad), cosf(angle), 0.00001f);
      near(target.direction * sinf(target.angle_rad), sinf(angle), 0.00001f);
    }
  }
  float speeds[4] = {3.0f, -4.0f, 6.0f, -8.0f};
  near(SwerveLimitWheelSpeeds(speeds, 4.0f), 0.5f, 0.00001f);
  const float limited[4] = {1.5f, -2.0f, 3.0f, -4.0f};
  for (int i = 0; i < 4; ++i) near(speeds[i], limited[i], 0.00001f);
}

static void assert_motor_vectors(float vx, float vy, float omega) {
  SwerveWheelVector wheels[4];
  SwerveResolve(vx, vy, omega, kRobotChassisHalfWidthM,
                 kRobotChassisHalfLengthM, wheels);
  for (int pass = 0; pass < 2; ++pass) {
    RobotChassisKinematics(NULL);
    for (int i = 0; i < 4; ++i) {
      g_robot.chassis_motor_6020[i].state.angle =
          g_robot.chassis_steering_requested_angle[i];
    }
  }
  float largest = 0.0f;
  for (int i = 0; i < 4; ++i) largest = fmaxf(largest, wheels[i].speed_mps);
  float scale = fminf(1.0f, kRobotChassisMaxWheelSpeedMps / largest);
  for (int i = 0; i < 4; ++i) {
    float heading = RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                                  kRobotSteeringForwardAngleRad[i]) /
                    kRobotSteeringDirection[i];
    float speed = g_robot.chassis_motor_3508[i].state.given_omega *
                  kRobotWheelRadiusM / kRobotDriveReductionRatio /
                  kRobotDriveDirection[i];
    near(cosf(heading) * speed, wheels[i].vx * scale, 0.00002f);
    near(sinf(heading) * speed, wheels[i].vy * scale, 0.00002f);
    assert(fabsf(speed) <= kRobotChassisMaxWheelSpeedMps + 0.00001f);
  }
}

static void test_application(void) {
  /* Inverted GM6020: motor offsets must be the negative of body headings,
   * relative to each wheel's own forward encoder zero. */
  for (int i = 0; i < 4; ++i) {
    near(RobotWrapPi(RobotSteeringTargetAngle(i, 0.0f) -
                    kRobotSteeringForwardAngleRad[i]), 0.0f, 0.00001f);
    near(RobotWrapPi(RobotSteeringTargetAngle(i, (float)M_PI / 4.0f) -
                    kRobotSteeringForwardAngleRad[i]),
         -(float)M_PI / 4.0f, 0.00001f);
    near(RobotWrapPi(RobotSteeringTargetAngle(i, -(float)M_PI / 4.0f) -
                    kRobotSteeringForwardAngleRad[i]),
         (float)M_PI / 4.0f, 0.00001f);
  }
  reset_robot();
  g_dt7_object.rocker.ch3 = 660;
  assert_motor_vectors(5.0f, 0.0f, 0.0f);
  g_dt7_object.rocker.ch3 = -660;
  RobotChassisKinematics(NULL);
  for (int i = 0; i < 4; ++i) {
    near(RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                     kRobotSteeringForwardAngleRad[i]), 0.0f, 0.00001f);
    near(g_robot.chassis_motor_3508[i].state.given_omega,
         -kRobotDriveDirection[i] * 5.0f * kRobotDriveReductionRatio /
             kRobotWheelRadiusM, 0.0001f);
  }
  /* ch2<0 and A command left; ch2>0 and D command right. */
  reset_robot();
  g_dt7_object.rocker.ch2 = -660;
  assert_motor_vectors(0.0f, 5.0f, 0.0f);
  reset_robot();
  g_dt7_object.rocker.ch2 = 660;
  assert_motor_vectors(0.0f, -5.0f, 0.0f);
  reset_robot();
  g_dt7_object.key.a = true;
  assert_motor_vectors(0.0f, 5.0f, 0.0f);
  reset_robot();
  g_dt7_object.key.d = true;
  assert_motor_vectors(0.0f, -5.0f, 0.0f);
  reset_robot();
  g_dt7_object.key.w = true;
  assert_motor_vectors(5.0f, 0.0f, 0.0f);
  reset_robot();
  g_dt7_object.key.s = true;
  assert_motor_vectors(-5.0f, 0.0f, 0.0f);
  /* Each sign of pure yaw must remain tangential through the motor mapping. */
  reset_robot();
  g_dt7_object.rocker.wheel = 100;
  assert_motor_vectors(0.0f, 0.0f, RobotChassisYawRateCommand(100.0f / 660.0f));
  reset_robot();
  g_dt7_object.rocker.wheel = -100;
  assert_motor_vectors(0.0f, 0.0f, RobotChassisYawRateCommand(-100.0f / 660.0f));
  reset_robot();
  g_dt7_object.rocker.ch3 = 660;
  g_dt7_object.rocker.ch2 = -660;
  g_dt7_object.rocker.wheel = 100;
  // Invalid IMU: spin requests must not fall back to rotating body translation.
  assert_motor_vectors(0.0f, 0.0f, RobotChassisYawRateCommand(100.0f / 660.0f));
  near(g_robot.chassis.actual_velocity.vx, 0, 0.00001f);
  near(g_robot.chassis.actual_velocity.vy, 0, 0.00001f);
  /* Return to idle: no stale drive request or undefined zero-vector heading. */
  memset(&g_dt7_object, 0, sizeof(g_dt7_object));
  g_dt7_object.rocker.sw1 = 2U;
  RobotChassisKinematics(NULL);
  assert(g_robot.chassis.steering_idle);
  for (int i = 0; i < 4; ++i) {
    near(g_robot.chassis_motor_3508[i].state.given_omega, 0.0f, 0.00001f);
    near(g_robot.chassis_steering_requested_angle[i],
         g_robot.chassis_motor_6020[i].state.angle, 0.00001f);
  }
}

static void test_host_body_translation(void) {
  reset_robot();
  g_robot.chassis.imu_ready = true;
  g_robot.chassis.yaw_angle = 3;
  HostVelocityCommand command = {.wz = 1};
  RobotChassisKinematics(&command);
  // Body-frame UART7 samples must not acquire a second rotation at any
  // accumulated heading, including wraparound, either spin sign and diagonals.
  const float inputs[5][3] = {{0, 2, 0.4f}, {0, -2, 0.4f},
                            {2, 0, 0.3f}, {-2, 1, 0.4f}, {1, -2, 0.4f}};
  for (int sign = -1; sign <= 1; sign += 2) {
    command.wz = (float)sign;
    g_robot.chassis.yaw_rate = (float)sign * 2;
    float lead = g_robot.chassis.yaw_rate * kRobotChassisSpinYawCompensationS;
    for (int deg = -720; deg <= 720; deg += 13) {
      g_robot.chassis.yaw_angle = RobotWrapPi(3 + deg * (float)M_PI / 180);
      for (unsigned i = 0; i < 5; ++i) {
        command.vx = inputs[i][0]; command.vy = inputs[i][1];
        float body_vx = command.vx;
        float body_vy = command.vy;
        RobotChassisKinematics(&command);
        near(g_robot.chassis.actual_velocity.vx,
             (cosf(lead) * body_vx + sinf(lead) * body_vy) * inputs[i][2],
             0.00001f);
        near(g_robot.chassis.actual_velocity.vy,
             (-sinf(lead) * body_vx + cosf(lead) * body_vy) * inputs[i][2],
             0.00001f);
        near(g_robot.chassis.desired_velocity.vx, body_vx, 0.00001f);
        near(g_robot.chassis.desired_velocity.vy, body_vy, 0.00001f);
      }
      // Simulate a fixed reference direction sent by the host as changing
      // body-frame components. Inverse rotation at the sender is sufficient.
      float yaw = g_robot.chassis.yaw_angle - 3;
      float body_vx = cosf(yaw) + 2 * sinf(yaw);
      float body_vy = -sinf(yaw) + 2 * cosf(yaw);
      command.vx = body_vx;
      command.vy = body_vy;
      RobotChassisKinematics(&command);
      float predicted_yaw = yaw + lead;
      near(cosf(predicted_yaw) * g_robot.chassis.actual_velocity.vx -
           sinf(predicted_yaw) * g_robot.chassis.actual_velocity.vy, 0.4f,
           0.00001f);
      near(sinf(predicted_yaw) * g_robot.chassis.actual_velocity.vx +
           cosf(predicted_yaw) * g_robot.chassis.actual_velocity.vy, 0.8f,
           0.00001f);
    }
  }
}

static void test_spin_translation(void) {
  reset_robot();
  g_robot.chassis.imu_ready = true;
  g_robot.chassis.yaw_angle = 3.0f;
  float vx = 0.0f, vy = 0.0f;
  RobotChassisSpinTranslation(&vx, &vy, 1.0f, false);
  assert(g_robot.chassis.spin_translation_active);
  near(g_robot.chassis.spin_translation_reference_yaw, 3.0f, 0.00001f);

  // Reconstruct reference-frame translation for both spin directions across
  // multiple full turns and wrapped IMU headings. Include prediction of yaw.
  for (int sign = -1; sign <= 1; sign += 2) {
    g_robot.chassis.yaw_rate = (float)sign * 2.0f;
    for (int deg = -720; deg <= 720; deg += 13) {
      g_robot.chassis.yaw_angle = RobotWrapPi(3.0f + deg * (float)M_PI / 180.0f);
      test_gimbal_yaw = RobotWrapPi(3.0f - g_robot.chassis.yaw_angle);
      float delta = g_robot.chassis.yaw_angle - 3.0f +
                    g_robot.chassis.yaw_rate * 0.005f;
      const float inputs[4][3] = {{0, 2, 0.4f}, {0, -2, 0.4f},
                                {2, 0, 0.3f}, {-2, 1, 0.4f}};
      for (int i = 0; i < 4; ++i) {
        vx = inputs[i][0]; vy = inputs[i][1];
        RobotChassisSpinTranslation(&vx, &vy, (float)sign, false);
        near(cosf(delta) * vx - sinf(delta) * vy,
             inputs[i][0] * inputs[i][2], 0.00001f);
        near(sinf(delta) * vx + cosf(delta) * vy,
             inputs[i][1] * inputs[i][2], 0.00001f);
        near(g_robot.chassis.spin_translation_reference_yaw, 3.0f, 0.00001f);
      }
    }
  }
  // A zero yaw request restores full-speed body translation, even if feedback
  // still reports rotation. A new spin must latch a new reference heading.
  test_gimbal_yaw = 0;
  vx = 1; vy = 2;
  RobotChassisSpinTranslation(&vx, &vy, 0.0f, false);
  assert(!g_robot.chassis.spin_translation_active);
  near(vx, 1, 0.00001f); near(vy, 2, 0.00001f);
  g_robot.chassis.yaw_angle = -1.0f;
  g_robot.chassis.yaw_rate = 0;
  RobotChassisSpinTranslation(&vx, &vy, -1.0f, false);
  near(g_robot.chassis.spin_translation_reference_yaw, -1, 0.00001f);
  near(vx, 0.4f, 0.00001f); near(vy, 0.8f, 0.00001f);
  g_robot.chassis.imu_ready = false;
  vx = 1; vy = 2;
  RobotChassisSpinTranslation(&vx, &vy, -1.0f, false);
  assert(!g_robot.chassis.spin_translation_active);
  near(vx, 0, 0.00001f); near(vy, 0, 0.00001f);
  // Straight translation is still available without IMU.
  test_gimbal_yaw = 0;
  vx = 1; vy = 2;
  RobotChassisSpinTranslation(&vx, &vy, 0.0f, false);
  near(vx, 1, 0.00001f); near(vy, 2, 0.00001f);

  // Real remote -> transform -> inverse kinematics -> inverted GM6020 and
  // M3508 targets: hold a forward reference while the body turns 90 degrees.
  reset_robot();
  g_robot.chassis.imu_ready = true;
  g_dt7_object.rocker.ch3 = 660;
  g_dt7_object.rocker.wheel = -100;
  float request = RobotChassisYawRateCommand(-100.0f / 660.0f);
  assert_motor_vectors(1.5f, 0, request * 1.5f);
  g_robot.chassis.yaw_angle = (float)M_PI / 2.0f;
  test_gimbal_yaw = -(float)M_PI / 2.0f;
  assert_motor_vectors(0, -1.5f, request * 1.5f);
  g_dt7_object.rocker.ch2 = -660;
  assert_motor_vectors(2, -2, request * 1.5f);
  g_dt7_object.rocker.wheel = 0;
  assert_motor_vectors(5, -5, 0);
  assert(!g_robot.chassis.spin_translation_active);
  g_dt7_object.rocker.wheel = -100;
  RobotChassisKinematics(NULL);
  near(g_robot.chassis.spin_translation_reference_yaw, (float)M_PI / 2.0f, 0.00001f);
  g_dt7_object.rocker.ch2 = g_dt7_object.rocker.ch3 = 0;
  g_dt7_object.rocker.wheel = 0;
  RobotChassisKinematics(NULL);
  assert(!g_robot.chassis.spin_translation_active);
}

int main(void) {
  test_vectors();
  test_optimization();
  test_application();
  test_spin_translation();
  test_host_body_translation();
  puts("PASS: swerve vector invariants, shortest angle, cos^3, wrap, speed limit and application mapping");
  return 0;
}
"""

if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="swerve_port_") as directory:
        test_c = Path(directory) / "test.c"
        test_exe = Path(directory) / "test.exe"
        test_c.write_text(harness, encoding="utf-8")
        subprocess.run([
            "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
            "-I", str(ROOT / "Skywalker/Device/Inc"),
            "-I", str(ROOT / "Skywalker/Application/Inc"),
            str(test_c), str(ROOT / "Skywalker/Algorithm/Src/alg_swerve.c"),
            "-lm", "-o", str(test_exe),
        ], check=True)
        subprocess.run([str(test_exe)], check=True)
