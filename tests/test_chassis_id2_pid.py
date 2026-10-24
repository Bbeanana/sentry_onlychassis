"""Exercise all four chassis motor groups and the steering interlock."""

from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Skywalker/Application/Src/app_robot.c").read_text(encoding="utf-8")
assert "RobotSteeringSlidingCalculate" not in source
assert "kRobotGimbalEnabled = 0U" in source
assert "kRobotShootEnabled = 0U" in source
assert "kRobotChassisMotorCount = 4U" in source
assert "kRobotSteeringTargetRateMaxRadps = 4.0f" in source
assert "SwerveResolve(vx, vy, wz" in source
assert "SwerveOptimizeWheel(" in source
assert "SwerveLimitWheelSpeeds(drive_speeds" in source
for encoder in (3060, 2421, 2383, 3097):
    assert f"{encoder}.0f / 8192.0f" in source
assert re.search(
    r"chassis_gm6020_speed_pid\s*=\s*\{\s*"
    r"\.kp\s*=\s*0\.25f,\s*\.ki\s*=\s*0\.005f,.*?"
    r"\.output_max\s*=\s*2\.0f,\s*\.integral_max\s*=\s*1\.5f,",
    source,
    re.S,
)
assert "chassis_gm6020_id2_speed_pid" not in source
assert "RobotSteeringUpdateId2SpeedGain" not in source
assert "kRobotSteeringId2HoldSpeedKp" not in source
assert "chassis_id2_move_gain_active" not in source
assert "DjiMotorCurrentLoopInit" not in source
assert re.search(
    r"DjiMotorInit\(&g_robot\.chassis_motor_3508\[i\],\s*kDjiMotorM3508,\s*"
    r"kDjiMotorModeSpeed", source
)
assert re.search(
    r"DjiMotorInit\(&g_robot\.chassis_motor_6020\[i\],\s*kDjiMotorGm6020,\s*"
    r"kDjiMotorModePosition,\s*&chassis_gm6020_speed_pid,\s*"
    r"&chassis_gm6020_position_pid\)", source
)
assert "chassis_motor_3508[kRobotChassisCanSlotWheelIndex[3]]" in source
assert "kRobotChassisCanSlotWheelIndex[3]" in source
assert "kRobotChassisCanSlotWheelIndex[4] = {0U, 2U, 3U, 1U}" in source
assert "else if (std_id == 0x205U)" not in source

names = (
    "RobotClamp",
    "RobotWrapPi",
    "RobotSteeringTargetAngle",
    "RobotSteeringVectorActive",
    "RobotSteeringUpdateTarget",
    "RobotChassisFeedbackValid",
    "RobotSteeringDriveReady",
    "RobotPackDjiCurrents",
    "RobotFdcan1RxCallback",
    "RobotFdcan3RxCallback",
)
functions = []
for name in names:
    match = re.search(
        r"^static [^\n]*\b" + name + r"\([^{}]*\) \{.*?^\}",
        source,
        re.M | re.S,
    )
    if not match:
        raise RuntimeError(f"Cannot locate {name}")
    functions.append(match.group())

harness = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
enum { kRobotChassisMotorCount = 4U, kRobotChassisNoMotorTxTest = 0U,
       kRobotChassisFeedbackTimeoutMs = 50U, kRobotChassisPeriodMs = 2U };
static const float kRobotSteeringDriveEnableErrorRad = 0.10f;
static const float kRobotSteeringDriveDisableErrorRad = 0.40f;
static const float kRobotSteeringVectorHoldSpeedMps = 0.05f;
static const float kRobotSteeringVectorResumeSpeedMps = 0.10f;
static const float kRobotSteeringTargetRateMaxRadps = 4.0f;
static const float kRobotSteeringDirectionLockBandRad = 0.10f;
static const uint8_t kRobotChassisCanSlotWheelIndex[4] = {0U, 2U, 3U, 1U};
static const float kRobotSteeringForwardAngleRad[4] = {
    3060.0f / 8192.0f * 2.0f * (float)M_PI,
    2421.0f / 8192.0f * 2.0f * (float)M_PI,
    2383.0f / 8192.0f * 2.0f * (float)M_PI,
    3097.0f / 8192.0f * 2.0f * (float)M_PI,
};
static const float kRobotSteeringDirection[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
static uint32_t tick = 100U;
static uint32_t osKernelGetTickCount(void) { return tick; }
typedef struct {
  int type;
  unsigned updates;
  struct {
    float angle, given_angle, given_current;
    struct { float kp; } speed_loop;
  } state;
} DjiMotor;
typedef struct {
  DjiMotor chassis_motor_3508[kRobotChassisMotorCount];
  DjiMotor chassis_motor_6020[kRobotChassisMotorCount];
  bool chassis_drive_steering_ready;
  struct { bool spin_translation_active, spin_drive_tracking; } chassis;
  volatile uint32_t chassis_3508_last_feedback_tick[kRobotChassisMotorCount];
  volatile uint32_t chassis_6020_last_feedback_tick[kRobotChassisMotorCount];
  volatile bool chassis_3508_has_feedback[kRobotChassisMotorCount];
  volatile bool chassis_6020_has_feedback[kRobotChassisMotorCount];
  bool chassis_steering_target_initialized[kRobotChassisMotorCount];
  float chassis_steering_requested_angle[kRobotChassisMotorCount];
  float chassis_steering_last_direction[kRobotChassisMotorCount];
} RobotApp;
static RobotApp g_robot;
static void DjiMotorUpdate(DjiMotor *motor, uint8_t *data) {
  (void)data;
  ++motor->updates;
}
static int16_t DjiMotorCurrentAmperesToUnits(int type, float current) {
  (void)type;
  return (int16_t)(current * 1000.0f);
}
""" + "\n".join(functions) + r"""
int main(void) {
  uint8_t frame[8] = {0};
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    float target = RobotSteeringTargetAngle(i, 0.0f);
    assert(fabsf(RobotWrapPi(target - kRobotSteeringForwardAngleRad[i])) <
           0.00001f);
    float left_target = RobotSteeringTargetAngle(i, (float)M_PI / 2.0f);
    assert(fabsf(RobotWrapPi(left_target - target + (float)M_PI / 2.0f)) <
           0.00001f);
    float right_target = RobotSteeringTargetAngle(i, -(float)M_PI / 2.0f);
    assert(fabsf(RobotWrapPi(right_target - target - (float)M_PI / 2.0f)) <
           0.00001f);
  }

  RobotFdcan1RxCallback(0x204U, frame);
  RobotFdcan1RxCallback(0x209U, frame);
  for (uint8_t can_slot = 0; can_slot < kRobotChassisMotorCount; can_slot++) {
    uint8_t wheel_index = kRobotChassisCanSlotWheelIndex[can_slot];
    assert(g_robot.chassis_motor_6020[wheel_index].updates == 0U);
    RobotFdcan1RxCallback(0x205U + can_slot, frame);
    assert(g_robot.chassis_motor_6020[wheel_index].updates == 1U);
    assert(g_robot.chassis_6020_has_feedback[wheel_index]);
  }

  RobotFdcan3RxCallback(0x200U, frame);
  RobotFdcan3RxCallback(0x205U, frame);
  for (uint8_t can_slot = 0; can_slot < kRobotChassisMotorCount; can_slot++) {
    uint8_t wheel_index = kRobotChassisCanSlotWheelIndex[can_slot];
    assert(g_robot.chassis_motor_3508[wheel_index].updates == 0U);
    RobotFdcan3RxCallback(0x201U + can_slot, frame);
    assert(g_robot.chassis_motor_3508[wheel_index].updates == 1U);
    assert(g_robot.chassis_3508_has_feedback[wheel_index]);
  }

  g_robot.chassis_motor_3508[0].state.given_current = 1.2f;
  g_robot.chassis_motor_3508[1].state.given_current = -1.2f;
  g_robot.chassis_motor_3508[2].state.given_current = 0.5f;
  g_robot.chassis_motor_3508[3].state.given_current = -0.5f;
  RobotPackDjiCurrents(&g_robot.chassis_motor_3508[0],
                       &g_robot.chassis_motor_3508[1],
                       &g_robot.chassis_motor_3508[2],
                       &g_robot.chassis_motor_3508[3], frame);
  const uint8_t expected[8] = {0x04U, 0xB0U, 0xFBU, 0x50U,
                               0x01U, 0xF4U, 0xFEU, 0x0CU};
  assert(memcmp(frame, expected, sizeof(expected)) == 0);

  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_motor_6020[i].state.given_angle = 0.2f;
    g_robot.chassis_steering_requested_angle[i] = 0.2f;
  }
  assert(!RobotSteeringDriveReady());
  // A close rate-limited setpoint must not hide a distant final heading.
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_motor_6020[i].state.given_angle = 0.008f;
    g_robot.chassis_steering_requested_angle[i] = 0.6f;
  }
  assert(!RobotSteeringDriveReady());
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_motor_6020[i].state.given_angle = 0.08f;
    g_robot.chassis_steering_requested_angle[i] = 0.08f;
  }
  assert(RobotSteeringDriveReady());
  g_robot.chassis_motor_6020[2].state.given_angle = 0.41f;
  g_robot.chassis_steering_requested_angle[2] = 0.41f;
  assert(!RobotSteeringDriveReady());
  g_robot.chassis_motor_6020[2].state.given_angle = 0.08f;
  g_robot.chassis_steering_requested_angle[2] = 0.08f;
  assert(RobotSteeringDriveReady());
  tick += 51U;
  assert(!RobotSteeringDriveReady());

  // Dynamic spin tracking must wait for initial alignment, then use the
  // solver's per-wheel compensation instead of cutting all four drive motors.
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_6020_last_feedback_tick[i] = tick;
    g_robot.chassis_motor_6020[i].state.angle = 0;
    g_robot.chassis_steering_requested_angle[i] = 0.6f;
  }
  g_robot.chassis.spin_translation_active = true;
  assert(!RobotSteeringDriveReady());
  assert(!g_robot.chassis.spin_drive_tracking);
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++)
    g_robot.chassis_steering_requested_angle[i] = 0.08f;
  assert(RobotSteeringDriveReady());
  assert(g_robot.chassis.spin_drive_tracking);
  for (unsigned pass = 0; pass < 20; ++pass) {
    g_robot.chassis_steering_requested_angle[2] =
        pass % 2 ? 0.6f : 0.08f;
    assert(RobotSteeringDriveReady());
  }
  // Feedback loss must stop and require fresh initial alignment.
  tick += 51U;
  assert(!RobotSteeringDriveReady());
  assert(!g_robot.chassis.spin_drive_tracking);
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++)
    g_robot.chassis_6020_last_feedback_tick[i] = tick;
  g_robot.chassis_steering_requested_angle[2] = 0.6f;
  assert(!RobotSteeringDriveReady());
  g_robot.chassis_steering_requested_angle[2] = 0.08f;
  assert(RobotSteeringDriveReady());
  // Exiting spin immediately restores the strict normal-motion gate.
  g_robot.chassis.spin_translation_active = false;
  g_robot.chassis_steering_requested_angle[2] = 0.41f;
  assert(!RobotSteeringDriveReady());
  assert(!g_robot.chassis.spin_drive_tracking);

  assert(!RobotSteeringVectorActive(0.06f, false));
  assert(RobotSteeringVectorActive(0.11f, false));
  assert(RobotSteeringVectorActive(0.06f, true));
  assert(!RobotSteeringVectorActive(0.04f, true));

  DjiMotor target_motor = {0};
  g_robot.chassis_steering_requested_angle[0] = (float)M_PI - 0.02f;
  RobotSteeringUpdateTarget(0, &target_motor);
  assert(fabsf(target_motor.state.given_angle - 0.008f) < 0.00001f);
  g_robot.chassis_steering_requested_angle[0] = -(float)M_PI + 0.02f;
  RobotSteeringUpdateTarget(0, &target_motor);
  assert(fabsf(target_motor.state.given_angle - 0.016f) < 0.00001f);

  puts("PASS: four chassis groups, current slots, timeout and interlock");
}
"""

with tempfile.TemporaryDirectory(prefix="chassis_four_pid_") as directory:
    test_c = Path(directory) / "test.c"
    test_exe = Path(directory) / "test.exe"
    test_c.write_text(harness, encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", str(test_c),
         "-lm", "-o", str(test_exe)],
        check=True,
    )
    subprocess.run([str(test_exe)], check=True)

    pid_c = Path(directory) / "pid_test.c"
    pid_exe = Path(directory) / "pid_test.exe"
    pid_c.write_text(r"""
#include <assert.h>
#include <math.h>
#include "dev_dji_motor.h"
int main(void) {
  PidConfig speed = {
      .kp = 0.1f, .sampling_time = 0.002f,
      .is_antiwindup_enabled = true, .output_max = 1.0f};
  PidConfig position = {
      .kp = 2.0f, .sampling_time = 0.002f,
      .is_antiwindup_enabled = true, .output_max = 3.0f};
  DjiMotor drive, steer;
  DjiMotorInit(&drive, kDjiMotorM3508, kDjiMotorModeSpeed, &speed, 0);
  drive.state.given_omega = 2.0f;
  DjiMotorCurrentCalculate(&drive);
  assert(fabsf(drive.state.given_current - 0.2f) < 0.00001f);

  DjiMotorInit(&steer, kDjiMotorGm6020, kDjiMotorModePosition,
               &speed, &position);
  steer.state.angle = 2.0f * (float)M_PI - 0.02f;
  steer.state.continuous_angle = steer.state.angle;
  steer.state.given_angle = 0.01f;
  DjiMotorCurrentCalculate(&steer);
  assert(fabsf(steer.state.given_omega - 0.06f) < 0.0001f);
  assert(fabsf(steer.state.given_current - 0.006f) < 0.0001f);

  PidConfig chassis_speed = {
      .kp = 0.12f, .ki = 0.20f, .sampling_time = 0.002f,
      .is_antiwindup_enabled = true, .output_max = 1.2f,
      .integral_max = 1.5f};
  DjiMotorInit(&steer, kDjiMotorGm6020, kDjiMotorModeSpeed,
               &chassis_speed, 0);
  steer.state.given_omega = 0.0f;
  steer.state.omega = 100.0f;
  DjiMotorCurrentCalculate(&steer);
  assert(fabsf(steer.state.given_current + 1.2f) < 0.0001f);
  int16_t command_units = DjiMotorCurrentAmperesToUnits(
      kDjiMotorGm6020, steer.state.given_current);
  assert(command_units > -7000 && command_units < 7000);

  PidConfig current_test_speed = {
      .kp = 1.0f, .sampling_time = 0.002f,
      .is_antiwindup_enabled = true, .output_max = 1.0f};
  DjiMotorInit(&steer, kDjiMotorGm6020, kDjiMotorModeSpeed,
               &current_test_speed, 0);
  steer.state.given_omega = 1.0f;
  steer.state.omega = 0.0f;
  steer.state.current = 0.0f;
  DjiMotorCurrentCalculate(&steer);
  assert(fabsf(steer.state.given_current - 1.0f) < 0.0001f);
  steer.state.current = 0.4f;
  DjiMotorCurrentCalculate(&steer);
  assert(fabsf(steer.state.given_current - 1.0f) < 0.0001f);

  PidConfig hold_speed = {
      .kp = 0.20f, .sampling_time = 0.002f,
      .is_antiwindup_enabled = true, .output_max = 0.8f};
  PidConfig hold_position = {
      .kp = 40.0f, .sampling_time = 0.002f,
      .is_antiwindup_enabled = true, .output_max = 4.0f};
  for (int count = -1; count <= 1; ++count) {
    for (int rpm = -1; rpm <= 1; rpm += 2) {
      DjiMotorInit(&steer, kDjiMotorGm6020, kDjiMotorModePosition,
                   &hold_speed, &hold_position);
      steer.state.continuous_angle =
          (float)count * 2.0f * (float)M_PI / 8192.0f;
      steer.state.omega = (float)rpm * 2.0f * (float)M_PI / 60.0f;
      DjiMotorCurrentCalculate(&steer);
      assert(fabsf(steer.state.given_current) < 0.03f);
    }
  }
  DjiMotorInit(&steer, kDjiMotorGm6020, kDjiMotorModePosition,
               &hold_speed, &hold_position);
  steer.state.given_angle = 0.2f;
  DjiMotorCurrentCalculate(&steer);
  assert(fabsf(steer.state.given_current - 0.8f) < 0.0001f);
  steer.state.given_angle = -0.2f;
  DjiMotorCurrentCalculate(&steer);
  assert(fabsf(steer.state.given_current + 0.8f) < 0.0001f);

}
""", encoding="utf-8")
    subprocess.run(
        ["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
         "-I", str(ROOT / "Skywalker/Device/Inc"),
         "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
         str(pid_c),
         str(ROOT / "Skywalker/Device/Src/dev_dji_motor.c"),
         str(ROOT / "Skywalker/Algorithm/Src/alg_pid.c"),
         str(ROOT / "Skywalker/Algorithm/Src/alg_conversion.c"),
         "-lm", "-o", str(pid_exe)],
        check=True,
    )
    subprocess.run([str(pid_exe)], check=True)
