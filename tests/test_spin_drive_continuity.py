"""Exercise actual spin kinematics, steering gate, motor PID and CAN with lag."""

from pathlib import Path
import subprocess
import tempfile

from test_swerve_port import ROOT, constant, function, harness

base = harness.split("static void test_vectors(void)")[0]
base = base.replace(
    "bool chassis_steering_vector_active[4];",
    "bool chassis_steering_vector_active[4];\n"
    "  bool chassis_steering_target_initialized[4];\n"
    "  float chassis_steering_last_direction[4];\n"
    "  bool chassis_drive_steering_ready;\n"
    "  uint32_t chassis_3508_last_feedback_tick[4], chassis_6020_last_feedback_tick[4];\n"
    "  bool chassis_3508_has_feedback[4], chassis_6020_has_feedback[4];",
).replace(
    "float spin_translation_relative_yaw;",
    "float spin_translation_relative_yaw, estimated_power;",
)
constants = "\n".join(constant(name) for name in (
    "kRobotSteeringTargetRateMaxRadps", "kRobotSteeringDirectionLockBandRad",
    "kRobotSteeringDriveEnableErrorRad", "kRobotSteeringDriveDisableErrorRad",
    "kRobotNoRefereeCurrentScale",
))
stubs = r"""
enum { kRobotChassisNoMotorTxTest = 0, kRobotChassisPeriodMs = 2,
       kRobotChassisFeedbackTimeoutMs = 50, kRobotRefereeEnabled = 0,
       kRobotCanDataLength = 8 };
static uint32_t tick = 100;
static unsigned can_calls;
static int hfdcan1, hfdcan3;
static uint32_t osKernelGetTickCount(void) { return tick; }
static void RobotChassisImuUpdate(void) { /* deterministic measured yaw below */ }
static float RobotEstimateChassisPower(void) { return 0; }
static float RobotChassisPowerLimit(void) { return 100; }
static void RobotApplyChassisPowerLimit(float limit) { (void)limit; }
static void FdcanTransmit(int *bus, uint8_t *data, uint16_t size, uint32_t id) {
  assert((bus == &hfdcan1 && id == 0x1FE) ||
         (bus == &hfdcan3 && id == 0x200));
  assert(data != NULL && size == 8);
  ++can_calls;
}
"""
functions = "\n".join(function(name) for name in (
    "RobotSteeringUpdateTarget", "RobotChassisFeedbackValid",
    "RobotSteeringDriveReady", "RobotPackDjiCurrents", "RobotChassisControl",
))
tests = r"""
static void feedback_aligned(void) {
  for (int i = 0; i < 4; ++i) {
    DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    steer->state.angle = g_robot.chassis_steering_requested_angle[i];
    steer->state.continuous_angle = steer->state.angle;
    g_robot.chassis_3508_has_feedback[i] = true;
    g_robot.chassis_6020_has_feedback[i] = true;
    g_robot.chassis_3508_last_feedback_tick[i] = tick;
    g_robot.chassis_6020_last_feedback_tick[i] = tick;
  }
}
static void assert_stopped(void) {
  assert(!g_robot.chassis_drive_steering_ready);
  assert(!g_robot.chassis.spin_drive_tracking);
  for (int i = 0; i < 4; ++i) {
    near(g_robot.chassis_motor_3508[i].state.given_omega, 0, 0.00001f);
    near(g_robot.chassis_motor_3508[i].state.given_current, 0, 0.00001f);
  }
}
int main(void) {
  reset_robot();
  PidConfig drive_pid = {.kp = 0.05f, .sampling_time = 0.002f,
                        .output_max = 14.648f};
  PidConfig steer_speed = {.kp = 0.20f, .sampling_time = 0.002f,
                          .output_max = 0.8f};
  PidConfig steer_position = {.kp = 40, .sampling_time = 0.002f,
                             .output_max = 4};
  for (int i = 0; i < 4; ++i) {
    DjiMotorInit(&g_robot.chassis_motor_3508[i], kDjiMotorM3508,
                 kDjiMotorModeSpeed, &drive_pid, NULL);
    DjiMotorInit(&g_robot.chassis_motor_6020[i], kDjiMotorGm6020,
                 kDjiMotorModePosition, &steer_speed, &steer_position);
    g_robot.chassis_motor_6020[i].state.angle = kRobotSteeringForwardAngleRad[i];
    g_robot.chassis_motor_6020[i].state.continuous_angle =
        g_robot.chassis_motor_6020[i].state.angle;
  }
  g_robot.chassis.imu_ready = true;
  g_robot.chassis.yaw_rate = 1;
  HostVelocityCommand command = {.vx = 1, .wz = 1};
  RobotChassisKinematics(&command);
  feedback_aligned();
  // Entering spin cannot inherit ordinary-mode drive permission when unaligned.
  g_robot.chassis_drive_steering_ready = true;
  g_robot.chassis_motor_6020[0].state.angle -= 0.55f;
  g_robot.chassis_motor_6020[0].state.continuous_angle =
      g_robot.chassis_motor_6020[0].state.angle;
  RobotChassisControl(&command);
  assert_stopped();
  feedback_aligned();
  RobotChassisControl(&command);
  assert(g_robot.chassis_drive_steering_ready && g_robot.chassis.spin_drive_tracking);
  float aligned_speed = fabsf(g_robot.chassis_motor_3508[0].state.given_omega);

  // After startup, replay a 32-degree tracking lag through the real solver,
  // gate, PID and CAN. Previously this cut all four drive currents every time.
  for (unsigned pass = 0; pass < 30; ++pass) {
    tick += 2;
    g_robot.chassis.yaw_angle += 0.002f;
    RobotChassisKinematics(&command);
    feedback_aligned();
    g_robot.chassis_motor_6020[0].state.angle = RobotWrapPi(
        g_robot.chassis_motor_6020[0].state.angle - 0.55f);
    g_robot.chassis_motor_6020[0].state.continuous_angle =
        g_robot.chassis_motor_6020[0].state.angle;
    RobotChassisControl(&command);
    assert(g_robot.chassis_drive_steering_ready);
    assert(g_robot.chassis.spin_drive_tracking);
    for (int i = 0; i < 4; ++i) {
      assert(fabsf(g_robot.chassis_motor_3508[i].state.given_current) > 0.001f);
      assert(fabsf(g_robot.chassis_motor_6020[i].state.given_current) <= 0.80001f);
    }
    assert(fabsf(g_robot.chassis_motor_3508[0].state.given_omega) < aligned_speed);
  }
  // A wheel orthogonal to its requested vector gets zero drive via cos^3,
  // while the other aligned wheels retain drive permission.
  RobotChassisKinematics(&command);
  feedback_aligned();
  g_robot.chassis_motor_6020[0].state.angle = RobotWrapPi(
      g_robot.chassis_motor_6020[0].state.angle - (float)M_PI / 2);
  g_robot.chassis_motor_6020[0].state.continuous_angle =
      g_robot.chassis_motor_6020[0].state.angle;
  RobotChassisControl(&command);
  assert(g_robot.chassis_drive_steering_ready);
  near(g_robot.chassis_motor_3508[0].state.given_omega, 0, 0.00001f);
  for (int i = 1; i < 4; ++i)
    assert(fabsf(g_robot.chassis_motor_3508[i].state.given_current) > 0.001f);

  // Both drive and steering feedback loss retain the all-wheel stop behavior.
  for (unsigned steering_loss = 0; steering_loss < 2; ++steering_loss) {
    feedback_aligned();
    RobotChassisControl(&command);
    if (steering_loss) g_robot.chassis_6020_last_feedback_tick[1] = tick - 51;
    else g_robot.chassis_3508_last_feedback_tick[1] = tick - 51;
    RobotChassisControl(&command);
    assert_stopped();
    feedback_aligned();
    g_robot.chassis_motor_6020[0].state.angle -= 0.55f;
    g_robot.chassis_motor_6020[0].state.continuous_angle =
        g_robot.chassis_motor_6020[0].state.angle;
    RobotChassisControl(&command);
    assert_stopped(); // Reappearance alone cannot bypass initial alignment.
    feedback_aligned();
    RobotChassisControl(&command);
    assert(g_robot.chassis_drive_steering_ready);
  }
  // A zero command clears the tracking latch and drive output.
  HostVelocityCommand zero = {0};
  RobotChassisControl(&zero);
  assert_stopped();
  assert(can_calls >= 60);
  puts("PASS: continuous spin drive through real PID/CAN, per-wheel cosine reduction and timeout rearming");
}
"""

with tempfile.TemporaryDirectory(prefix="spin_drive_continuity_") as directory:
    work = Path(directory)
    test_c = work / "test.c"
    executable = work / "test.exe"
    test_c.write_text(base + constants + stubs + functions + tests, encoding="utf-8")
    subprocess.run([
        "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
        "-I", str(ROOT / "Skywalker/Device/Inc"),
        "-I", str(ROOT / "Skywalker/Application/Inc"),
        str(test_c), str(ROOT / "Skywalker/Algorithm/Src/alg_swerve.c"),
        str(ROOT / "Skywalker/Algorithm/Src/alg_pid.c"),
        str(ROOT / "Skywalker/Algorithm/Src/alg_conversion.c"),
        str(ROOT / "Skywalker/Device/Src/dev_dji_motor.c"),
        "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
