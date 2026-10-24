"""Exercise UART7 parsing, actual chassis task dispatch, kinematics and stop CAN."""

from pathlib import Path
import re
import subprocess
import tempfile

from test_swerve_port import ROOT, harness as swerve_harness, source


def function(name):
    match = re.search(
        r"^(?:static )?[^\n]*\b" + name + r"\([^{}]*\) \{.*?^\}",
        source, re.M | re.S,
    )
    if not match:
        raise RuntimeError(f"Cannot locate {name}")
    return match.group()


base = swerve_harness.split("static void test_vectors(void)")[0]
base = base.replace(
    "bool chassis_steering_vector_active[4];",
    "bool chassis_steering_vector_active[4];\n"
    "  bool chassis_steering_target_initialized[4];\n"
    "  float chassis_steering_last_direction[4];\n"
    "  bool chassis_drive_steering_ready;\n"
    "  bool chassis_host_control_active;\n"
    "  uint32_t chassis_imu_last_tick;",
)

hal_stub = r"""
#include <stdint.h>
extern uint32_t test_tick, test_primask;
static inline uint32_t HAL_GetTick(void) { return test_tick; }
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1; }
static inline void __set_PRIMASK(uint32_t mask) { test_primask = mask; }
"""

extra = r"""
#include "stm32h7xx_hal.h"
uint32_t test_tick, test_primask;
enum { kRobotChassisEnabled = 1U, kRobotChassisHostControlEnabled = 1U,
       kRobotHostCommandTimeoutMs = 100U, kRobotDt7OfflineTimeoutMs = 100U,
       kRobotRemoteSwitchSafeModeEnabled = TEST_REMOTE_STOP,
       kRobotCanDataLength = 8U, kRobotBigYawEnabled = 0U };
static void RobotBigYawControl(void) {}
static uint32_t dt7_frames, dt7_tick;
static uint32_t osKernelGetTickCount(void) { return test_tick; }
static uint32_t Dt7FrameCount(void) { return dt7_frames; }
static uint32_t Dt7LastUpdateTick(void) { return dt7_tick; }
static bool SwitchIsDown(uint8_t value) { return value == 2U; }
static unsigned control_calls, can_calls;
static bool last_control_was_host;
static int hfdcan1, hfdcan3;
static uint32_t can_ids[2];
static void FdcanTransmit(int *bus, uint8_t *data, uint16_t length,
                          uint32_t id) {
  assert((id == 0x200 && bus == &hfdcan3) ||
         (id == 0x1FE && bus == &hfdcan1));
  assert(length == 8);
  for (unsigned i = 0; i < 8; ++i) assert(data[i] == 0);
  can_ids[can_calls++ % 2] = id;
}
// Test the task's real dispatch and solver while separate PID/interlock tests
// cover the motor control loop. No hardware or IMU transactions on the host.
static void RobotChassisControl(const HostVelocityCommand *command) {
  ++control_calls;
  last_control_was_host = command != NULL;
  RobotChassisKinematics(command);
}
"""

functions = "\n".join(function(name) for name in (
    "RobotStopChassisMotors", "RobotTransmitChassisStopCurrents",
    "RobotDt7IsOnline", "RobotEnterChassisSafeMode", "RobotChassisSelectControlSource",
    "ChassisTaskLoop",
))

tests = r"""
static void receive(int16_t vx, int16_t vy, int16_t wz, bool valid) {
  // Wire axes match the body frame: first vx forward, then vy left.
  int16_t values[3] = {vx, vy, wz};
  uint8_t frame[9] = {0xAA, 6};
  for (unsigned i = 0; i < 3; ++i) {
    frame[2 + 2*i] = (uint8_t)(uint16_t)values[i];
    frame[3 + 2*i] = (uint8_t)((uint16_t)values[i] >> 8);
  }
  for (unsigned i = 0; i < 8; ++i) frame[8] ^= frame[i];
  if (!valid) frame[8] ^= 1;
  HostVelocityProtocolRxCallback(frame, 3);
  HostVelocityProtocolRxCallback(frame + 3, 6);
}

static void stopped(void) {
  unsigned before = control_calls, before_can = can_calls;
  ChassisTaskLoop();
  assert(control_calls == before && can_calls == before_can + 2);
  assert(can_ids[0] == 0x200 && can_ids[1] == 0x1FE);
  assert(g_robot.chassis.steering_idle);
  assert(!g_robot.chassis.spin_translation_active);
  near(g_robot.chassis.desired_velocity.vx, 0, 0.00001f);
  near(g_robot.chassis.desired_velocity.vy, 0, 0.00001f);
  near(g_robot.chassis.desired_velocity.omega_z, 0, 0.00001f);
  for (unsigned i = 0; i < 4; ++i) {
    near(g_robot.chassis_motor_3508[i].state.given_omega, 0, 0.00001f);
    near(g_robot.chassis_motor_3508[i].state.given_current, 0, 0.00001f);
    near(g_robot.chassis_motor_6020[i].state.given_current, 0, 0.00001f);
  }
}

static void check_spin_vectors(float vx, float vy, float wz,
                               float body_vx, float body_vy, float body_wz) {
  SwerveWheelVector wheels[4];
  SwerveResolve(body_vx, body_vy, body_wz, kRobotChassisHalfWidthM,
                kRobotChassisHalfLengthM, wheels);
  for (unsigned pass = 0; pass < 2; ++pass) {
    ChassisTaskLoop();
    for (unsigned i = 0; i < 4; ++i)
      g_robot.chassis_motor_6020[i].state.angle =
          g_robot.chassis_steering_requested_angle[i];
  }
  assert(!g_robot.chassis.steering_idle);
  near(g_robot.chassis.desired_velocity.vx, vx, 0.00001f);
  near(g_robot.chassis.desired_velocity.vy, vy, 0.00001f);
  near(g_robot.chassis.desired_velocity.omega_z, wz, 0.00001f);
  float largest = 0;
  for (unsigned i = 0; i < 4; ++i) largest = fmaxf(largest, wheels[i].speed_mps);
  float scale = fminf(1, kRobotChassisMaxWheelSpeedMps / largest);
  for (unsigned i = 0; i < 4; ++i) {
    float heading = RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                                kRobotSteeringForwardAngleRad[i]) /
                    kRobotSteeringDirection[i];
    float speed = g_robot.chassis_motor_3508[i].state.given_omega *
                  kRobotWheelRadiusM / kRobotDriveReductionRatio /
                  kRobotDriveDirection[i];
    near(cosf(heading) * speed, wheels[i].vx * scale, 0.00002f);
    near(sinf(heading) * speed, wheels[i].vy * scale, 0.00002f);
  }
}

static void check_vectors(float vx, float vy, float wz) {
  bool blocked = !g_robot.chassis.imu_ready &&
                 fabsf(wz) > kRobotChassisSpinCommandEpsilon;
  check_spin_vectors(vx, vy, wz, blocked ? 0 : vx, blocked ? 0 : vy, wz);
}

static void test_spin_host(void) {
  reset_robot();
  HostVelocityProtocolInit();
  dt7_frames = 0;
  test_tick = 1000;
  g_robot.chassis.imu_ready = true;
  g_robot.chassis.yaw_angle = 1;
  g_robot.chassis.yaw_rate = 1;
  receive(0, 0, 1000, true);
  check_spin_vectors(0, 0, 1, 0, 0, 1);
  assert(g_robot.chassis.spin_translation_active);
  near(g_robot.chassis.spin_translation_reference_yaw, 1, 0.00001f);
  // UART7 remains in the current body frame at the quarter turn. Only the
  // 5 ms prediction rotates its vectors; integrated yaw must not rotate twice.
  g_robot.chassis.yaw_angle = 1 + (float)M_PI / 2.0f - 0.005f;
  float lead = kRobotChassisSpinYawCompensationS;
  receive(0, 1000, 1000, true);
  check_spin_vectors(0, 1, 1, 0.4f * sinf(lead), 0.4f * cosf(lead), 1);
  receive(1000, 0, 1000, true);
  check_spin_vectors(1, 0, 1, 0.3f * cosf(lead), -0.3f * sinf(lead), 1);
  receive(-1000, -1000, 1000, true);
  check_spin_vectors(-1, -1, 1, -0.4f * (cosf(lead) + sinf(lead)),
                     0.4f * (sinf(lead) - cosf(lead)), 1);
  test_tick = 1101;
  stopped();
  near(g_robot.chassis.spin_translation_reference_yaw, 0, 0.00001f);
  // Reception after timeout latches the current yaw, without stale heading.
  g_robot.chassis.yaw_angle = -2;
  g_robot.chassis.yaw_rate = 0;
  receive(0, 1000, -1000, true);
  check_spin_vectors(0, 1, -1, 0, 0.4f, -1.5f);
  near(g_robot.chassis.spin_translation_reference_yaw, -2, 0.00001f);
  receive(0, 1000, 0, true);
  check_vectors(0, 1, 0);
  assert(!g_robot.chassis.spin_translation_active);
  receive(0, 1000, 1000, true);
  check_spin_vectors(0, 1, 1, 0, 0.4f, 1.5f);
  receive(0, 0, 0, true);
  ChassisTaskLoop();
  assert(!g_robot.chassis.spin_translation_active);
}

int main(void) {
  reset_robot();
  HostVelocityProtocolInit();
  test_tick = 100;
  stopped(); // No UART7 frame and no DT7: zero-current CAN.

  // Literal protocol examples test wire order independently of receive().
  uint8_t forward[9] = {0xAA, 0x06, 0xE8, 0x03, 0, 0, 0, 0, 0x47};
  uint8_t left[9] = {0xAA, 0x06, 0, 0, 0xE8, 0x03, 0, 0, 0x47};
  HostVelocityProtocolRxCallback(forward, sizeof(forward));
  check_vectors(1, 0, 0);
  for (unsigned i = 0; i < 4; ++i) {
    near(RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                    kRobotSteeringForwardAngleRad[i]), 0, 0.00001f);
    near(g_robot.chassis_motor_3508[i].state.given_omega *
         kRobotWheelRadiusM / kRobotDriveReductionRatio /
         kRobotDriveDirection[i], 1, 0.00001f);
  }
  HostVelocityProtocolRxCallback(left, sizeof(left));
  check_vectors(0, 1, 0);
  for (unsigned i = 0; i < 4; ++i) {
    float angle = RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                             kRobotSteeringForwardAngleRad[i]) /
                  kRobotSteeringDirection[i];
    float speed = g_robot.chassis_motor_3508[i].state.given_omega *
                  kRobotWheelRadiusM / kRobotDriveReductionRatio /
                  kRobotDriveDirection[i];
    near(cosf(angle) * speed, 0, 0.00001f);
    near(sinf(angle) * speed, 1, 0.00001f);
  }

  // Remote keys, stick and auto-spin switch cannot override a host command.
  g_dt7_object.rocker.ch2 = 660;
  g_dt7_object.rocker.ch3 = 660;
  g_dt7_object.rocker.wheel = 660;
  g_dt7_object.key.w = g_dt7_object.key.a = g_dt7_object.key.e = true;
  g_dt7_object.rocker.sw1 = 1;
  receive(1200, -700, 1500, true);
  check_vectors(1.2f, -0.7f, 1.5f);
  receive(-1200, 700, -1500, true);
  check_vectors(-1.2f, 0.7f, -1.5f);
  // UART7 positive vx points forward, positive vy points left.
  receive(1000, 0, 0, true);
  check_vectors(1, 0, 0);
  receive(-1000, 0, 0, true);
  check_vectors(-1, 0, 0);
  receive(0, 1000, 0, true);
  check_vectors(0, 1, 0);
  receive(0, -1000, 0, true);
  check_vectors(0, -1, 0);
  receive(0, 0, 1000, true);
  check_vectors(0, 0, 1);
  receive(0, 0, -1000, true);
  check_vectors(0, 0, -1);
  receive(32767, -32768, 32767, true);
  check_vectors(5, -5, 10);
  receive(0, 0, 1, true); // Tiny pure yaw must not be suppressed by idle logic.
  check_vectors(0, 0, 0.001f);

  receive(0, 0, 0, true);
  g_robot.chassis.imu_ready = true;
  g_robot.chassis.yaw_rate = 2;
  ChassisTaskLoop();
  assert(g_robot.chassis.steering_idle);
  for (unsigned i = 0; i < 4; ++i)
    near(g_robot.chassis_motor_3508[i].state.given_omega, 0, 0.00001f);
  g_robot.chassis.imu_ready = false;

  receive(0, 1000, 0, true);
  test_tick = 200;
  check_vectors(0, 1, 0); // 100 ms is still valid.
  receive(1000, 0, 0, false); // Invalid frame cannot postpone stopping.
  test_tick = 201;
  stopped();
  // An online DT7 takes priority even when UART7 is stale.
  memset(&g_dt7_object, 0, sizeof(g_dt7_object));
  g_dt7_object.rocker.sw1 = 3;
  g_dt7_object.rocker.ch3 = 132; // 1 m/s forward from the remote.
  dt7_frames = 1;
  dt7_tick = test_tick;
  check_vectors(1, 0, 0);
  assert(!last_control_was_host && !g_robot.chassis_host_control_active);
  receive(0, -1000, 0, true); // Both online, conflicting velocity commands.
  check_vectors(1, 0, 0);
  assert(!last_control_was_host);

  // An online centered remote blocks the host as well.
  g_dt7_object.rocker.ch3 = 0;
  ChassisTaskLoop();
  assert(!last_control_was_host && g_robot.chassis.steering_idle);
  g_dt7_object.rocker.ch3 = 132;

  g_dt7_object.rocker.sw1 = 2;
  if (TEST_REMOTE_STOP) stopped();
  else {
    check_vectors(1, 0, 0);
    assert(!last_control_was_host);
  }
  g_dt7_object.rocker.sw1 = 3;
  test_tick = 301; // Exactly 100 ms after DT7: remote is still online.
  receive(0, -1000, 0, true);
  check_vectors(1, 0, 0);
  assert(!last_control_was_host);

  // 101 ms after DT7: host takes over, with a cleared spin reference.
  g_robot.chassis.spin_translation_active = true;
  g_robot.chassis.spin_translation_reference_yaw = 1;
  test_tick = 302;
  check_vectors(0, -1, 0);
  assert(last_control_was_host && g_robot.chassis_host_control_active);
  assert(!g_robot.chassis.spin_translation_active);
  near(g_robot.chassis.spin_translation_reference_yaw, 0, 0.00001f);

  // A returning DT7 immediately regains control despite a fresh host frame.
  dt7_tick = test_tick;
  check_vectors(1, 0, 0);
  assert(!last_control_was_host && !g_robot.chassis_host_control_active);
  // DT7 tick wrap does not make an expired remote appear online.
  dt7_tick = UINT32_MAX - 10;
  test_tick = 89;
  receive(0, -1000, 0, true);
  check_vectors(1, 0, 0);
  assert(!last_control_was_host);
  test_tick = 90;
  check_vectors(0, -1, 0);
  assert(last_control_was_host);
  test_tick = 190; // Both links now stale.
  stopped();
  test_spin_host();
  puts("PASS: host chassis control, DT7 priority, automatic handover and offline stop");
}
"""

with tempfile.TemporaryDirectory(prefix="host_chassis_control_") as directory:
    work = Path(directory)
    (work / "stm32h7xx_hal.h").write_text(hal_stub, encoding="utf-8")
    (work / "test.c").write_text(base + extra + functions + tests, encoding="utf-8")
    for remote_stop in (0, 1):
        executable = work / f"test_{remote_stop}.exe"
        subprocess.run([
            "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            f"-DTEST_REMOTE_STOP={remote_stop}", "-I", str(work),
            "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
            "-I", str(ROOT / "Skywalker/Device/Inc"),
            "-I", str(ROOT / "Skywalker/Application/Inc"),
            str(work / "test.c"),
            str(ROOT / "Skywalker/Application/Src/app_host_protocol.c"),
            str(ROOT / "Skywalker/Algorithm/Src/alg_swerve.c"),
            str(ROOT / "Skywalker/Algorithm/Src/alg_pid.c"),
            "-lm", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
