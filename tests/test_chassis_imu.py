"""Run the real IMU update -> spin transform -> kinematics with mocked SPI data."""

from pathlib import Path
import re
import subprocess
import tempfile

from test_swerve_port import ROOT, function, harness, source, constant

base = harness.split("static void test_vectors(void)")[0]
base = base.replace(
    "bool chassis_steering_vector_active[4];",
    "uint32_t chassis_imu_last_tick;\n  bool chassis_steering_vector_active[4];",
).replace(
    "float spin_translation_relative_yaw;",
    "float spin_translation_relative_yaw, control_dt;\n"
    "    uint32_t imu_sample_count, imu_error_count;\n    int imu_status;",
)
axis = re.search(r"kRobotImuGyroZAxis\s*=\s*(\d+)", source).group(1)
mock = r"""
typedef int Bmi088Status;
enum { kBmi088NoError = 0, kBmi088NoSensor = 255,
       kRobotImuGyroZAxis = AXIS };
static uint32_t now, read_calls;
static int read_status;
static struct {
  bool is_inited, is_gyro_offset_inited, is_heater_enabled;
  float corrected_gyro[3];
} g_bmi088;
static uint32_t osKernelGetTickCount(void) { return now; }
static int Bmi088Update(void *object) {
  assert(object == &g_bmi088);
  ++read_calls;
  return read_status;
}
static float Bmi088HeaterControl(void *object) {
  assert(object == &g_bmi088);
  return 0;
}
""".replace("AXIS", axis)

tests = r"""
int main(void) {
  reset_robot();
  now = 1000;
  RobotChassisImuUpdate();
  assert(!g_robot.chassis.imu_ready && read_calls == 0);
  g_bmi088.is_inited = true;
  RobotChassisImuUpdate();
  assert(!g_robot.chassis.imu_ready && read_calls == 0);
  // Never run an uncalibrated gyro or start calibration while moving.
  g_bmi088.is_gyro_offset_inited = true;
  g_bmi088.corrected_gyro[kRobotImuGyroZAxis] = 1.5f;
  RobotChassisImuUpdate();
  assert(g_robot.chassis.imu_ready && read_calls == 1);
  assert(g_robot.chassis.imu_sample_count == 1);
  near(g_robot.chassis.yaw_angle, 0, 0.00001f);
  near(g_robot.chassis.yaw_rate, kRobotImuYawRateSign * 1.5f, 0.00001f);

  // Exercise the measured gyro through the full application transform.
  HostVelocityCommand command = {.vx = 1, .wz = 1};
  g_bmi088.corrected_gyro[kRobotImuGyroZAxis] = 0;
  RobotChassisImuUpdate();
  RobotChassisKinematics(&command);
  near(g_robot.chassis.actual_velocity.vx, 0.3f, 0.00001f);
  near(g_robot.chassis.actual_velocity.vy, 0, 0.00001f);
  g_bmi088.corrected_gyro[kRobotImuGyroZAxis] =
      (float)M_PI / 2 / kRobotImuYawRateSign;
  now += 1000;
  RobotChassisImuUpdate();
  RobotChassisKinematics(&command);
  near(g_robot.chassis.yaw_angle, (float)M_PI / 2, 0.00001f);
  near(g_robot.chassis.spin_translation_relative_yaw,
       (float)M_PI / 2 * 1.005f, 0.00001f);
  // The host's input is already body-relative: use only measured-rate lead,
  // without rotating by the integrated heading.
  float lead = (float)M_PI / 2 * kRobotChassisSpinYawCompensationS;
  near(g_robot.chassis.actual_velocity.vx, 0.3f * cosf(lead), 0.00001f);
  near(g_robot.chassis.actual_velocity.vy, -0.3f * sinf(lead), 0.00001f);
  float previous_yaw = g_robot.chassis.yaw_angle;

  read_status = kBmi088NoSensor;
  now += 2;
  RobotChassisImuUpdate();
  RobotChassisKinematics(&command);
  assert(!g_robot.chassis.imu_ready);
  assert(!g_robot.chassis.spin_translation_active);
  assert(g_robot.chassis.imu_error_count == 1);
  near(g_robot.chassis.yaw_angle, previous_yaw, 0.00001f);
  near(g_robot.chassis.actual_velocity.vx, 0, 0.00001f);
  near(g_robot.chassis.actual_velocity.vy, 0, 0.00001f);
  near(g_robot.chassis.desired_velocity.omega_z, 1, 0.00001f);

  // Recovery must retry the sensor even though imu_ready was cleared. The
  // unknown heading gap must not be integrated at the new sample's rate.
  uint32_t previous_reads = read_calls;
  uint32_t previous_samples = g_robot.chassis.imu_sample_count;
  now += 3000;
  read_status = kBmi088NoError;
  g_bmi088.corrected_gyro[kRobotImuGyroZAxis] = 4 / kRobotImuYawRateSign;
  RobotChassisImuUpdate();
  assert(read_calls == previous_reads + 1);
  assert(g_robot.chassis.imu_ready);
  assert(g_robot.chassis.imu_sample_count == previous_samples + 1);
  near(g_robot.chassis.yaw_angle, previous_yaw, 0.00001f);
  RobotChassisKinematics(&command);
  near(g_robot.chassis.spin_translation_reference_yaw, previous_yaw, 0.00001f);
  assert(g_robot.chassis.spin_translation_active);
  now += 2;
  RobotChassisImuUpdate();
  near(g_robot.chassis.yaw_angle, previous_yaw + 0.008f, 0.00001f);

  // Invalid numeric samples must never reach angle wrapping/trigonometry.
  g_bmi088.corrected_gyro[kRobotImuGyroZAxis] = NAN;
  now += 2;
  RobotChassisImuUpdate();
  assert(!g_robot.chassis.imu_ready && g_robot.chassis.imu_error_count == 2);
  assert(isfinite(g_robot.chassis.yaw_angle));
  puts("PASS: gyro integration, live spin transform, error recovery and invalid-IMU translation blocking");
}
"""

with tempfile.TemporaryDirectory(prefix="chassis_imu_") as directory:
    work = Path(directory)
    test_c = work / "test.c"
    executable = work / "test.exe"
    test_c.write_text(
        base + mock + constant("kRobotImuYawRateSign") + "\n" +
        function("RobotChassisImuUpdate") + tests, encoding="utf-8",
    )
    subprocess.run([
        "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
        "-I", str(ROOT / "Skywalker/Device/Inc"),
        "-I", str(ROOT / "Skywalker/Application/Inc"),
        str(test_c), str(ROOT / "Skywalker/Algorithm/Src/alg_swerve.c"),
        "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
