"""Real UART angle receiver -> real kinematics/motor targets and status sender."""
from pathlib import Path
import subprocess
import tempfile

from test_swerve_port import ROOT, harness, function, source
from test_gimbal_link import stub

assert "UartInit(&huart10, GimbalLinkRxCallback);" in source
assert "_Static_assert(!(kRobotGimbalLinkEnabled && kRobotVisionEnabled)" in source
assert "RobotSendGimbalLinkStatus();" in source
base = harness.split("static void test_vectors(void)")[0]
base = base.replace("bool chassis_steering_vector_active[4];",
                    "bool chassis_steering_vector_active[4];\n"
                    "  bool chassis_host_control_active, chassis_drive_steering_ready;")
extra = r"""
#include "stm32h7xx_hal.h"
uint32_t tick = 100, primask;
enum { kRobotGimbalLinkEnabled = 1, HAL_UART_STATE_READY = 0 };
static struct { int gState; } huart10;
static BoardLinkMessage last_status;
static unsigned status_count;
static void UartTransmit(void *uart, uint8_t *frame, uint16_t size) {
  assert(uart == &huart10 && size == kBoardLinkFrameSize);
  assert(BoardLinkDecode(frame, &last_status));
  assert(last_status.type == kBoardLinkStatus);
  ++status_count;
}
static uint8_t sequence;
static void feedback(float angle, bool valid) {
  uint8_t frame[kBoardLinkFrameSize];
  assert(BoardLinkEncodeYaw(angle, valid, sequence++, frame));
  GimbalLinkRxCallback(frame, 3);
  GimbalLinkRxCallback(frame + 3, sizeof(frame)-3);
}
static void check_remote_vector(float vx, float vy) {
  for (int pass=0; pass<2; ++pass) {
    RobotChassisKinematics(NULL);
    for (int i=0; i<4; ++i)
      g_robot.chassis_motor_6020[i].state.angle =
          g_robot.chassis_steering_requested_angle[i];
  }
  near(g_robot.chassis.actual_velocity.vx, vx, 0.0011f);
  near(g_robot.chassis.actual_velocity.vy, vy, 0.0011f);
  for (int i=0; i<4; ++i) {
    float heading=RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                              kRobotSteeringForwardAngleRad[i]) /
                  kRobotSteeringDirection[i];
    float speed=g_robot.chassis_motor_3508[i].state.given_omega *
                kRobotWheelRadiusM / kRobotDriveReductionRatio /
                kRobotDriveDirection[i];
    near(cosf(heading)*speed,vx,0.0011f);
    near(sinf(heading)*speed,vy,0.0011f);
  }
}
"""
tests = r"""
int main(void) {
  reset_robot(); GimbalLinkInit();
  g_dt7_object.rocker.ch3=132; // 1 m/s in gimbal forward
  check_remote_vector(0,0); // startup without relative yaw blocks translation
  const float angles[]={0,1.5707963f,-1.5707963f,3.1415926f,-3.1415926f};
  for (unsigned i=0;i<5;++i) {
    feedback(angles[i],true);
    check_remote_vector(cosf(angles[i]),sinf(angles[i]));
  }
  // A means gimbal left; with gimbal facing chassis left this points backward.
  g_dt7_object.rocker.ch3=0; g_dt7_object.key.a=true;
  feedback(1.5707963f,true);
  check_remote_vector(-5,0);
  g_dt7_object.key.a=false; g_dt7_object.rocker.ch3=132;
  tick+=51; check_remote_vector(0,0);
  feedback(0,true); check_remote_vector(1,0);
  feedback(0,false); check_remote_vector(0,0);
  feedback(-1.5707963f,true); check_remote_vector(0,-1);

  // A moving gimbal changes the reference immediately, including no-spin mode.
  feedback(0.6f,true); check_remote_vector(cosf(0.6f),sinf(0.6f));
  g_robot.chassis.imu_ready=true;
  g_dt7_object.rocker.wheel=100;
  g_robot.chassis.yaw_rate=2;
  feedback(-1.5707963f,true);
  for (unsigned i=0;i<4;++i) {
    // Accumulated body yaw must never be applied a second time.
    g_robot.chassis.yaw_angle=(float)i;
    RobotChassisKinematics(NULL);
    float angle=-1.571f-0.01f;
    near(g_robot.chassis.actual_velocity.vx,0.3f*cosf(angle),0.00001f);
    near(g_robot.chassis.actual_velocity.vy,0.3f*sinf(angle),0.00001f);
  }
  // With no angle, independent rotation remains available but translation stops.
  tick+=51;
  RobotChassisKinematics(NULL);
  near(g_robot.chassis.actual_velocity.vx,0,0.00001f);
  near(g_robot.chassis.actual_velocity.vy,0,0.00001f);
  assert(g_robot.chassis.desired_velocity.omega_z != 0);

  // UART7 body commands neither require nor rotate with the gimbal feedback.
  HostVelocityCommand host={.vx=1,.vy=0};
  RobotChassisKinematics(&host);
  near(g_robot.chassis.actual_velocity.vx,1,0.00001f);
  near(g_robot.chassis.actual_velocity.vy,0,0.00001f);
  feedback(1.57f,true); RobotChassisKinematics(&host);
  near(g_robot.chassis.actual_velocity.vx,1,0.00001f);
  host.wz=1; RobotChassisKinematics(&host);
  near(g_robot.chassis.actual_velocity.vx,0.3f*cosf(0.01f),0.00001f);
  near(g_robot.chassis.actual_velocity.vy,-0.3f*sinf(0.01f),0.00001f);

  g_robot.chassis.yaw_rate=-2;
  g_robot.chassis_host_control_active=true;
  g_robot.chassis_drive_steering_ready=true;
  RobotSendGimbalLinkStatus();
  assert(last_status.value==-2000 && last_status.flags==15);
  uint8_t last_sequence=last_status.sequence;
  huart10.gState=1; RobotSendGimbalLinkStatus(); assert(status_count==1);
  huart10.gState=0; tick+=51;
  g_robot.chassis.imu_ready=false;
  g_robot.chassis_host_control_active=false;
  g_robot.chassis_drive_steering_ready=false;
  RobotSendGimbalLinkStatus();
  assert(last_status.value==0 && last_status.flags==0);
  assert(last_status.sequence==(uint8_t)(last_sequence+1));
  puts("PASS: real gimbal UART to wheel targets, stale stop, body-host isolation and status TX");
}
"""

with tempfile.TemporaryDirectory(prefix="gimbal_chassis_") as directory:
    work=Path(directory)
    (work/"stm32h7xx_hal.h").write_text(stub)
    (work/"test.c").write_text(base+extra+function("RobotSendGimbalLinkStatus")+tests)
    exe=work/"test"
    subprocess.run([
        "gcc","-std=gnu11","-Wall","-Wextra","-Werror","-DTEST_REAL_GIMBAL_LINK",
        "-I",str(work),"-I",str(ROOT/"Skywalker/Algorithm/Inc"),
        "-I",str(ROOT/"Skywalker/Application/Inc"),"-I",str(ROOT/"Skywalker/Device/Inc"),
        str(work/"test.c"),str(ROOT/"Skywalker/Algorithm/Src/alg_swerve.c"),
        str(ROOT/"Skywalker/Algorithm/Src/alg_board_link.c"),
        str(ROOT/"Skywalker/Application/Src/app_gimbal_link.c"),"-lm","-o",str(exe),
    ],check=True)
    subprocess.run([str(exe)],check=True)
