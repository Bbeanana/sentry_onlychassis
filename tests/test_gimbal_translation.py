"""Verify actual command routing/coordinate transform with the current swerve solver."""
from pathlib import Path
import re
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'Skywalker/Application/Src/app_robot.c').read_text()
def function(name):
    return re.search(r'^(?:static )?[^\n]*\b'+name+r'\([^{};]*\) \{.*?^\}',s,re.M|re.S).group()
config=re.search(r'^enum \{.*?^};',s,re.M|re.S).group()
state=s[s.index('typedef struct {'):s.index('static float RobotClamp')].replace('static void RobotBigYawControl(void);','')
names=('kRobotBigYawFeedbackTimeoutMs','kRobotChassisSpinCommandEpsilon','kRobotChassisSpinYawCompensationS','kRobotChassisSpinLateralScale','kRobotChassisSpinForwardScale','kRobotMaxVxMps','kRobotMaxVyMps','kRobotMaxOmegaRadps')
constants='\n'.join(re.search(r'^static const float '+n+r'\b.*?;',s,re.M|re.S).group() for n in names if n!='kRobotBigYawFeedbackTimeoutMs')
# Execute the actual kinematics entry through its frame selection and transform.
entry=function('RobotChassisKinematics').split('  float wz = desired_yaw_rate;')[0]+'''\n  g_robot.chassis.actual_velocity.vx=vx;
  g_robot.chassis.actual_velocity.vy=vy;
}\n'''
code=r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "app_host_protocol.h"
#include "app_gimbal_link.h"
#include "alg_swerve.h"
#define ROBOT_CHASSIS_NO_MOTOR_TX_TEST 0
typedef int Bmi088Status;
'''+config+state+constants+r'''
static uint32_t tick=200;
static GimbalImuYaw imu_sample;
static bool imu_valid=true;
bool GimbalLinkGetImuYaw(GimbalImuYaw *sample){*sample=imu_sample;return imu_valid;}
static bool RobotChassisRemoteVelocity(bool *spin){*spin=false;return false;}
'''+function('RobotClamp')+function('RobotWrapPi')+function('RobotChassisGimbalHeadingUpdate')+function('RobotChassisSpinTranslation')+entry+r'''
static void near(float a,float b){assert(fabsf(a-b)<0.00001f);}
static void reset(void){
 memset(&g_robot,0,sizeof(g_robot));g_robot.chassis.imu_ready=true;
 g_robot.chassis.gimbal_imu_heading_valid=true;
 imu_valid=true;imu_sample=(GimbalImuYaw){.sender_boot_id=42,.zero_generation=1};
 g_robot.yaw_feedback_valid=true;g_robot.yaw_feedback_tick=tick;
}
int main(void){
 HostVelocityCommand c={.vx=0.2f,.vy=0,.wz=0.8f};
 reset();g_robot.chassis_interboard_control_active=true;
 // Physical CCW relative yaw from paired IMU headings.
 g_robot.chassis.gimbal_imu_absolute_yaw=(float)M_PI/2;
 RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0);
 near(g_robot.chassis.actual_velocity.vy,0.2f*kRobotChassisSpinForwardScale);
 // Full-turn sweep: body motion rotated back to gimbal frame remains forward.
 for(int i=-180;i<=180;i+=5){
  float a=i*(float)M_PI/180;
  g_robot.chassis.gimbal_imu_absolute_yaw=a;RobotChassisKinematics(&c);
  float x=g_robot.chassis.actual_velocity.vx,y=g_robot.chassis.actual_velocity.vy;
  near(cosf(a)*x+sinf(a)*y,0.2f*kRobotChassisSpinForwardScale);
  near(-sinf(a)*x+cosf(a)*y,0);
  SwerveWheelVector wheels[4];SwerveResolve(x,y,c.wz,0.106066f,0.106066f,wheels);
  float mean_x=0,mean_y=0;for(int k=0;k<4;k++){mean_x+=wheels[k].vx/4;mean_y+=wheels[k].vy/4;}
  near(mean_x,x);near(mean_y,y);
 }
 // Both headings vary independently; world motion always follows the current gimbal IMU.
 for(int gi=-150;gi<=150;gi+=30) for(int bi=-180;bi<=180;bi+=30){
  float ga=gi*(float)M_PI/180, ba=bi*(float)M_PI/180;
  g_robot.chassis.gimbal_imu_absolute_yaw=ga;g_robot.chassis.chassis_imu_absolute_yaw=ba;
  g_robot.yaw_motor.state.angle=99; // motor encoder must not influence the direction
  RobotChassisKinematics(&c);
  float x=g_robot.chassis.actual_velocity.vx,y=g_robot.chassis.actual_velocity.vy;
  near(cosf(ba)*x-sinf(ba)*y,cosf(ga)*0.2f*kRobotChassisSpinForwardScale);
  near(sinf(ba)*x+cosf(ba)*y,sinf(ga)*0.2f*kRobotChassisSpinForwardScale);
 }
 g_robot.chassis.chassis_imu_absolute_yaw=0;
 // Measured chassis rate, not requested rate, drives the 7 ms prediction.
 g_robot.chassis.gimbal_imu_absolute_yaw=0;g_robot.chassis.yaw_rate=2;
 RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vy,-sinf(0.014f)*0.2f*kRobotChassisSpinForwardScale);
 // UART7 is already in body coordinates even during spin.
 g_robot.chassis_interboard_control_active=false;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0.2f*kRobotChassisSpinForwardScale);
 near(g_robot.chassis.actual_velocity.vy,0);
 // Straight driving uses the same relative angle, without accumulating an offset.
 g_robot.chassis_interboard_control_active=true;c.wz=0;
 g_robot.chassis.gimbal_imu_absolute_yaw=(float)M_PI/2;
 for(int i=0;i<100;i++)RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0);near(g_robot.chassis.actual_velocity.vy,0.2f);
 near(g_robot.chassis.gimbal_imu_absolute_yaw,(float)M_PI/2);
 // Strafe follows gimbal left; stop remains stop, pure spin remains pure spin.
 c.vx=0;c.vy=0.2f;c.wz=0.8f;g_robot.chassis.yaw_rate=0;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,-0.2f*kRobotChassisSpinLateralScale);
 near(g_robot.chassis.actual_velocity.vy,0);
 c.vy=0;RobotChassisKinematics(&c);near(g_robot.chassis.actual_velocity.vx,0);near(g_robot.chassis.actual_velocity.vy,0);
 c.wz=0;RobotChassisKinematics(&c);assert(g_robot.chassis.steering_idle);
 // Feedback loss blocks gimbal-relative translation, not UART7 body translation.
 c.vx=0.2f;c.wz=0.8f;g_robot.chassis.gimbal_imu_heading_valid=false;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0);near(g_robot.chassis.actual_velocity.vy,0);
 g_robot.chassis_interboard_control_active=false;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0.2f*kRobotChassisSpinForwardScale);
 reset();g_robot.chassis_interboard_control_active=true;g_robot.chassis.imu_ready=false;
 RobotChassisKinematics(&c);near(g_robot.chassis.actual_velocity.vx,0);
 // Paired startup origin only while disabled; a stationary gimbal and rotating base
 // must produce relative yaw opposite to chassis rotation.
 reset();g_robot.chassis.gimbal_imu_heading_valid=false;
 g_robot.chassis.yaw_angle=0.4f;imu_sample.yaw_delta_rad=0.2f;
 RobotChassisGimbalHeadingUpdate(true);assert(!g_robot.chassis.gimbal_imu_heading_valid);
 RobotChassisGimbalHeadingUpdate(false);assert(g_robot.chassis.gimbal_imu_heading_valid);
 near(g_robot.chassis.gimbal_imu_absolute_yaw,0);
 g_robot.chassis.yaw_angle=0.4f+(float)M_PI/2;
 RobotChassisGimbalHeadingUpdate(true);
 near(g_robot.chassis.gimbal_imu_absolute_yaw,0);
 near(g_robot.chassis.chassis_imu_absolute_yaw,(float)M_PI/2);
 // Both headings turn together: relative yaw returns to zero.
 imu_sample.yaw_delta_rad=0.2f+(float)M_PI/2;
 RobotChassisGimbalHeadingUpdate(true);near(g_robot.chassis.gimbal_imu_absolute_yaw,(float)M_PI/2);
 // Pairing survives wraparound without an extra revolution; source age is compensated.
 imu_sample.yaw_delta_rad=0.2f+(float)M_PI/2-2*(float)M_PI;
 RobotChassisGimbalHeadingUpdate(true);near(g_robot.chassis.gimbal_imu_absolute_yaw,(float)M_PI/2);
 imu_sample.yaw_rate_rad_s=1;imu_sample.source_age_ms=20;
 RobotChassisGimbalHeadingUpdate(true);near(g_robot.chassis.gimbal_imu_absolute_yaw,(float)M_PI/2+0.02f);
 // Cloud data dropouts never change the fixed frame, including recovery while disabled.
 // Set a different gimbal heading from chassis heading so re-zeroing both would fail.
 imu_sample.yaw_delta_rad=0.2f+(float)M_PI/6;
 imu_sample.yaw_rate_rad_s=0;imu_sample.source_age_ms=0;
 g_robot.chassis.yaw_angle=0.4f+(float)M_PI/2;
 RobotChassisGimbalHeadingUpdate(true);
 float original_gimbal_ref=g_robot.chassis.gimbal_imu_reference_delta;
 float original_body_ref=g_robot.chassis.gimbal_imu_chassis_reference_yaw;
 c=(HostVelocityCommand){.vx=0.2f,.wz=0.8f};
 g_robot.chassis_interboard_control_active=true;
 RobotChassisKinematics(&c);
 float before_x=g_robot.chassis.actual_velocity.vx,before_y=g_robot.chassis.actual_velocity.vy;
 for(int stopped=0;stopped<2;stopped++){
  imu_valid=false;RobotChassisGimbalHeadingUpdate(!stopped);
  assert(!g_robot.chassis.gimbal_imu_heading_valid);
  assert(g_robot.chassis.gimbal_imu_origin_initialized);
  imu_valid=true;RobotChassisGimbalHeadingUpdate(!stopped);
  assert(g_robot.chassis.gimbal_imu_heading_valid);
  near(g_robot.chassis.gimbal_imu_reference_delta,original_gimbal_ref);
  near(g_robot.chassis.gimbal_imu_chassis_reference_yaw,original_body_ref);
  near(g_robot.chassis.gimbal_imu_absolute_yaw,(float)M_PI/6);
  near(g_robot.chassis.chassis_imu_absolute_yaw,(float)M_PI/2);
  RobotChassisKinematics(&c);
  near(g_robot.chassis.actual_velocity.vx,before_x);
  near(g_robot.chassis.actual_velocity.vy,before_y);
 }
 // New real IMU headings received after the dropout still determine the current direction.
 imu_valid=false;RobotChassisGimbalHeadingUpdate(true);
 imu_valid=true;imu_sample.yaw_delta_rad=0.2f+(float)M_PI/3;
 RobotChassisGimbalHeadingUpdate(false);RobotChassisKinematics(&c);
 near(g_robot.chassis.gimbal_imu_absolute_yaw,(float)M_PI/3);
 near(g_robot.chassis.actual_velocity.vx,sinf((float)M_PI/3)*0.2f*kRobotChassisSpinForwardScale);
 near(g_robot.chassis.actual_velocity.vy,-cosf((float)M_PI/3)*0.2f*kRobotChassisSpinForwardScale);
 // Ordinary disable/re-enable must preserve the exact same world frame, too.
 for(int i=0;i<100;i++)RobotChassisGimbalHeadingUpdate(false);
 near(g_robot.chassis.gimbal_imu_reference_delta,original_gimbal_ref);
 near(g_robot.chassis.gimbal_imu_chassis_reference_yaw,original_body_ref);
 // Sender reboot is a coordinate discontinuity; even DOWN must not silently assume alignment.
 imu_sample.sender_boot_id=43;
 RobotChassisGimbalHeadingUpdate(true);assert(!g_robot.chassis.gimbal_imu_heading_valid);
 RobotChassisGimbalHeadingUpdate(false);assert(!g_robot.chassis.gimbal_imu_heading_valid);
 near(g_robot.chassis.gimbal_imu_reference_delta,original_gimbal_ref);
 puts("PASS: fixed IMU origin across dropout/rearm, reboot invalidation, actual kinematics routing, full yaw sweep, swerve vectors, prediction, strafe, stop and feedback failures");
}
'''
with tempfile.TemporaryDirectory(prefix='gimbal_translation_') as d:
    p=Path(d);(p/'test.c').write_text(code)
    subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror',*[f'-I{root}/Skywalker/{x}/Inc' for x in ('Algorithm','Device','Application')],str(p/'test.c'),str(root/'Skywalker/Algorithm/Src/alg_swerve.c'),'-lm','-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
