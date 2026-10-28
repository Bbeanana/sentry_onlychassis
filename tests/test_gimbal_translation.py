"""Run the real encoder frame transform, command routing and swerve resolver."""
from pathlib import Path
import re
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root/'Skywalker/Application/Src/app_robot.c').read_text()
def function(name):
    return re.search(r'^(?:static )?[^\n]*\b'+name+r'\([^{};]*\) \{.*?^\}',s,re.M|re.S).group()
config = re.search(r'^enum \{.*?^};',s,re.M|re.S).group()
state = s[s.index('typedef struct {'):s.index('static float RobotClamp')].replace('static void RobotBigYawControl(void);','')
names = ('kRobotChassisSpinCommandEpsilon',
         'kRobotChassisSpinLateralScale','kRobotChassisSpinForwardScale',
         'kRobotMaxVxMps','kRobotMaxVyMps','kRobotMaxOmegaRadps',
         'kRobotBigYawGyroDirection','kRobotBigYawGyroRatio','kRobotBigYawForwardZeroRad')
constants = '\n'.join(re.search(r'^static const float '+n+r'\b.*?;',s,re.M|re.S).group() for n in names)
entry = function('RobotChassisKinematics').split('  float wz = desired_yaw_rate;')[0]+'''\n  g_robot.chassis.actual_velocity.vx=vx;
  g_robot.chassis.actual_velocity.vy=vy;
}\n'''
code = r'''
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
static uint32_t tick=200,irq_mask=0;
static uint32_t HAL_GetTick(void){return tick;}
static uint32_t __get_PRIMASK(void){return irq_mask;}
static void __disable_irq(void){irq_mask=1;}
static void __set_PRIMASK(uint32_t v){irq_mask=v;}
static bool RobotChassisRemoteVelocity(bool *spin){*spin=false;return false;}
'''+function('RobotClamp')+function('RobotWrapPi')+function('RobotBigYawEncoderUpdate')+function('RobotChassisGimbalHeadingUpdate')+function('RobotChassisSpinTranslation')+entry+r'''
static void near(float a,float b){assert(fabsf(a-b)<0.00001f);}
static void pose(float relative_yaw,float relative_rate){
 g_robot.yaw_feedback_valid=true;g_robot.yaw_feedback_tick=tick;
 g_robot.yaw_motor.state.angle=kRobotBigYawForwardZeroRad+
     relative_yaw*kRobotBigYawGyroDirection*kRobotBigYawGyroRatio;
 g_robot.big_yaw_encoder_initialized=true;
 g_robot.big_yaw_encoder_previous_raw=g_robot.yaw_motor.state.angle;
 g_robot.big_yaw_encoder_wraps=0;
 g_robot.big_yaw_encoder_continuous_rad=g_robot.yaw_motor.state.angle;
 g_robot.yaw_motor.state.omega=relative_rate*kRobotBigYawGyroDirection*kRobotBigYawGyroRatio;
 g_robot.yaw_motor.state.error_code=0;
 RobotChassisGimbalHeadingUpdate(true);assert(irq_mask==0);
 assert(g_robot.chassis.big_yaw_heading_valid);
}
int main(void){
 HostVelocityCommand c={.vx=0.2f,.vy=0,.wz=0.8f};
 g_robot.chassis_interboard_control_active=true;
 // Sweep past one full revolution on either side, including wrapping and both installation signs.
 for(int i=-720;i<=720;i+=5){
  const float angle=i*(float)M_PI/180;
  pose(angle,0);RobotChassisKinematics(&c);
  const float x=g_robot.chassis.actual_velocity.vx,y=g_robot.chassis.actual_velocity.vy;
  near(x,cosf(angle)*0.2f*kRobotChassisSpinForwardScale);
  near(y,sinf(angle)*0.2f*kRobotChassisSpinForwardScale);
  near(cosf(angle)*x+sinf(angle)*y,0.2f*kRobotChassisSpinForwardScale);
  near(-sinf(angle)*x+cosf(angle)*y,0);
  SwerveWheelVector wheels[4];SwerveResolve(x,y,c.wz,0.106066f,0.106066f,wheels);
  float mean_x=0,mean_y=0;
  for(int k=0;k<4;k++){mean_x+=wheels[k].vx/4;mean_y+=wheels[k].vy/4;}
  near(mean_x,x);near(mean_y,y);
 }
 // Real protocol wrapping, forward and reverse, many complete turns.
 g_robot.big_yaw_encoder_initialized=false;
 const double origin=kRobotBigYawForwardZeroRad;
 for(int segment=0;segment<2;++segment){
  for(int i=0;i<=20000;++i){
   const double physical=origin+(segment==0?i:20000-i)*0.01;
   double raw=fmod(physical+12.5,25.0);if(raw<0)raw+=25.0;raw-=12.5;
   g_robot.yaw_motor.state.angle=(float)raw;
   g_robot.yaw_motor.state.error_code=0;
   g_robot.yaw_feedback_tick=tick;
   RobotBigYawEncoderUpdate();RobotChassisGimbalHeadingUpdate(true);
   const double rel=kRobotBigYawGyroDirection*(physical-origin)/kRobotBigYawGyroRatio;
   assert(fabs(sin(rel)-sinf(g_robot.chassis.big_yaw_relative_yaw_rad))<0.00001);
   assert(fabs(cos(rel)-cosf(g_robot.chassis.big_yaw_relative_yaw_rad))<0.00001);
   RobotChassisKinematics(&c);
   near(g_robot.chassis.actual_velocity.vx,cos(rel)*0.2*kRobotChassisSpinForwardScale);
   near(g_robot.chassis.actual_velocity.vy,sin(rel)*0.2*kRobotChassisSpinForwardScale);
  }
 }
 assert(g_robot.big_yaw_encoder_wraps==0);
 // Cross the negative boundary too, then return through it.
 for(int segment=0;segment<2;++segment){
  for(int i=0;i<=20000;++i){
   const double physical=origin-(segment==0?i:20000-i)*0.01;
   double raw=fmod(physical+12.5,25.0);if(raw<0)raw+=25.0;raw-=12.5;
   g_robot.yaw_motor.state.angle=(float)raw;
   RobotBigYawEncoderUpdate();RobotChassisGimbalHeadingUpdate(false);
   const double rel=kRobotBigYawGyroDirection*(physical-origin)/kRobotBigYawGyroRatio;
   assert(fabs(sin(rel)-sinf(g_robot.chassis.big_yaw_relative_yaw_rad))<0.00001);
   assert(fabs(cos(rel)-cosf(g_robot.chassis.big_yaw_relative_yaw_rad))<0.00001);
  }
 }
 assert(g_robot.big_yaw_encoder_wraps==0);
 // Neither accumulated IMU angle nor the base's measured rate changes the encoder frame.
 // Repeat for simulated 10 minutes while both IMU headings accumulate large errors.
 for(int i=0;i<6000;i++){
  tick+=100;g_robot.chassis.yaw_angle=i*1.11f;
  g_robot.chassis.yaw_rate=(i%2)?30:-30;g_robot.chassis.imu_ready=false;
  pose(0.4f,0);RobotChassisKinematics(&c);
  near(g_robot.chassis.actual_velocity.vx,cosf(0.4f)*0.2f*kRobotChassisSpinForwardScale);
  near(g_robot.chassis.actual_velocity.vy,sinf(0.4f)*0.2f*kRobotChassisSpinForwardScale);
 }
 // Even during spin, theta is the encoder position without rate/age prediction.
 pose(0,-2);RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vy,0);
 tick+=10;RobotChassisGimbalHeadingUpdate(true);RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vy,0);
 // Straight driving and strafe use the same calibrated encoder frame.
 c.wz=0;pose((float)M_PI/2,0);RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0);near(g_robot.chassis.actual_velocity.vy,0.2f);
 c.vx=0;c.vy=0.2f;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,-0.2f);near(g_robot.chassis.actual_velocity.vy,0);
 c.wz=0.8f;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,-0.2f*kRobotChassisSpinLateralScale);
 c.vy=0;RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0);near(g_robot.chassis.actual_velocity.vy,0);
 c.wz=0;RobotChassisKinematics(&c);assert(g_robot.chassis.steering_idle);
 // No automatic zero capture at any initial angle, on disable, or on re-enable.
 pose(0.7f,0);const float raw=g_robot.yaw_motor.state.angle;
 for(int i=0;i<100;i++)RobotChassisGimbalHeadingUpdate(false);
 near(g_robot.chassis.big_yaw_relative_yaw_rad,0.7f);
 memset(&g_robot.chassis,0,sizeof(g_robot.chassis));RobotChassisGimbalHeadingUpdate(true);
 near(g_robot.chassis.big_yaw_relative_yaw_rad,0.7f);near(g_robot.chassis.big_yaw_encoder_angle_rad,raw);
 // Stale/missing/faulted/non-finite encoder data blocks translation but preserves chassis spin request.
 c=(HostVelocityCommand){.vx=0.2f,.wz=0.8f};
 for(int failure=0;failure<5;failure++){
  pose(0.4f,0);
  if(failure==0)g_robot.yaw_feedback_valid=false;
  if(failure==1)tick+=kRobotBigYawFeedbackTimeoutMs+1;
  if(failure==2)g_robot.yaw_motor.state.angle=NAN;
  if(failure==3)g_robot.yaw_motor.state.omega=INFINITY;
  if(failure==4)g_robot.yaw_motor.state.error_code=8;
  RobotChassisGimbalHeadingUpdate(true);assert(!g_robot.chassis.big_yaw_heading_valid);
  RobotChassisKinematics(&c);
  near(g_robot.chassis.actual_velocity.vx,0);near(g_robot.chassis.actual_velocity.vy,0);
  near(g_robot.chassis.desired_velocity.omega_z,0.8f);
 }
 // Fresh feedback restores the current actual angle, without assuming alignment.
 pose(-0.6f,0);RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vy,sinf(-0.6f)*0.2f*kRobotChassisSpinForwardScale);
 // UART7 body-frame input does not depend on yaw feedback or its position.
 g_robot.chassis_interboard_control_active=false;g_robot.chassis.big_yaw_heading_valid=false;
 RobotChassisKinematics(&c);
 near(g_robot.chassis.actual_velocity.vx,0.2f*kRobotChassisSpinForwardScale);
 near(g_robot.chassis.actual_velocity.vy,0);
 puts("PASS: absolute encoder frame, fixed zero/sign, full rotations, IMU drift independence, rearm, feedback faults and actual swerve routing");
}
'''
with tempfile.TemporaryDirectory(prefix='encoder_translation_') as directory:
    work=Path(directory)
    for zero,sign in ((0.0,-1.0),(1.234,-1.0),(1.234,1.0)):
        selected=re.sub(r'(kRobotBigYawForwardZeroRad = )[^;]+',rf'\g<1>{zero}f',code)
        selected=re.sub(r'(kRobotBigYawGyroDirection = )[^;]+',rf'\g<1>{sign}f',selected)
        (work/'test.c').write_text(selected)
        subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror',
                        *[f'-I{root}/Skywalker/{x}/Inc' for x in ('Algorithm','Device','Application')],
                        str(work/'test.c'),str(root/'Skywalker/Algorithm/Src/alg_swerve.c'),
                        '-lm','-o',str(work/'test')],check=True)
        subprocess.run([str(work/'test')],check=True)
