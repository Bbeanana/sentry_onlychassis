/**
 * @file app_robot.c
 * @brief 哨兵机器人的业务控制逻辑
 */

#include "app_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "alg_swerve.h"
#include "app_gimbal_link.h"
#include "app_host_protocol.h"
#include "app_protocol.h"
#include "app_referee.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "dev_bmi088.h"
#include "dev_dji_dt7.h"
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// 哨兵机器人的Skywalker实现，集中覆盖四个RTOS任务入口。

// 仅用于不接电机时观察CAN指令；接回电机前必须改为0。
// 可用编译参数 -DROBOT_CHASSIS_NO_MOTOR_TX_TEST=0 覆盖。
#ifndef ROBOT_CHASSIS_NO_MOTOR_TX_TEST
#define ROBOT_CHASSIS_NO_MOTOR_TX_TEST 0
#endif

// VOFA 调参观测的 GM6020 CAN ID（1~4）；也可用编译参数覆盖。
#ifndef ROBOT_GM6020_TELEMETRY_ID
#define ROBOT_GM6020_TELEMETRY_ID 1
#endif
#if ROBOT_GM6020_TELEMETRY_ID < 1 || ROBOT_GM6020_TELEMETRY_ID > 4
#error "ROBOT_GM6020_TELEMETRY_ID must be in [1, 4]"
#endif

enum {
  // 1U: 接入裁判系统；0U: 不接入裁判系统，使用本文件内默认参数。
  kRobotRefereeEnabled = 0U,
  // 四组舵轮底盘：M3508 ID 1~4，GM6020 ID 1~4。
  kRobotChassisEnabled = 1U,
  // DT7在线时优先遥控；1U允许DT7离线时由UART7接管，0U离线停车。
  kRobotChassisHostControlEnabled = 1U,
  // 非零旋转指令时启用 5ms 预测和小陀螺平移降速。
  kRobotChassisSpinTranslationEnabled = 1U,
  // USART10 用于云台相对角/底盘状态通信，与视觉串口互斥。
  kRobotGimbalLinkEnabled = 1U,
  kRobotGimbalEnabled = 0U,
  kRobotShootEnabled = 0U,
  kRobotVisionEnabled = 0U,
  kRobotKeyboardMouseEnabled = 1U,
  kRobotRemoteSwitchSafeModeEnabled = 0U,
  kRobotManagerPeriodMs = 10,
  kRobotGimbalPeriodMs = 2,
  kRobotChassisPeriodMs = 2,
  kRobotShootPeriodMs = 2,
  kRobotChassisMotorCount = 4U,
  kRobotCanDataLength = 8,
  kRobotDt7OfflineTimeoutMs = 100,
  kRobotHostCommandTimeoutMs = 100,
  kRobotChassisNoMotorTxTest = ROBOT_CHASSIS_NO_MOTOR_TX_TEST,
  // 平移输入归中时仍允许左拨轮原地旋转；设为0也允许QE/小陀螺原地旋转。
  kRobotSteeringIdleOverridesYaw = 1U,
  kRobotImuGyroZAxis = 2,
};

_Static_assert(!(kRobotGimbalLinkEnabled && kRobotVisionEnabled),
               "USART10 cannot serve gimbal link and vision simultaneously");

// 俯视底盘坐标：+X 向前，+Y 向左，正转角为逆时针。
// 默认方形底盘，沿用原0.15m中心到轮距离；必须按实际轮距修改。
static const float kRobotChassisHalfLengthM = 0.106066f;
static const float kRobotChassisHalfWidthM = 0.106066f;
static const float kRobotWheelRadiusM = 0.01f;
// 沿用原速度换算比例；若上面填实际轮半径，此处填电机到轮子的减速比。
static const float kRobotDriveReductionRatio = 1.0f;
static const float kRobotChassisTranslationDeadband = 0.05f;
static const float kRobotChassisTranslationStartDeadband = 0.08f;
// ID 顺序：左前、右前、左后、右后；以下编码值对应轮子朝 +X（车头）。
static const float kRobotSteeringForwardAngleRad[4] = {
    3060.0f / 8192.0f * 2.0f * (float)M_PI,
    2421.0f / 8192.0f * 2.0f * (float)M_PI,
    2383.0f / 8192.0f * 2.0f * (float)M_PI,
    3097.0f / 8192.0f * 2.0f * (float)M_PI,
};
// 四个 GM6020 均倒置安装：相对前向零位的编码器角度偏移与轮组舵角相反。
// 平移、原地旋转和混合运动统一使用此安装符号，零位编码值保持不变。
static const float kRobotSteeringDirection[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
static const float kRobotDriveDirection[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
static const float kRobotMaxVxMps = 5.0f;
static const float kRobotMaxVyMps = 5.0f;
// 四轮补偿后等比例限速，保持原直行速度上限，避免斜向合速度超限。
static const float kRobotChassisMaxWheelSpeedMps = 5.0f;
static const float kRobotSteeringCosineDeadbandRad = 0.05f;
static const float kRobotMaxOmegaRadps = 10.0f;
static const float kRobotChassisWheelMaxOmegaRadps = -14.0f;
static const float kRobotNoRefereeCurrentScale = 0.5f;
static const float kRobotChassisYawRateFeedbackKp = 0.5f;
static const float kRobotChassisAutoSpinOmegaRadps = 4.0f;
// 齐奇小陀螺：5 ms 航向前馈，横移/纵移分别保留 40%/30% 平移速度。
static const float kRobotChassisSpinYawCompensationS = 0.005f;
static const float kRobotChassisSpinLateralScale = 0.4f;
static const float kRobotChassisSpinForwardScale = 0.3f;
static const float kRobotChassisSpinCommandEpsilon = 0.0001f;
static const float kRobotChassisYawWheelDeadband = 0.05f;
static const float kRobotImuYawRateSign = 1.0f;
static const float kRobotGimbalYawFieldKp = 6.0f;
static const float kRobotGimbalYawMotorSign = 1.0f;
static const float kRobotGimbalYawRcSign = -1.0f;
static const float kRobotGimbalYawMaxOmegaRadps = 3.0f;
static const float kRobotGimbalYawDeadband = 0.05f;
static const float kRobotGimbalPitchRcSign = 1.0f;
static const float kRobotGimbalPitchMinRad = -(float)M_PI / 4.0f;
static const float kRobotGimbalPitchMaxRad = (float)M_PI / 4.0f;
static const float kRobotGimbalPitchMaxOmegaRadps = 1.5f;
static const float kRobotGimbalPitchDeadband = 0.05f;
static const float kRobotMouseYawRadPerCount = 0.0015f;
static const float kRobotMousePitchRadPerCount = 0.0008f;
static const float kRobotImuHeaterTargetTemperatureC = 40.0f;
static const float kRobotPitchOffsetRad = -1.66286434f;
static const float kRobotBulletFeedOmegaRadps = 910.0f;
static const float kRobotFrictionOmegaRadps = 800.0f;
static const float kRobotDefaultChassisPowerW = 100.0f;

// M3508底盘功率估计参数，保留旧车测试得到的基础模型。
static const float kRobotPowerK0 = 0.30f;
static const float kRobotPowerK1 = 0.37f;
static const float kRobotPowerK2 = 8.30f;
static const float kRobotPowerK3 = 0.0f;

// 启动前先对齐；普通运动使用误差滞回，小陀螺运行中使用每轮 cos³ 降速。
static const float kRobotSteeringDriveEnableErrorRad = 0.10f;
static const float kRobotSteeringDriveDisableErrorRad = 0.40f;
// 合速度接近零时 atan2 对噪声非常敏感，使用滞回保持上一舵角。
static const float kRobotSteeringVectorHoldSpeedMps = 0.05f;
static const float kRobotSteeringVectorResumeSpeedMps = 0.10f;
// 舵角目标按位置环允许的最大速度推进，并在 180 度附近锁定旋转方向。
static const float kRobotSteeringTargetRateMaxRadps = 4.0f;
static const float kRobotSteeringDirectionLockBandRad = 0.10f;
enum { kRobotChassisFeedbackTimeoutMs = 50U };

static PidConfig g_robot_bmi088_heater_pid = {
    .kp = 0.20f,
    .ki = 0.03f,
    .kd = 0.0f,
    .sampling_time = 0.002f,
    .is_antiwindup_enabled = true,
    .output_max = 1.0f,
    .integral_max = 10.0f,
};

typedef struct {
  float vx;      // m/s，+X 向前。
  float vy;      // m/s，+Y 向左。
  float omega_z; // rad/s，逆时针为正。
} RobotVelocity;

typedef struct {
  RobotVelocity desired_velocity;
  RobotVelocity actual_velocity;
  float estimated_power;
  float yaw_angle;
  float target_yaw_angle;
  float yaw_rate;
  float control_dt;
  Bmi088Status imu_status;
  bool imu_ready;
  bool translation_stick_active;
  bool steering_idle;
  bool spin_translation_active;
  bool spin_drive_tracking;
  float spin_translation_reference_yaw;
  float spin_translation_relative_yaw;
  uint32_t imu_sample_count;
  uint32_t imu_error_count;
} RobotChassisState;

typedef struct {
  float yaw_angle;
  float target_yaw_angle;
  float pitch_angle;
  float yaw_rate;
  float pitch_rate;
} RobotGimbalState;

typedef struct {
  DjiMotor chassis_motor_3508[kRobotChassisMotorCount];
  DjiMotor chassis_motor_6020[kRobotChassisMotorCount];
  volatile uint32_t chassis_3508_last_feedback_tick[kRobotChassisMotorCount];
  volatile uint32_t chassis_6020_last_feedback_tick[kRobotChassisMotorCount];
  volatile bool chassis_3508_has_feedback[kRobotChassisMotorCount];
  volatile bool chassis_6020_has_feedback[kRobotChassisMotorCount];
  bool chassis_drive_steering_ready;
  bool chassis_host_control_active;
  bool chassis_steering_vector_active[kRobotChassisMotorCount];
  bool chassis_steering_target_initialized[kRobotChassisMotorCount];
  float chassis_steering_requested_angle[kRobotChassisMotorCount];
  float chassis_steering_last_direction[kRobotChassisMotorCount];
  DjiMotor friction_motor[2];
  DjiMotor bullet_feed_motor;
  DjiMotor pitch_motor;
  DmMotor yaw_motor;
  RobotChassisState chassis;
  RobotGimbalState gimbal;
  uint16_t bullet_count;
  uint32_t chassis_imu_last_tick;
  uint32_t gimbal_last_tick;
  uint32_t keyboard_mouse_last_frame_count;
  volatile bool is_initialized;
  volatile bool is_initializing;
} RobotApp;

static RobotApp g_robot;

static float RobotClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float RobotRcNormalize(int16_t value) {
  return RobotClamp((float)value / 660.0f, -1.0f, 1.0f);
}

static float RobotApplyDeadband(float value, float deadband) {
  if (value > -deadband && value < deadband) {
    return 0.0f;
  }
  return value;
}

static bool RobotAutoSpinIsEnabled(void) {
  return SwitchIsUp(g_dt7_object.rocker.sw1);
}

static float RobotWrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static bool RobotChassisStickActive(float vx, float vy, bool was_active) {
  float stick_magnitude = fmaxf(fabsf(vx), fabsf(vy));
  return was_active ? stick_magnitude >= kRobotChassisTranslationDeadband
                    : stick_magnitude > kRobotChassisTranslationStartDeadband;
}

static float RobotSteeringTargetAngle(uint8_t index,
                                      float angle_from_forward) {
  // 安装方向统一由每轮配置决定，平移和旋转使用同一套坐标变换。
  return RobotWrapPi(kRobotSteeringForwardAngleRad[index] +
                     kRobotSteeringDirection[index] * angle_from_forward);
}

static bool RobotSteeringVectorActive(float wheel_speed, bool was_active) {
  return was_active ? wheel_speed >= kRobotSteeringVectorHoldSpeedMps
                    : wheel_speed > kRobotSteeringVectorResumeSpeedMps;
}

static void RobotSteeringUpdateTarget(uint8_t index, DjiMotor *steer) {
  if (!g_robot.chassis_steering_target_initialized[index]) {
    steer->state.given_angle = steer->state.angle;
    g_robot.chassis_steering_last_direction[index] = 0.0f;
    g_robot.chassis_steering_target_initialized[index] = true;
  }

  float target_error = RobotWrapPi(
      g_robot.chassis_steering_requested_angle[index] -
      steer->state.given_angle);
  if (fabsf(fabsf(target_error) - (float)M_PI) <
          kRobotSteeringDirectionLockBandRad &&
      g_robot.chassis_steering_last_direction[index] != 0.0f) {
    target_error = copysignf(fabsf(target_error),
                             g_robot.chassis_steering_last_direction[index]);
  }

  float max_step = kRobotSteeringTargetRateMaxRadps *
                   (float)kRobotChassisPeriodMs * 0.001f;
  float target_step = RobotClamp(target_error, -max_step, max_step);
  if (target_step != 0.0f) {
    g_robot.chassis_steering_last_direction[index] = target_step;
  }
  steer->state.given_angle =
      RobotWrapPi(steer->state.given_angle + target_step);
}

static bool RobotChassisFeedbackValid(bool has_feedback,
                                      uint32_t last_feedback_tick,
                                      uint32_t now) {
  return kRobotChassisNoMotorTxTest != 0U ||
         (has_feedback &&
          (uint32_t)(now - last_feedback_tick) <=
              kRobotChassisFeedbackTimeoutMs);
}

static bool RobotSteeringDriveReady(void) {
  if (kRobotChassisNoMotorTxTest != 0U) {
    return true;
  }
  uint32_t now = osKernelGetTickCount();
  if (!g_robot.chassis.spin_translation_active) {
    g_robot.chassis.spin_drive_tracking = false;
  }
  float max_error = 0.0f;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    const DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    if (!RobotChassisFeedbackValid(g_robot.chassis_6020_has_feedback[i],
                                   g_robot.chassis_6020_last_feedback_tick[i],
                                   now)) {
      g_robot.chassis_drive_steering_ready = false;
      g_robot.chassis.spin_drive_tracking = false;
      return false;
    }
    // 检查最终优化目标，避免斜坡目标贴近反馈时提前放行驱动轮。
    float error = fabsf(RobotWrapPi(g_robot.chassis_steering_requested_angle[i] -
                                      steer->state.angle));
    max_error = fmaxf(max_error, error);
  }
  if (g_robot.chassis.spin_translation_active) {
    // 小陀螺每次进入/停车恢复后先对齐。运行时目标连续变化，跟踪误差
    // 由各轮 cos³ 补偿降低驱动力，不用普通运动的 0.4 rad 阈值切断全车。
    if (!g_robot.chassis.spin_drive_tracking &&
        max_error < kRobotSteeringDriveEnableErrorRad) {
      g_robot.chassis.spin_drive_tracking = true;
    }
    g_robot.chassis_drive_steering_ready = g_robot.chassis.spin_drive_tracking;
    return g_robot.chassis_drive_steering_ready;
  }
  if (g_robot.chassis_drive_steering_ready) {
    if (max_error > kRobotSteeringDriveDisableErrorRad) {
      g_robot.chassis_drive_steering_ready = false;
    }
  } else if (max_error < kRobotSteeringDriveEnableErrorRad) {
    g_robot.chassis_drive_steering_ready = true;
  }
  return g_robot.chassis_drive_steering_ready;
}

static void RobotUpdateKeyboardMouseInput(void) {
  if (kRobotKeyboardMouseEnabled == 0U) {
    g_robot.keyboard_mouse_last_frame_count = Dt7FrameCount();
    return;
  }

  uint32_t frame_count = Dt7FrameCount();
  if (frame_count == g_robot.keyboard_mouse_last_frame_count) {
    return;
  }
  g_robot.keyboard_mouse_last_frame_count = frame_count;

  g_robot.gimbal.target_yaw_angle = RobotWrapPi(
      g_robot.gimbal.target_yaw_angle +
      (float)g_dt7_object.mouse.x * kRobotMouseYawRadPerCount);
  g_robot.gimbal.pitch_angle -=
      (float)g_dt7_object.mouse.y * kRobotMousePitchRadPerCount;
}

static float RobotChassisYawRateCommand(float wheel_input) {
  float wheel_yaw_rate = -wheel_input * kRobotChassisWheelMaxOmegaRadps;
  float key_yaw_rate = ((float)g_dt7_object.key.e -
                        (float)g_dt7_object.key.q) * kRobotMaxOmegaRadps;
  return RobotClamp(wheel_yaw_rate + key_yaw_rate,
                    -kRobotMaxOmegaRadps, kRobotMaxOmegaRadps);
}

static float RobotGimbalYawRateCommand(void) {
  float yaw_input = RobotApplyDeadband(
      RobotRcNormalize(g_dt7_object.rocker.ch0),
      kRobotGimbalYawDeadband);
  float yaw_rate =
      kRobotGimbalYawRcSign * yaw_input * kRobotGimbalYawMaxOmegaRadps;
  return RobotClamp(yaw_rate, -kRobotGimbalYawMaxOmegaRadps,
                       kRobotGimbalYawMaxOmegaRadps);
}

static float RobotGimbalPitchRateCommand(void) {
  float pitch_input = RobotApplyDeadband(
      RobotRcNormalize(g_dt7_object.rocker.ch1),
      kRobotGimbalPitchDeadband);
  float pitch_rate = kRobotGimbalPitchRcSign * pitch_input *
                     kRobotGimbalPitchMaxOmegaRadps;
  return RobotClamp(pitch_rate, -kRobotGimbalPitchMaxOmegaRadps,
                       kRobotGimbalPitchMaxOmegaRadps);
}

static void RobotPackDjiCurrents(const DjiMotor *motor_0,
                                    const DjiMotor *motor_1,
                                    const DjiMotor *motor_2,
                                    const DjiMotor *motor_3,
                                    uint8_t tx_data[8]) {
  const DjiMotor *motors[4] = {motor_0, motor_1, motor_2, motor_3};
  memset(tx_data, 0, 8);
  for (uint8_t i = 0; i < 4U; i++) {
    if (motors[i] == NULL) {
      continue;
    }
    int16_t current_units = DjiMotorCurrentAmperesToUnits(
        motors[i]->type, motors[i]->state.given_current);
    tx_data[i * 2U] = (uint8_t)(current_units >> 8);
    tx_data[i * 2U + 1U] = (uint8_t)(current_units & 0xFFU);
  }
}

// FireWater 纯数字 CSV，10 ms 一行；通道表见 FireWater_telemetry.md。
static uint16_t RobotFormatGm6020Telemetry(char *buffer, size_t capacity) {
  if (buffer == NULL || capacity == 0U) {
    return 0U;
  }
  const uint8_t index = ROBOT_GM6020_TELEMETRY_ID - 1U;
  // 短临界区复制电机与控制状态，浮点计算和格式化在临界区外进行。
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  DjiMotorState state = g_robot.chassis_motor_6020[index].state;
  DjiMotorMode mode = g_robot.chassis_motor_6020[index].mode;
  float requested_angle = g_robot.chassis_steering_requested_angle[index];
  bool target_initialized = g_robot.chassis_steering_target_initialized[index];
  bool has_feedback = g_robot.chassis_6020_has_feedback[index];
  uint32_t feedback_tick = g_robot.chassis_6020_last_feedback_tick[index];
  uint32_t now = osKernelGetTickCount();
  bool drive_ready = g_robot.chassis_drive_steering_ready;
  bool host_control = g_robot.chassis_host_control_active;
  __set_PRIMASK(lock);

  uint32_t age_ms = has_feedback ? now - feedback_tick : 0U;
  bool feedback_valid = has_feedback && age_ms <= kRobotChassisFeedbackTimeoutMs;
  bool active = feedback_valid && target_initialized && state.is_enabled;
  bool position_active = active && mode == kDjiMotorModePosition;
  // 给请求角选择距当前反馈最近的连续角分支；有效目标使用 PID 实际 reference。
  float requested_continuous = state.continuous_angle;
  if (active) {
    requested_continuous += RobotWrapPi(requested_angle - state.angle);
  }
  float target_continuous = position_active ? state.position_loop.reference
                                           : state.continuous_angle;
  int length = snprintf(
      buffer, capacity,
      "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
      "%.1f,%u,%u,%lu,%.4f,%.4f,%u,%u\n",
      (unsigned)ROBOT_GM6020_TELEMETRY_ID,
      (double)requested_continuous, (double)target_continuous,
      (double)state.continuous_angle,
      (double)(position_active ? state.position_loop.error : 0.0f),
      (double)state.given_omega, (double)state.omega,
      (double)state.speed_loop.error,
      (double)state.given_current, (double)state.current,
      (double)(state.speed_loop.kp * state.speed_loop.error),
      (double)(state.speed_loop.ki * state.speed_loop.error_sum),
      (double)state.temperature, (unsigned)feedback_valid, (unsigned)active,
      (unsigned long)age_ms, (double)state.position_loop.output_max,
      (double)state.speed_loop.output_max,
      (unsigned)drive_ready, (unsigned)host_control);
  if (length < 0 || (size_t)length >= capacity) {
    return 0U;
  }
  return (uint16_t)length;
}

static void RobotSendGm6020Telemetry(void) {
  if (huart1.gState != HAL_UART_STATE_READY) {
    return;
  }
  static char tx_data[kUartBufferSize];
  uint16_t length = RobotFormatGm6020Telemetry(tx_data, sizeof(tx_data));
  if (length != 0U) {
    UartTransmit(&huart1, (uint8_t *)tx_data, length);
  }
}

// FDCAN1 接收底盘 GM6020 ID 1~4 的反馈（0x205~0x208）。
static void RobotFdcan1RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x205U && std_id <= 0x208U) {
    uint8_t index = (uint8_t)(std_id - 0x205U);
    DjiMotorUpdate(&g_robot.chassis_motor_6020[index], rx_buffer);
    g_robot.chassis_6020_last_feedback_tick[index] = osKernelGetTickCount();
    g_robot.chassis_6020_has_feedback[index] = true;
  }
}

static void RobotFdcan2RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  switch (std_id) {
    case 0x201U:
      DjiMotorUpdate(&g_robot.friction_motor[0], rx_buffer);
      break;
    case 0x202U:
      DjiMotorUpdate(&g_robot.friction_motor[1], rx_buffer);
      break;
    case 0x20BU:
      DjiMotorUpdate(&g_robot.pitch_motor, rx_buffer);
      break;
    default:
      if (std_id == g_robot.yaw_motor.state.id ||
          std_id == g_robot.yaw_motor.state.master_id) {
        DmMotorUpdate(&g_robot.yaw_motor, rx_buffer);
      }
      break;
  }
}

static void RobotFdcan3RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x201U && std_id <= 0x204U) {
    uint8_t index = (uint8_t)(std_id - 0x201U);
    DjiMotorUpdate(&g_robot.chassis_motor_3508[index], rx_buffer);
    g_robot.chassis_3508_last_feedback_tick[index] = osKernelGetTickCount();
    g_robot.chassis_3508_has_feedback[index] = true;
  }
}

static void RobotMotorInit(void) {
  static PidConfig m2006_speed_pid = {
      .kp = 1.515502929687f,
      .ki = 1.165771484375f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 6.103515625f,
  };
  static PidConfig m3508_speed_pid = {
      .kp = 0.0515502929687f,
      .ki = 0.00f,
      .kd = 0.00f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
      .is_deadband_enabled = true,
      .deadband_threshold = 1.0f,
  };
  static PidConfig chassis_m3508_speed_pid = {
      .kp = 0.0315f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
  };
  static PidConfig chassis_gm6020_position_pid = {
      .kp = 40.0f,
      .ki = 0.005f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 0.0f,
  };
  static PidConfig chassis_gm6020_speed_pid = {
      .kp = 0.25f,
      .ki = 0.005f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 2.0f,
      .integral_max = 1.5f,
  };
  static PidConfig gimble_gm6020_speed_pid = {
      .kp = 3.51f,
      .ki = 1.1f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 0.54f,
      .integral_max = 0.54f,
  };
  static PidConfig gimbie_gm6020_position_pid = {
      .kp = 3.51f,
      .ki = 0.51f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_feedforward_enabled = true,
      .kff = 0.1f,
      .is_antiwindup_enabled = true,
      .output_max = 10.0f,
      .integral_max = 10.0f,
  };
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotorInit(&g_robot.chassis_motor_3508[i], kDjiMotorM3508,
                 kDjiMotorModeSpeed, &chassis_m3508_speed_pid, NULL);
    DjiMotorInit(&g_robot.chassis_motor_6020[i], kDjiMotorGm6020,
                 kDjiMotorModePosition, &chassis_gm6020_speed_pid,
                 &chassis_gm6020_position_pid);
  }
  DjiMotorInit(&g_robot.friction_motor[0], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_robot.friction_motor[1], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_robot.bullet_feed_motor, kDjiMotorM2006,
               kDjiMotorModeSpeed, &m2006_speed_pid, NULL);
  DjiMotorInit(&g_robot.pitch_motor, kDjiMotorGm6020,
               kDjiMotorModePosition, &gimble_gm6020_speed_pid,&gimbie_gm6020_position_pid);
  DmMotorInit(&g_robot.yaw_motor, kDmMotorTypeJ4310, kDmMotorModeVelocity,
              0x06U, 0x11U);
}

static void RobotSharedInit(void) {
  if (g_robot.is_initialized) {
    return;
  }

  uint32_t lock = osKernelLock();
  if (g_robot.is_initialized || g_robot.is_initializing) {
    (void)osKernelRestoreLock(lock);
    while (!g_robot.is_initialized) {
      osDelay(1);
    }
    return;
  }
  memset(&g_robot, 0, sizeof(g_robot));
  g_robot.chassis.steering_idle = true;
  g_robot.is_initializing = true;
  (void)osKernelRestoreLock(lock);

  RobotMotorInit();
  if (kRobotChassisEnabled != 0U) {
    FdcanInit(&hfdcan1, RobotFdcan1RxCallback);
  }
  if (kRobotGimbalEnabled != 0U || kRobotShootEnabled != 0U) {
    FdcanInit(&hfdcan2, RobotFdcan2RxCallback);
  }
  if (kRobotChassisEnabled != 0U || kRobotShootEnabled != 0U) {
    FdcanInit(&hfdcan3, RobotFdcan3RxCallback);
  }
  UartInit(&huart5, Dt7RxCallback);
  HostVelocityProtocolInit();
  UartInit(&huart7, HostVelocityProtocolRxCallback);
  if (kRobotGimbalLinkEnabled != 0U) {
    GimbalLinkInit();
    UartInit(&huart10, GimbalLinkRxCallback);
  }
  if (kRobotVisionEnabled != 0U) {
    UartInit(&huart10, VisionProtocolRxCallback);
  }
  if (kRobotRefereeEnabled != 0U) {
    RefereeInit(&g_referee, &huart1, &huart1);
  } else if (kRobotChassisEnabled != 0U) {
    UartInit(&huart1, NULL);
  }
  if (kRobotChassisEnabled != 0U) {
    g_robot.chassis.imu_status = Bmi088Init(&g_bmi088, &hspi2);
    if (g_robot.chassis.imu_status == kBmi088NoError) {
      Bmi088HeaterInit(&g_bmi088, &htim3, TIM_CHANNEL_4,
                       &g_robot_bmi088_heater_pid,
                       kRobotImuHeaterTargetTemperatureC);
      g_robot.chassis.imu_status =
          Bmi088CalibrateGyroOffset(&g_bmi088, 300U, 1U);
      g_robot.chassis.imu_ready =
          g_robot.chassis.imu_status == kBmi088NoError;
    }
    g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
    g_robot.gimbal.target_yaw_angle = g_robot.chassis.target_yaw_angle;
  }
  g_robot.gimbal.pitch_angle = kRobotPitchOffsetRad;

  if (kRobotGimbalEnabled != 0U) {
    uint8_t tx_data[kRobotCanDataLength] = {0};
    DmMotorPackEnable(&g_robot.yaw_motor, tx_data);
    FdcanTransmit(&hfdcan2, tx_data, kRobotCanDataLength,
                  DmMotorTxId(&g_robot.yaw_motor));
  }
  g_robot.chassis_imu_last_tick = osKernelGetTickCount();
  g_robot.gimbal_last_tick = osKernelGetTickCount();
  g_robot.is_initialized = true;
  g_robot.is_initializing = false;
}

static float RobotEstimateChassisPower(void) {
  float total_power = 0.0f;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    float current = g_robot.chassis_motor_3508[i].state.given_current;
    float omega = g_robot.chassis_motor_3508[i].state.omega;
    float torque = kRobotPowerK0 * current;
    float wheel_power =
        torque * omega + kRobotPowerK1 * fabsf(omega) +
        kRobotPowerK2 * torque * torque +
        kRobotPowerK3 / (float)kRobotChassisMotorCount;
    total_power += fmaxf(wheel_power, 0.0f);
  }
  return total_power;
}

static void RobotApplyChassisPowerLimit(float max_power) {
  float command_power = RobotEstimateChassisPower();
  if (max_power <= 0.0f) {
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_current = 0.0f;
    }
    g_robot.chassis.estimated_power = 0.0f;
    return;
  }

  if (command_power > max_power) {
    float scale = max_power / command_power;
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_current *= scale;
    }
    g_robot.chassis.estimated_power = max_power;
  } else {
    g_robot.chassis.estimated_power = command_power;
  }
}

static float RobotChassisPowerLimit(void) {
  if (kRobotRefereeEnabled != 0U &&
      g_referee.game_robot_state.chassis_power_limit > 0U) {
    return (float)g_referee.game_robot_state.chassis_power_limit;
  }
  return kRobotDefaultChassisPowerW;
}

static void RobotChassisImuUpdate(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_robot.chassis_imu_last_tick;
  g_robot.chassis_imu_last_tick = now;
  g_robot.chassis.control_dt = (float)elapsed_ms * 0.001f;

  // 运行中读失败可以重试；初始化或零偏标定失败不能在运动中直接补标定。
  if (!g_bmi088.is_inited || !g_bmi088.is_gyro_offset_inited) {
    g_robot.chassis.imu_ready = false;
    g_robot.chassis.yaw_rate = 0.0f;
    return;
  }

  bool was_ready = g_robot.chassis.imu_ready;
  Bmi088Status status = Bmi088Update(&g_bmi088);
  if (g_bmi088.is_heater_enabled) {
    (void)Bmi088HeaterControl(&g_bmi088);
  }
  float yaw_rate =
      kRobotImuYawRateSign * g_bmi088.corrected_gyro[kRobotImuGyroZAxis];
  if (status == kBmi088NoError && !isfinite(yaw_rate)) {
    status = kBmi088NoSensor;
  }
  g_robot.chassis.imu_status = status;
  if (status != kBmi088NoError) {
    ++g_robot.chassis.imu_error_count;
    g_robot.chassis.imu_ready = false;
    g_robot.chassis.yaw_rate = 0.0f;
    g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
    return;
  }

  g_robot.chassis.imu_ready = true;
  ++g_robot.chassis.imu_sample_count;
  g_robot.chassis.yaw_rate = yaw_rate;
  // 恢复后的首帧不把失效期间的时间按当前角速度积分。
  if (was_ready) {
    g_robot.chassis.yaw_angle = RobotWrapPi(
        g_robot.chassis.yaw_angle + yaw_rate * (float)elapsed_ms * 0.001f);
  }
}

static bool RobotChassisRemoteVelocity(bool *spin_in_place) {
  float forward_stick = RobotRcNormalize(g_dt7_object.rocker.ch3);
  float left_stick = -RobotRcNormalize(g_dt7_object.rocker.ch2);
  g_robot.chassis.translation_stick_active = RobotChassisStickActive(
      forward_stick, left_stick, g_robot.chassis.translation_stick_active);
  float vx_input = (g_robot.chassis.translation_stick_active
                        ? RobotApplyDeadband(forward_stick,
                                             kRobotChassisTranslationDeadband)
                        : 0.0f) +
                   (float)g_dt7_object.key.w - (float)g_dt7_object.key.s;
  float vy_input = (g_robot.chassis.translation_stick_active
                        ? RobotApplyDeadband(left_stick,
                                             kRobotChassisTranslationDeadband)
                        : 0.0f) +
                   (float)g_dt7_object.key.a - (float)g_dt7_object.key.d;
  bool translation_key_active = g_dt7_object.key.w || g_dt7_object.key.a ||
                                g_dt7_object.key.s || g_dt7_object.key.d;
  float wheel_input = RobotApplyDeadband(
      RobotRcNormalize(g_dt7_object.rocker.wheel),
      kRobotChassisYawWheelDeadband);
  bool wheel_spin_active = wheel_input != 0.0f;
  float desired_yaw_rate = RobotChassisYawRateCommand(wheel_input);
  bool auto_spin_enabled = RobotAutoSpinIsEnabled();
  if (auto_spin_enabled) {
    if (g_robot.chassis.imu_ready) {
      g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
    }
    desired_yaw_rate += kRobotChassisAutoSpinOmegaRadps;
    desired_yaw_rate = RobotClamp(desired_yaw_rate, -kRobotMaxOmegaRadps,
                                     kRobotMaxOmegaRadps);
  }

  // 必须在IMU角速度补偿前判断，摇杆归中时停止舵向角度控制。
  bool steering_idle = !g_robot.chassis.translation_stick_active &&
                       !translation_key_active &&
                       !wheel_spin_active &&
                       (kRobotSteeringIdleOverridesYaw != 0U ||
                        desired_yaw_rate == 0.0f);
  g_robot.chassis.desired_velocity.vx =
      RobotClamp(vx_input, -1.0f, 1.0f) * kRobotMaxVxMps;
  g_robot.chassis.desired_velocity.vy =
      RobotClamp(vy_input, -1.0f, 1.0f) * kRobotMaxVyMps;
  g_robot.chassis.desired_velocity.omega_z = desired_yaw_rate;
  *spin_in_place = wheel_spin_active &&
                   g_robot.chassis.desired_velocity.vx == 0.0f &&
                   g_robot.chassis.desired_velocity.vy == 0.0f;
  return steering_idle;
}

static void RobotChassisSpinTranslation(float *vx, float *vy,
                                        float desired_yaw_rate,
                                        bool input_in_body_frame) {
  float reference_vx = *vx;
  float reference_vy = *vy;
  float gimbal_yaw = 0.0f;
  if (!input_in_body_frame) {
    GimbalYawFeedback feedback;
    if (!GimbalLinkGetYaw(&feedback)) {
      // 没有新鲜相对角时禁止遥控平移，旋转指令由独立 wz 保留。
      *vx = 0.0f;
      *vy = 0.0f;
      g_robot.chassis.spin_translation_active = false;
      g_robot.chassis.spin_drive_tracking = false;
      g_robot.chassis.spin_translation_relative_yaw = 0.0f;
      return;
    }
    gimbal_yaw = feedback.relative_yaw_rad;
  }
  if (kRobotChassisSpinTranslationEnabled == 0U ||
      fabsf(desired_yaw_rate) <= kRobotChassisSpinCommandEpsilon) {
    g_robot.chassis.spin_translation_active = false;
    g_robot.chassis.spin_translation_relative_yaw = 0.0f;
    // R(alpha): 云台前/左速度 -> 车身前/左速度，直行也需要转换。
    *vx = cosf(gimbal_yaw) * reference_vx - sinf(gimbal_yaw) * reference_vy;
    *vy = sinf(gimbal_yaw) * reference_vx + cosf(gimbal_yaw) * reference_vy;
    return;
  }
  if (!g_robot.chassis.imu_ready) {
    // 没有有效航向时不静默退回随车头旋转的平移；保持已有旋转指令。
    g_robot.chassis.spin_translation_active = false;
    g_robot.chassis.spin_translation_relative_yaw = 0.0f;
    *vx = 0.0f;
    *vy = 0.0f;
    return;
  }
  if (!g_robot.chassis.spin_translation_active) {
    g_robot.chassis.spin_translation_reference_yaw = g_robot.chassis.yaw_angle;
    g_robot.chassis.spin_translation_active = true;
  }

  // 积分航向仅保留作诊断，平移方向由实时云台相对角确定。
  float body_yaw = RobotWrapPi(
      g_robot.chassis.yaw_angle - g_robot.chassis.spin_translation_reference_yaw);
  float relative_yaw = RobotWrapPi(
      body_yaw +
      g_robot.chassis.yaw_rate * kRobotChassisSpinYawCompensationS);
  g_robot.chassis.spin_translation_relative_yaw = relative_yaw;
  // R(alpha - lead)，UART7 的 alpha=0；不再叠加累计航向转换。
  float transform_yaw = g_robot.chassis.yaw_rate *
                            kRobotChassisSpinYawCompensationS - gimbal_yaw;
  float sin_yaw = sinf(transform_yaw);
  float cos_yaw = cosf(transform_yaw);
  // 按原始输入选择降速比例，避免坐标转换后比例随航向跳变。
  float scale = fabsf(reference_vy) > kRobotChassisSpinCommandEpsilon
                    ? kRobotChassisSpinLateralScale
                    : (fabsf(reference_vx) > kRobotChassisSpinCommandEpsilon
                           ? kRobotChassisSpinForwardScale : 1.0f);
  *vx = (cos_yaw * reference_vx + sin_yaw * reference_vy) * scale;
  *vy = (-sin_yaw * reference_vx + cos_yaw * reference_vy) * scale;
}

static void RobotChassisKinematics(const HostVelocityCommand *host_command) {
  bool steering_idle;
  bool wheel_spin_in_place;
  if (host_command != NULL) {
    // UART7 始终为当前车身坐标：+vx 向车头，+vy 向车左，单位 m/s。
    g_robot.chassis.desired_velocity.vx =
        RobotClamp(host_command->vx, -kRobotMaxVxMps, kRobotMaxVxMps);
    g_robot.chassis.desired_velocity.vy =
        RobotClamp(host_command->vy, -kRobotMaxVyMps, kRobotMaxVyMps);
    g_robot.chassis.desired_velocity.omega_z =
        RobotClamp(host_command->wz, -kRobotMaxOmegaRadps, kRobotMaxOmegaRadps);
    steering_idle = g_robot.chassis.desired_velocity.vx == 0.0f &&
                    g_robot.chassis.desired_velocity.vy == 0.0f &&
                    g_robot.chassis.desired_velocity.omega_z == 0.0f;
    wheel_spin_in_place = !steering_idle &&
                         g_robot.chassis.desired_velocity.vx == 0.0f &&
                         g_robot.chassis.desired_velocity.vy == 0.0f;
    g_robot.chassis.translation_stick_active = false;
  } else {
    steering_idle = RobotChassisRemoteVelocity(&wheel_spin_in_place);
  }
  g_robot.chassis.steering_idle = steering_idle;
  if (steering_idle) {
    g_robot.chassis.spin_translation_active = false;
    g_robot.chassis.spin_drive_tracking = false;
    g_robot.chassis.spin_translation_relative_yaw = 0.0f;
    memset(&g_robot.chassis.desired_velocity, 0,
           sizeof(g_robot.chassis.desired_velocity));
    g_robot.chassis.actual_velocity.vx = 0.0f;
    g_robot.chassis.actual_velocity.vy = 0.0f;
    g_robot.chassis.actual_velocity.omega_z = g_robot.chassis.yaw_rate;
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_omega = 0.0f;
      g_robot.chassis_steering_vector_active[i] = false;
      g_robot.chassis_steering_requested_angle[i] =
          g_robot.chassis_motor_6020[i].state.angle;
    }
    return;
  }

  // 遥控始终使用实时云台坐标；UART7 保持当前车身坐标。
  float vx = g_robot.chassis.desired_velocity.vx;
  float vy = g_robot.chassis.desired_velocity.vy;
  float desired_yaw_rate = g_robot.chassis.desired_velocity.omega_z;
  RobotChassisSpinTranslation(&vx, &vy, desired_yaw_rate,
                              host_command != NULL);
  float wz = desired_yaw_rate;
  if (g_robot.chassis.imu_ready) {
    wz += kRobotChassisYawRateFeedbackKp *
          (desired_yaw_rate - g_robot.chassis.yaw_rate);
  }
  wz = RobotClamp(wz, -kRobotMaxOmegaRadps, kRobotMaxOmegaRadps);
  g_robot.chassis.actual_velocity.vx = vx;
  g_robot.chassis.actual_velocity.vy = vy;
  g_robot.chassis.actual_velocity.omega_z = g_robot.chassis.yaw_rate;
  // 齐奇舵轮解算：向量叠加 -> 最优角反向 -> cos^3 -> 四轮等比例限速。
  // 按本车 +X 向前、+Y 向左、实际轮距和 SI 单位适配轮组解算。
  SwerveWheelVector wheels[kSwerveWheelCount];
  float drive_speeds[kSwerveWheelCount] = {0};
  SwerveResolve(vx, vy, wz, kRobotChassisHalfWidthM,
                 kRobotChassisHalfLengthM, wheels);
  float cosine_deadband = fabsf(wz) > 0.0001f
                             ? 0.0f : kRobotSteeringCosineDeadbandRad;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    float wheel_speed = wheels[i].speed_mps;
    g_robot.chassis_steering_vector_active[i] =
        (wheel_spin_in_place && wheel_speed > 0.0f) ||
        RobotSteeringVectorActive(
            wheel_speed, g_robot.chassis_steering_vector_active[i]);
    if (!g_robot.chassis_steering_vector_active[i]) {
      g_robot.chassis_motor_3508[i].state.given_omega = 0.0f;
      continue;
    }
    float target_angle = RobotSteeringTargetAngle(
        i, wheels[i].angle_from_forward);
    SwerveWheelTarget target = SwerveOptimizeWheel(
        target_angle, steer->state.angle, wheel_speed, cosine_deadband);
    g_robot.chassis_steering_requested_angle[i] = target.angle_rad;
    drive_speeds[i] = target.drive_speed_mps;
  }
  SwerveLimitWheelSpeeds(drive_speeds, kRobotChassisMaxWheelSpeedMps);
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_motor_3508[i].state.given_omega =
        kRobotDriveDirection[i] * drive_speeds[i] *
        kRobotDriveReductionRatio / kRobotWheelRadiusM;
  }
}

static void RobotChassisControl(const HostVelocityCommand *host_command) {
  RobotChassisImuUpdate();
  RobotChassisKinematics(host_command);

  // 零速度时停止舵向角度控制并发送零电流；再次移动时从当前角度起步。
  bool steering_idle = g_robot.chassis.steering_idle;
  bool no_motor_test_idle = kRobotChassisNoMotorTxTest != 0U &&
                            g_robot.chassis.desired_velocity.vx == 0.0f &&
                            g_robot.chassis.desired_velocity.vy == 0.0f &&
                            g_robot.chassis.desired_velocity.omega_z == 0.0f;
  uint32_t now = osKernelGetTickCount();
  bool all_steer_feedback_valid = true;
  bool all_drive_feedback_valid = true;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotor *steer = &g_robot.chassis_motor_6020[i];
    bool steer_feedback_valid = RobotChassisFeedbackValid(
        g_robot.chassis_6020_has_feedback[i],
        g_robot.chassis_6020_last_feedback_tick[i], now);
    all_steer_feedback_valid =
        all_steer_feedback_valid && steer_feedback_valid;
    if (!steering_idle && !no_motor_test_idle && steer_feedback_valid) {
      RobotSteeringUpdateTarget(i, steer);
      DjiMotorCurrentCalculate(steer);
    } else {
      steer->state.given_angle = steer->state.angle;
      steer->state.given_omega = 0.0f;
      steer->state.given_current = 0.0f;
      PidReset(&steer->state.position_loop);
      PidReset(&steer->state.speed_loop);
      g_robot.chassis_steering_target_initialized[i] = false;
      g_robot.chassis_steering_last_direction[i] = 0.0f;
    }
    all_drive_feedback_valid =
        all_drive_feedback_valid &&
        RobotChassisFeedbackValid(g_robot.chassis_3508_has_feedback[i],
                                  g_robot.chassis_3508_last_feedback_tick[i],
                                  now);
  }
  bool drive_ready = !steering_idle && !no_motor_test_idle &&
                      all_drive_feedback_valid &&
                     all_steer_feedback_valid && RobotSteeringDriveReady();
  if (!drive_ready) {
    g_robot.chassis_drive_steering_ready = false;
    g_robot.chassis.spin_drive_tracking = false;
  }
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    DjiMotor *drive = &g_robot.chassis_motor_3508[i];
    if (drive_ready) {
      DjiMotorCurrentCalculate(drive);
    } else {
      drive->state.given_omega = 0.0f;
      drive->state.given_current = 0.0f;
      PidReset(&drive->state.speed_loop);
    }
  }
  if (kRobotRefereeEnabled != 0U) {
    RobotApplyChassisPowerLimit(RobotChassisPowerLimit());
  } else {
    for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
      g_robot.chassis_motor_3508[i].state.given_current *=
          kRobotNoRefereeCurrentScale;
    }
    g_robot.chassis.estimated_power = RobotEstimateChassisPower();
  }
  uint8_t tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(&g_robot.chassis_motor_3508[0],
                       &g_robot.chassis_motor_3508[1],
                       &g_robot.chassis_motor_3508[2],
                       &g_robot.chassis_motor_3508[3], tx_data);
  FdcanTransmit(&hfdcan3, tx_data, kRobotCanDataLength, 0x200U);
  RobotPackDjiCurrents(&g_robot.chassis_motor_6020[0],
                       &g_robot.chassis_motor_6020[1],
                       &g_robot.chassis_motor_6020[2],
                       &g_robot.chassis_motor_6020[3], tx_data);
  FdcanTransmit(&hfdcan1, tx_data, kRobotCanDataLength, 0x1FEU);
}

static void RobotGimbalControl(void) {
  if (g_vision_to_gimbal.mode == 1U || g_vision_to_gimbal.mode == 2U) {
    g_robot.gimbal.yaw_rate = g_vision_to_gimbal.yaw_velocity;
    g_robot.gimbal.pitch_angle =
        g_vision_to_gimbal.pitch + kRobotPitchOffsetRad;
    g_robot.gimbal.pitch_rate = g_vision_to_gimbal.pitch_velocity;
  } else {
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed_ms = now - g_robot.gimbal_last_tick;
    g_robot.gimbal_last_tick = now;
    float dt = (float)elapsed_ms * 0.001f;

    g_robot.gimbal.target_yaw_angle = RobotWrapPi(
        g_robot.gimbal.target_yaw_angle +
        RobotGimbalYawRateCommand() * dt);
    if (RobotAutoSpinIsEnabled()) {
      g_robot.gimbal.target_yaw_angle =
          g_robot.chassis.target_yaw_angle;
    }

    float chassis_yaw = g_robot.chassis.imu_ready
                            ? g_robot.chassis.yaw_angle
                            : 0.0f;
    float yaw_error =
        RobotWrapPi(g_robot.gimbal.target_yaw_angle - chassis_yaw);
    g_robot.gimbal.yaw_rate =
        kRobotGimbalYawMotorSign *
        (kRobotGimbalYawFieldKp * yaw_error -
         g_robot.chassis.yaw_rate);

    g_robot.gimbal.pitch_rate = RobotGimbalPitchRateCommand();
    g_robot.gimbal.pitch_angle += g_robot.gimbal.pitch_rate * dt;
  }

  g_robot.gimbal.pitch_angle =
      RobotClamp(g_robot.gimbal.pitch_angle,
                    kRobotPitchOffsetRad + kRobotGimbalPitchMinRad,
                    kRobotPitchOffsetRad + kRobotGimbalPitchMaxRad);
  g_robot.pitch_motor.state.given_angle = g_robot.gimbal.pitch_angle;
  DjiMotorCurrentCalculate(&g_robot.pitch_motor);

  // GM6020 pitch 在 FDCAN2，反馈 ID 0x20B 为电机 ID 7，对应 0x2FE 第3路电流。
  uint8_t pitch_tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(NULL, NULL, &g_robot.pitch_motor, NULL,
                          pitch_tx_data);
  FdcanTransmit(&hfdcan2, pitch_tx_data, kRobotCanDataLength,
                0x2FEU);

  g_robot.yaw_motor.state.given_omega = g_robot.gimbal.yaw_rate;

  //g_robot.yaw_motor.state.given_omega = 1.0f;
  uint8_t yaw_tx_data[kRobotCanDataLength] = {0};
  DmMotorPackVelocity(&g_robot.yaw_motor, yaw_tx_data);
  FdcanTransmit(&hfdcan2, yaw_tx_data, kRobotCanDataLength,
                DmMotorTxId(&g_robot.yaw_motor));
  if (g_robot.yaw_motor.state.error_code != 0U) {
    DmMotorPackClearError(&g_robot.yaw_motor, yaw_tx_data);
    FdcanTransmit(&hfdcan2, yaw_tx_data, kRobotCanDataLength,
                  DmMotorTxId(&g_robot.yaw_motor));
  }
}

static void RobotShootControl(void) {
  bool should_feed = SwitchIsUp(g_dt7_object.rocker.sw2) ||
                     g_dt7_object.mouse.l != 0U ||
                     g_vision_to_gimbal.mode == 2U;
  bool should_spin = SwitchIsUp(g_dt7_object.rocker.sw2) || should_feed;

  g_robot.bullet_feed_motor.state.given_omega =
      should_feed ? kRobotBulletFeedOmegaRadps : 0.0f;
  g_robot.friction_motor[0].state.given_omega =
      should_spin ? -kRobotFrictionOmegaRadps : 0.0f;
  g_robot.friction_motor[1].state.given_omega =
      should_spin ? kRobotFrictionOmegaRadps : 0.0f;

  if (should_feed) {
    DjiMotorCurrentCalculate(&g_robot.bullet_feed_motor);
  } else {
    g_robot.bullet_feed_motor.state.given_current = 0.0f;
  }
  uint8_t bullet_tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(&g_robot.bullet_feed_motor, NULL, NULL, NULL,
                          bullet_tx_data);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kRobotCanDataLength, 0x1FFU);

  if (should_spin) {
    DjiMotorCurrentCalculate(&g_robot.friction_motor[0]);
    DjiMotorCurrentCalculate(&g_robot.friction_motor[1]);
  } else {
    g_robot.friction_motor[0].state.given_current = 0.0f;
    g_robot.friction_motor[1].state.given_current = 0.0f;
  }
  uint8_t friction_tx_data[kRobotCanDataLength] = {0};
  RobotPackDjiCurrents(&g_robot.friction_motor[0],
                          &g_robot.friction_motor[1], NULL, NULL,
                          friction_tx_data);
  FdcanTransmit(&hfdcan2, friction_tx_data, kRobotCanDataLength,
                0x200U);
}

static void RobotStopChassisMotors(void) {
  memset(&g_robot.chassis.desired_velocity, 0,
         sizeof(g_robot.chassis.desired_velocity));
  memset(&g_robot.chassis.actual_velocity, 0,
         sizeof(g_robot.chassis.actual_velocity));
  g_robot.chassis_drive_steering_ready = false;
  g_robot.chassis.translation_stick_active = false;
  g_robot.chassis.steering_idle = true;
  g_robot.chassis.spin_translation_active = false;
  g_robot.chassis.spin_drive_tracking = false;
  g_robot.chassis.spin_translation_reference_yaw = 0.0f;
  g_robot.chassis.spin_translation_relative_yaw = 0.0f;
  for (uint8_t i = 0; i < kRobotChassisMotorCount; i++) {
    g_robot.chassis_steering_vector_active[i] = false;
    g_robot.chassis_steering_target_initialized[i] = false;
    g_robot.chassis_steering_requested_angle[i] = 0.0f;
    g_robot.chassis_steering_last_direction[i] = 0.0f;
    g_robot.chassis_motor_3508[i].state.given_omega = 0.0f;
    g_robot.chassis_motor_3508[i].state.given_current = 0.0f;
    PidReset(&g_robot.chassis_motor_3508[i].state.speed_loop);
    g_robot.chassis_motor_6020[i].state.given_angle = 0.0f;
    g_robot.chassis_motor_6020[i].state.given_omega = 0.0f;
    g_robot.chassis_motor_6020[i].state.given_current = 0.0f;
    PidReset(&g_robot.chassis_motor_6020[i].state.position_loop);
    PidReset(&g_robot.chassis_motor_6020[i].state.speed_loop);
  }
  g_robot.chassis.target_yaw_angle = g_robot.chassis.yaw_angle;
  g_robot.chassis_imu_last_tick = osKernelGetTickCount();
}

static void RobotChassisSelectControlSource(bool host_control) {
  if (g_robot.chassis_host_control_active != host_control) {
    // 切换来源时清除旧PID、舵角和旋转参考，再从当前反馈重新起步。
    RobotStopChassisMotors();
    g_robot.chassis_host_control_active = host_control;
  }
}

static void RobotStopGimbalMotors(void) {
  g_robot.gimbal.target_yaw_angle = g_robot.chassis.target_yaw_angle;
  g_robot.gimbal.pitch_angle = kRobotPitchOffsetRad;
  g_robot.gimbal.pitch_rate = 0.0f;
  g_robot.pitch_motor.state.given_angle = g_robot.gimbal.pitch_angle;
  g_robot.pitch_motor.state.given_omega = 0.0f;
  g_robot.pitch_motor.state.given_current = 0.0f;
  g_robot.yaw_motor.state.given_omega = 0.0f;
  g_robot.gimbal_last_tick = osKernelGetTickCount();
}

static void RobotStopShootMotors(void) {
  g_robot.bullet_feed_motor.state.given_omega = 0.0f;
  g_robot.bullet_feed_motor.state.given_current = 0.0f;
  g_robot.friction_motor[0].state.given_omega = 0.0f;
  g_robot.friction_motor[0].state.given_current = 0.0f;
  g_robot.friction_motor[1].state.given_omega = 0.0f;
  g_robot.friction_motor[1].state.given_current = 0.0f;
}

static void RobotTransmitGimbalStopCurrents(void) {
  uint8_t pitch_tx_data[kRobotCanDataLength] = {0};
  uint8_t yaw_tx_data[kRobotCanDataLength] = {0};
  DmMotorPackVelocity(&g_robot.yaw_motor, yaw_tx_data);

  FdcanTransmit(&hfdcan2, pitch_tx_data, kRobotCanDataLength,
                0x2FEU);
  FdcanTransmit(&hfdcan2, yaw_tx_data, kRobotCanDataLength,
                DmMotorTxId(&g_robot.yaw_motor));
}

static void RobotTransmitChassisStopCurrents(void) {
  uint8_t tx_data[kRobotCanDataLength] = {0};

  FdcanTransmit(&hfdcan3, tx_data, kRobotCanDataLength, 0x200U);
  FdcanTransmit(&hfdcan1, tx_data, kRobotCanDataLength, 0x1FEU);
}

static void RobotTransmitShootStopCurrents(void) {
  uint8_t friction_tx_data[kRobotCanDataLength] = {0};
  uint8_t bullet_tx_data[kRobotCanDataLength] = {0};

  FdcanTransmit(&hfdcan2, friction_tx_data, kRobotCanDataLength,
                0x200U);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kRobotCanDataLength, 0x1FFU);
}

static bool RobotDt7IsOnline(void) {
  if (Dt7FrameCount() == 0U) {
    return false;
  }
  return (uint32_t)(osKernelGetTickCount() - Dt7LastUpdateTick()) <=
         kRobotDt7OfflineTimeoutMs;
}

static bool RobotIsSafeMode(void) {
  if (!RobotDt7IsOnline()) {
    return true;
  }
  return kRobotRemoteSwitchSafeModeEnabled != 0U &&
         SwitchIsDown(g_dt7_object.rocker.sw1);
}

static void RobotEnterGimbalSafeMode(void) {
  RobotStopGimbalMotors();
  RobotTransmitGimbalStopCurrents();
}

static void RobotEnterChassisSafeMode(void) {
  RobotStopChassisMotors();
  RobotTransmitChassisStopCurrents();
}

static void RobotEnterShootSafeMode(void) {
  RobotStopShootMotors();
  RobotTransmitShootStopCurrents();
}

static void RobotUpdateVisionTx(void) {
  g_gimbal_to_vision.mode = 0U;
  g_gimbal_to_vision.q[0] = 1.0f;
  g_gimbal_to_vision.q[1] = 0.0f;
  g_gimbal_to_vision.q[2] = 0.0f;
  g_gimbal_to_vision.q[3] = 0.0f;
  g_gimbal_to_vision.yaw = g_robot.yaw_motor.state.angle;
  g_gimbal_to_vision.yaw_velocity = g_robot.yaw_motor.state.omega;
  g_gimbal_to_vision.pitch = g_robot.pitch_motor.state.angle;
  g_gimbal_to_vision.pitch_velocity = g_robot.pitch_motor.state.omega;
  g_gimbal_to_vision.bullet_speed =
      (kRobotRefereeEnabled != 0U) ? g_referee.shoot_data.bullet_speed
                                      : 0.0f;
  g_gimbal_to_vision.bullet_count = g_robot.bullet_count;
}

uint32_t ManagerTaskInit(void) {
  RobotSharedInit();
  return kRobotManagerPeriodMs;
}

static void RobotSendGimbalLinkStatus(void) {
  if (kRobotGimbalLinkEnabled == 0U || huart10.gState != HAL_UART_STATE_READY) {
    return;
  }
  static uint8_t sequence;
  GimbalYawFeedback feedback;
  uint8_t flags = GimbalLinkGetYaw(&feedback) ? kBoardLinkStatusYawReady : 0U;
  // Chassis task can update these fields; take one bounded snapshot.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool imu_ready = g_robot.chassis.imu_ready;
  float yaw_rate = g_robot.chassis.yaw_rate;
  bool host_control = g_robot.chassis_host_control_active;
  bool drive_ready = g_robot.chassis_drive_steering_ready;
  __set_PRIMASK(primask);
  if (imu_ready) flags |= kBoardLinkStatusImuReady;
  if (host_control) flags |= kBoardLinkStatusHostControl;
  if (drive_ready) flags |= kBoardLinkStatusDriveReady;
  if (!imu_ready || !isfinite(yaw_rate)) yaw_rate = 0.0f;
  BoardLinkMessage message = {
      .type = kBoardLinkStatus, .sequence = sequence++, .flags = flags,
      .value = (int16_t)lroundf(RobotClamp(yaw_rate * 1000.0f, -32768.0f, 32767.0f)),
  };
  uint8_t frame[kBoardLinkFrameSize];
  BoardLinkEncode(&message, frame);
  UartTransmit(&huart10, frame, sizeof(frame));
}

void ManagerTaskLoop(void) {
  if (kRobotRefereeEnabled != 0U) {
    RefereeUpdate(&g_referee);
  }
  RobotUpdateKeyboardMouseInput();
  RobotSendGimbalLinkStatus();
  if (kRobotChassisEnabled != 0U && kRobotRefereeEnabled == 0U) {
    RobotSendGm6020Telemetry();
  }
  if (kRobotVisionEnabled != 0U) {
    RobotUpdateVisionTx();
    VisionProtocolTransmitGimbalState();
  }
}

uint32_t GimbalTaskInit(void) {
  if (kRobotGimbalEnabled == 0U) {
    return kRobotGimbalPeriodMs;
  }
  RobotSharedInit();
  return kRobotGimbalPeriodMs;
}

void GimbalTaskLoop(void) {
  if (kRobotGimbalEnabled == 0U) {
    return;
  }
  if (RobotIsSafeMode()) {
    RobotEnterGimbalSafeMode();
    return;
  }
  RobotGimbalControl();
}

uint32_t ChassisTaskInit(void) {
  if (kRobotChassisEnabled == 0U) {
    return kRobotChassisPeriodMs;
  }
  RobotSharedInit();
  return kRobotChassisPeriodMs;
}

void ChassisTaskLoop(void) {
  if (kRobotChassisEnabled == 0U) {
    return;
  }
  if (RobotDt7IsOnline()) {
    RobotChassisSelectControlSource(false);
    if (kRobotRemoteSwitchSafeModeEnabled != 0U &&
        SwitchIsDown(g_dt7_object.rocker.sw1)) {
      RobotEnterChassisSafeMode();
      return;
    }
    RobotChassisControl(NULL);
    return;
  }

  HostVelocityCommand command;
  if (kRobotChassisHostControlEnabled != 0U &&
      HostVelocityProtocolGetCommand(&command, kRobotHostCommandTimeoutMs)) {
    RobotChassisSelectControlSource(true);
    RobotChassisControl(&command);
    return;
  }
  RobotEnterChassisSafeMode();
}

uint32_t ShootTaskInit(void) {
  if (kRobotShootEnabled == 0U) {
    return kRobotShootPeriodMs;
  }
  RobotSharedInit();
  return kRobotShootPeriodMs;
}

void ShootTaskLoop(void) {
  if (kRobotShootEnabled == 0U) {
    return;
  }
  if (RobotIsSafeMode()) {
    RobotEnterShootSafeMode();
    return;
  }
  RobotShootControl();
}
