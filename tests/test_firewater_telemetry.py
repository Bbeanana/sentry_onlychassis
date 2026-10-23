"""Exercise GM6020 PID channels and the UART1 FireWater send path."""

from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Skywalker/Application/Src/app_robot.c").read_text(encoding="utf-8")
assert "kRobotManagerPeriodMs = 10" in source
assert "RobotSendGm6020Telemetry();" in source
assert "UartInit(&huart1, NULL);" in source


def definition(name):
    match = re.search(
        r"^static [^\n]*\b" + name + r"\([^{}]*\) \{.*?^\}",
        source, re.M | re.S,
    )
    if not match:
        raise RuntimeError(f"Cannot find {name}")
    return match.group()


functions = "\n".join(definition(name) for name in (
    "RobotWrapPi", "RobotFormatGm6020Telemetry", "RobotSendGm6020Telemetry",
))
selection = re.search(
    r"#ifndef ROBOT_GM6020_TELEMETRY_ID.*?#endif\n#if.*?#endif",
    source, re.S,
).group()

harness = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dev_dji_motor.h"
""" + selection + r"""
static struct {
  DjiMotor chassis_motor_6020[4];
  float chassis_steering_requested_angle[4];
  bool chassis_steering_target_initialized[4];
  bool chassis_6020_has_feedback[4];
  uint32_t chassis_6020_last_feedback_tick[4];
  bool chassis_drive_steering_ready, chassis_host_control_active;
} g_robot;
static const uint8_t kRobotChassisCanSlotWheelIndex[4] = {0U, 2U, 1U, 3U};
static uint32_t tick, primask;
static uint32_t osKernelGetTickCount(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; }
static void __set_PRIMASK(uint32_t mask) { primask = mask; }
enum { kUartBufferSize = 512, kRobotChassisFeedbackTimeoutMs = 50,
       HAL_UART_STATE_READY = 0 };
static struct { unsigned gState; } huart1;
static char sent[kUartBufferSize];
static unsigned sends;
static void UartTransmit(void *uart, uint8_t *data, uint16_t length) {
  assert(uart == &huart1);
  assert(length < sizeof(sent));
  memcpy(sent, data, length);
  sent[length] = '\0';
  ++sends;
}
""" + functions + r"""
static double channel[20];
static void read_channels(void) {
  char *cursor = sent;
  for (unsigned i = 0; i < 20; ++i) {
    channel[i] = strtod(cursor, &cursor);
    assert(*cursor == (i == 19 ? '\n' : ','));
    ++cursor;
  }
  assert(*cursor == '\0');
}
static void near(double actual, double expected) {
  assert(fabs(actual - expected) < 0.00011);
}
int main(void) {
  const unsigned index =
      kRobotChassisCanSlotWheelIndex[ROBOT_GM6020_TELEMETRY_ID - 1];
  RobotSendGm6020Telemetry();
  read_channels();
  near(channel[0], ROBOT_GM6020_TELEMETRY_ID);
  for (unsigned i = 1; i < 20; ++i) near(channel[i], 0);

  // Distinct motor values expose incorrect ID selection.
  for (unsigned i = 0; i < 4; ++i)
    g_robot.chassis_motor_6020[i].state.continuous_angle = 100 + i;
  DjiMotor *motor = &g_robot.chassis_motor_6020[index];
  DjiMotorState *state = &motor->state;
  motor->mode = kDjiMotorModePosition;
  state->is_enabled = true;
  state->angle = 6.2f;
  state->continuous_angle = 6.2f + 2 * (float)M_PI;
  state->position_loop.reference = state->continuous_angle + 0.1f;
  state->position_loop.error = 0.1f;
  state->position_loop.output_max = 4;
  state->given_omega = 2;
  state->omega = 1.5f;
  state->speed_loop.error = 0.5f;
  state->speed_loop.kp = 0.2f;
  state->speed_loop.ki = 0.3f;
  state->speed_loop.error_sum = -0.5f;
  state->speed_loop.output_max = 0.8f;
  state->given_current = -0.05f;
  state->current = -0.04f;
  state->temperature = 37;
  g_robot.chassis_steering_requested_angle[index] = 0.15f;
  g_robot.chassis_steering_target_initialized[index] = true;
  g_robot.chassis_6020_has_feedback[index] = true;
  g_robot.chassis_6020_last_feedback_tick[index] = 100;
  g_robot.chassis_drive_steering_ready = true;
  g_robot.chassis_host_control_active = true;
  tick = 105;
  primask = 1;
  RobotSendGm6020Telemetry();
  assert(primask == 1);
  read_channels();
  near(channel[1], 0.15 + 4 * M_PI);
  near(channel[2], state->position_loop.reference);
  near(channel[3], state->continuous_angle);
  near(channel[4], 0.1);
  near(channel[5], 2);
  near(channel[6], 1.5);
  near(channel[7], 0.5);
  near(channel[8], -0.05);
  near(channel[9], -0.04);
  near(channel[10], 0.1);
  near(channel[11], -0.15);
  near(channel[12], 37);
  near(channel[13], 1);
  near(channel[14], 1);
  near(channel[15], 5);
  near(channel[16], 4);
  near(channel[17], 0.8);
  near(channel[18], 1);
  near(channel[19], 1);

  char csv[512];
  uint16_t length = RobotFormatGm6020Telemetry(csv, sizeof(csv));
  assert(length == strlen(sent) && strcmp(csv, sent) == 0);
  assert(RobotFormatGm6020Telemetry(csv, length) == 0);
  assert(RobotFormatGm6020Telemetry(csv, length + 1) == length);
  assert(RobotFormatGm6020Telemetry(csv, 5) == 0);
  assert(RobotFormatGm6020Telemetry(csv, 0) == 0);
  assert(RobotFormatGm6020Telemetry(NULL, sizeof(csv)) == 0);
  unsigned previous_sends = sends;
  huart1.gState = 1;
  RobotSendGm6020Telemetry();
  assert(sends == previous_sends);
  huart1.gState = HAL_UART_STATE_READY;

  tick = 150;
  RobotSendGm6020Telemetry(); read_channels();
  near(channel[13], 1);
  tick = 151;
  RobotSendGm6020Telemetry(); read_channels();
  near(channel[13], 0); near(channel[14], 0);
  near(channel[1], state->continuous_angle);
  near(channel[2], state->continuous_angle);
  near(channel[4], 0);
  g_robot.chassis_6020_last_feedback_tick[index] = UINT32_MAX - 10;
  tick = 5;
  RobotSendGm6020Telemetry(); read_channels();
  near(channel[13], 1); near(channel[15], 16);

  // Idle/reset and speed-only mode must not display an old position reference.
  g_robot.chassis_steering_target_initialized[index] = false;
  primask = 0;
  RobotSendGm6020Telemetry(); read_channels();
  assert(primask == 0);
  near(channel[14], 0); near(channel[2], state->continuous_angle);
  g_robot.chassis_steering_target_initialized[index] = true;
  motor->mode = kDjiMotorModeSpeed;
  RobotSendGm6020Telemetry(); read_channels();
  near(channel[14], 1); near(channel[4], 0);
  near(channel[2], state->continuous_angle);
  printf("PASS: GM6020 ID %u PID telemetry, wrap, freshness, buffer and UART1\n",
         ROBOT_GM6020_TELEMETRY_ID);
}
"""

with tempfile.TemporaryDirectory(prefix="firewater_telemetry_") as directory:
    work = Path(directory)
    (work / "test.c").write_text(harness, encoding="utf-8")
    for motor_id in range(1, 5):
        executable = work / f"test_{motor_id}.exe"
        subprocess.run([
            "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            f"-DROBOT_GM6020_TELEMETRY_ID={motor_id}",
            "-I", str(ROOT / "Skywalker/Device/Inc"),
            "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
            str(work / "test.c"), "-lm", "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
