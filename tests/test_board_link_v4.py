"""Host regression tests for the v4 board-link codec and stream parser."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "alg_board_link.h"

static void nearf(float a, float b) { assert(fabsf(a - b) < 0.00001f); }

int main(void) {
  uint8_t bytes[kBoardLinkMaxFrameSize];
  uint16_t length = 0;
  BoardLinkFrame frame;
  BoardLinkParser parser;

  BoardLinkHeartbeat heartbeat = {
      .role = kBoardLinkRoleGimbalController,
      .safety_state = kBoardLinkSafetyActive,
      .active_reasons = 3,
      .sender_uptime_ms = 1234,
      .sender_boot_id = 0x1122334455667788ULL,
      .ready = true,
      .sync_requested = false,
  };
  assert(BoardLinkEncodeHeartbeat(&heartbeat, 7, bytes, sizeof(bytes), &length));
  assert(length == 34);
  assert(bytes[0] == 0xA5 && bytes[1] == 0x5A && bytes[2] == 4 && bytes[3] == 1);
  assert(BoardLinkDecodeFrame(bytes, length, &frame));
  frame.received_ms = 100;
  BoardLinkHeartbeat decoded_heartbeat;
  assert(BoardLinkDecodeHeartbeat(&frame, &decoded_heartbeat));
  assert(decoded_heartbeat.sender_boot_id == heartbeat.sender_boot_id);
  assert(decoded_heartbeat.stamp.sequence == 7);
  assert(decoded_heartbeat.stamp.timestamp_ms == 100);

  BoardLinkChassisControl control = {
      .mode = kBoardLinkModeBodyVelocity,
      .source = 1,
      .global_action = 2,
      .vx_m_s = 1.25f,
      .vy_m_s = -0.5f,
      .wz_rad_s = 2.0f,
      .active_reasons = 0,
      .command_sequence = 55,
      .receiver_boot_id = 0x8877665544332211ULL,
  };
  assert(BoardLinkEncodeChassisControl(&control, 8, bytes, sizeof(bytes), &length));
  assert(length == 46);
  BoardLinkParserInit(&parser);
  uint8_t noise[] = {0x00, 0x7F, 0xA5};
  BoardLinkParserFeed(&parser, noise, sizeof(noise), 200);
  BoardLinkParserFeed(&parser, bytes, 5, 201);
  BoardLinkParserFeed(&parser, bytes + 5, (uint16_t)(length - 5), 202);
  assert(BoardLinkParserPop(&parser, &frame));
  BoardLinkChassisControl decoded_control;
  assert(BoardLinkDecodeChassisControl(&frame, &decoded_control));
  nearf(decoded_control.vx_m_s, 1.25f);
  nearf(decoded_control.vy_m_s, -0.5f);
  nearf(decoded_control.wz_rad_s, 2.0f);
  assert(decoded_control.command_sequence == 55);
  assert(decoded_control.stamp.timestamp_ms == 202);
  assert(!BoardLinkParserPop(&parser, &frame));

  bytes[15] ^= 0x01;
  BoardLinkParserFeed(&parser, bytes, length, 300);
  assert(parser.crc_errors > 0);
  assert(!BoardLinkParserPop(&parser, &frame));

  BoardLinkChassisFeedback feedback = {
      .execution_state = kBoardLinkExecutionRunning,
      .safety_state = kBoardLinkSafetyActive,
      .ready = true,
      .armed = true,
      .active_reasons = 0,
      .last_command_sequence = 55,
      .valid_fields = 3,
      .vx_m_s = 1.0f,
      .vy_m_s = 2.0f,
      .wz_rad_s = -3.0f,
      .power_w = 25.0f,
  };
  assert(BoardLinkEncodeChassisFeedback(&feedback, 9, bytes, sizeof(bytes), &length));
  assert(length == 46);
  assert(BoardLinkDecodeFrame(bytes, length, &frame));
  BoardLinkChassisFeedback decoded_feedback;
  assert(BoardLinkDecodeChassisFeedback(&frame, &decoded_feedback));
  nearf(decoded_feedback.power_w, 25.0f);

  BoardLinkOperatorControl op = {
      .run_allowed = true,
      .emergency_stop = false,
      .source_sequence = 10,
      .source_age_ms = 4,
      .clear_event_id = 0,
      .clear_event_age_ms = UINT32_MAX,
      .sender_boot_id = 1,
      .receiver_boot_id = 2,
  };
  assert(BoardLinkEncodeOperatorControl(&op, 10, bytes, sizeof(bytes), &length));
  assert(length == 50);
  assert(BoardLinkDecodeFrame(bytes, length, &frame));
  BoardLinkOperatorControl decoded_op;
  assert(BoardLinkDecodeOperatorControl(&frame, &decoded_op));
  assert(decoded_op.run_allowed && decoded_op.clear_event_age_ms == UINT32_MAX);

  BoardLinkBigYawRequest yaw = {
      .mode = kBoardLinkBigYawFollowCenter,
      .permission_valid = true,
      .permission_enabled = true,
      .target_rate_rad_s = 1.5f,
      .source_sequence = 20,
      .source_age_ms = 2,
      .command_age_ms = 3,
      .receiver_boot_id = 3,
      .permission_age_ms = 4,
      .stamp = {.sequence = 21},
  };
  assert(BoardLinkEncodeBigYawRequest(&yaw, 11, bytes, sizeof(bytes), &length));
  assert(length == 50);
  assert(BoardLinkDecodeFrame(bytes, length, &frame));
  BoardLinkBigYawRequest decoded_yaw;
  assert(BoardLinkDecodeBigYawRequest(&frame, &decoded_yaw));
  nearf(decoded_yaw.target_rate_rad_s, 1.5f);
  assert(decoded_yaw.stamp.sequence == 21);

  assert(BoardLinkSequenceIsNewer(0, 0xFFFFFFFFU));
  assert(!BoardLinkSequenceIsNewer(7, 7));
  assert(!BoardLinkSequenceIsNewer(6, 7));
  puts("PASS: board-link v4 framing, CRC, parser, roles, payloads and sequence");
  return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="board_link_v4_") as directory:
    work = Path(directory)
    (work / "test.c").write_text(HARNESS)
    exe = work / "test"
    subprocess.run([
        "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
        str(work / "test.c"),
        str(ROOT / "Skywalker/Algorithm/Src/alg_board_link.c"),
        "-lm", "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
