#include "app_gimbal_link.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#if defined(__GNUC__)
__attribute__((weak))
#endif
bool GimbalLinkTransportSend(const uint8_t *frame, uint16_t length) {
  (void)frame;
  (void)length;
  return false;
}

static BoardLinkParser g_parser;
static volatile BoardLinkHeartbeat g_heartbeat;
static volatile BoardLinkChassisControl g_chassis_control;
static volatile BoardLinkBigYawRequest g_big_yaw_request;
static volatile GimbalImuYaw g_imu_yaw;
static volatile bool g_has_imu_yaw;
static uint32_t g_last_imu_frame_sequence;
static bool g_has_imu_sequence;
static volatile bool g_has_heartbeat;
static volatile bool g_has_chassis_control;
static volatile bool g_has_big_yaw_request;
static volatile bool g_has_control_sequence;
static volatile bool g_has_big_yaw_sequence;
static volatile uint32_t g_last_control_frame_sequence;
static volatile uint32_t g_last_big_yaw_source_sequence;
static uint64_t g_local_boot_id;
static uint32_t g_tx_frame_sequence;

/* V1 compatibility stream. It is never used by the v4 application path. */
static uint8_t g_legacy_rx[kBoardLinkFrameSize];
static uint8_t g_legacy_rx_length;
static volatile GimbalYawFeedback g_legacy_feedback;
static volatile bool g_has_legacy_feedback;

static uint64_t GimbalLinkMakeBootId(void) {
  return ((uint64_t)HAL_GetTick() << 32) ^
         (uint64_t)(uintptr_t)&g_local_boot_id ^ 0x534B5957414C4B52ULL;
}

static void GimbalLinkResetPeerSnapshots(void) {
  g_has_imu_yaw = false;
  g_has_imu_sequence = false;
  memset((void *)&g_imu_yaw, 0, sizeof(g_imu_yaw));
  g_has_chassis_control = false;
  g_has_control_sequence = false;
  g_has_big_yaw_request = false;
  g_has_big_yaw_sequence = false;
  memset((void *)&g_chassis_control, 0, sizeof(g_chassis_control));
  memset((void *)&g_big_yaw_request, 0, sizeof(g_big_yaw_request));
}

static uint32_t GimbalLinkLoad32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float GimbalLinkLoadFloat(const uint8_t *p) {
  uint32_t raw = GimbalLinkLoad32(p); float value;
  memcpy(&value, &raw, sizeof(value)); return value;
}

static void GimbalLinkAcceptFrame(const BoardLinkFrame *frame) {
  if (frame == NULL) {
    return;
  }
  switch (frame->message_id) {
    case kBoardLinkHeartbeat: {
      BoardLinkHeartbeat heartbeat;
      if (!BoardLinkDecodeHeartbeat(frame, &heartbeat) ||
          heartbeat.role != kBoardLinkRoleGimbalController) {
        return;
      }
      uint64_t previous_boot = g_has_heartbeat ? g_heartbeat.sender_boot_id : 0U;
      if (previous_boot != 0U && previous_boot != heartbeat.sender_boot_id) {
        GimbalLinkResetPeerSnapshots();
      }
      g_heartbeat = heartbeat;
      g_has_heartbeat = true;
      break;
    }
    case kBoardLinkChassisControl: {
      BoardLinkChassisControl control;
      if (!BoardLinkDecodeChassisControl(frame, &control)) {
        return;
      }
      if (g_has_control_sequence &&
          !BoardLinkSequenceIsNewer(frame->frame_sequence,
                                    g_last_control_frame_sequence)) {
        return;
      }
      g_last_control_frame_sequence = frame->frame_sequence;
      g_has_control_sequence = true;
      g_chassis_control = control;
      g_has_chassis_control = true;
      break;
    }
    case kBoardLinkGimbalImuYaw: {
      const uint8_t *p = frame->payload;
      if (frame->sender_role != kBoardLinkRoleGimbalController || frame->payload_length != 32U ||
          p[0] > 1U || p[1] || p[2] || p[3]) return;
      GimbalImuYaw sample = {
          .yaw_delta_rad = GimbalLinkLoadFloat(p + 4),
          .yaw_rate_rad_s = GimbalLinkLoadFloat(p + 8),
          .source_age_ms = GimbalLinkLoad32(p + 12),
          .source_sequence = GimbalLinkLoad32(p + 16),
          .zero_generation = GimbalLinkLoad32(p + 20),
          .receiver_boot_id = (uint64_t)GimbalLinkLoad32(p + 24) |
                              ((uint64_t)GimbalLinkLoad32(p + 28) << 32),
          .stamp = {.timestamp_ms = frame->received_ms, .sequence = frame->frame_sequence, .valid = true},
          .valid = p[0] != 0,
      };
      if (sample.receiver_boot_id != g_local_boot_id || sample.zero_generation == 0 ||
          !isfinite(sample.yaw_delta_rad) || !isfinite(sample.yaw_rate_rad_s) ||
          fabsf(sample.yaw_delta_rad) > 3.142f || fabsf(sample.yaw_rate_rad_s) > 35.0f ||
          (g_has_imu_sequence && !BoardLinkSequenceIsNewer(frame->frame_sequence, g_last_imu_frame_sequence))) return;
      // Repeated sensor samples must not refresh their physical freshness.
      if (sample.valid && g_has_imu_yaw && g_imu_yaw.valid &&
          sample.zero_generation == g_imu_yaw.zero_generation &&
          sample.source_sequence == g_imu_yaw.source_sequence) {
        uint32_t elapsed = (uint32_t)(HAL_GetTick() - g_imu_yaw.stamp.timestamp_ms);
        uint32_t prior_age = g_imu_yaw.source_age_ms;
        uint32_t age = prior_age > 50U || elapsed > 50U ? 51U : prior_age + elapsed;
        if (sample.source_age_ms < age) sample.source_age_ms = age;
      }
      g_last_imu_frame_sequence = frame->frame_sequence; g_has_imu_sequence = true;
      g_imu_yaw = sample; g_has_imu_yaw = true;
      break;
    }
    case kBoardLinkBigYawRequest: {
      BoardLinkBigYawRequest request;
      if (!BoardLinkDecodeBigYawRequest(frame, &request)) {
        return;
      }
      if (request.receiver_boot_id != g_local_boot_id) {
        return;
      }
      if (g_has_big_yaw_sequence &&
          !BoardLinkSequenceIsNewer(request.source_sequence,
                                    g_last_big_yaw_source_sequence)) {
        return;
      }
      g_last_big_yaw_source_sequence = request.source_sequence;
      g_has_big_yaw_sequence = true;
      g_big_yaw_request = request;
      g_has_big_yaw_request = true;
      break;
    }
    default:
      /* The chassis currently observes the other v4 messages through the
       * generic parser; application consumers can be added without changing
       * the wire decoder. */
      break;
  }
}

static void GimbalLinkProcessV4(uint8_t *data, uint16_t length) {
  BoardLinkParserFeed(&g_parser, data, length, HAL_GetTick());
  BoardLinkFrame frame;
  while (BoardLinkParserPop(&g_parser, &frame)) {
    GimbalLinkAcceptFrame(&frame);
  }
}

static void GimbalLinkProcessLegacy(uint8_t *data, uint16_t length) {
  if (data == NULL) return;
  for (uint16_t i = 0U; i < length; ++i) {
    if (g_legacy_rx_length >= kBoardLinkFrameSize) {
      g_legacy_rx_length = 0U;
    }
    g_legacy_rx[g_legacy_rx_length++] = data[i];
    while (g_legacy_rx_length != 0U) {
      if (g_legacy_rx[0] == 0xA5U &&
          (g_legacy_rx_length < 2U || g_legacy_rx[1] == 0x5AU)) {
        if (g_legacy_rx_length < kBoardLinkFrameSize) break;
        BoardLinkMessage message;
        if (BoardLinkDecode(g_legacy_rx, &message)) {
          if (message.type == kBoardLinkYaw &&
              (message.flags & (uint8_t)~1U) == 0U &&
              message.value >= -3142 && message.value <= 3142) {
            bool valid = (message.flags & kBoardLinkYawValid) != 0U;
            if (!valid || !g_has_legacy_feedback ||
                message.sequence != g_legacy_feedback.sequence) {
              g_legacy_feedback.relative_yaw_rad =
                  (float)message.value * 0.001f;
              g_legacy_feedback.last_update_ms = HAL_GetTick();
              g_legacy_feedback.sequence = message.sequence;
              g_legacy_feedback.valid = valid;
              ++g_legacy_feedback.frame_count;
              g_has_legacy_feedback = true;
            }
          }
          g_legacy_rx_length = 0U;
          break;
        }
      }
      --g_legacy_rx_length;
      memmove(g_legacy_rx, &g_legacy_rx[1], g_legacy_rx_length);
    }
  }
}

void GimbalLinkInit(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  BoardLinkParserInit(&g_parser);
  memset((void *)&g_heartbeat, 0, sizeof(g_heartbeat));
  memset((void *)&g_chassis_control, 0, sizeof(g_chassis_control));
  memset((void *)&g_big_yaw_request, 0, sizeof(g_big_yaw_request));
  memset((void *)&g_legacy_feedback, 0, sizeof(g_legacy_feedback));
  GimbalLinkResetPeerSnapshots();
  g_has_heartbeat = false;
  g_has_chassis_control = false;
  g_has_big_yaw_request = false;
  g_has_control_sequence = false;
  g_has_big_yaw_sequence = false;
  g_last_control_frame_sequence = 0U;
  g_last_big_yaw_source_sequence = 0U;
  g_local_boot_id = GimbalLinkMakeBootId();
  g_tx_frame_sequence = 0U;
  g_legacy_rx_length = 0U;
  g_has_legacy_feedback = false;
  __set_PRIMASK(primask);
}

void GimbalLinkRxCallback(uint8_t *data, uint16_t length) {
  if (data == NULL || length == 0U) {
    return;
  }
  GimbalLinkProcessV4(data, length);
  GimbalLinkProcessLegacy(data, length);
}

bool GimbalLinkGetChassisControl(BoardLinkChassisControl *control) {
  if (control == NULL) {
    return false;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  BoardLinkChassisControl sample = g_chassis_control;
  bool available = g_has_chassis_control;
  __set_PRIMASK(primask);
  if (!available || !sample.stamp.valid ||
      (uint32_t)(HAL_GetTick() - sample.stamp.timestamp_ms) >
          kGimbalLinkControlTimeoutMs) {
    *control = (BoardLinkChassisControl){0};
    return false;
  }
  *control = sample;
  return true;
}

bool GimbalLinkGetImuYaw(GimbalImuYaw *yaw) {
  if (yaw == NULL) return false;
  uint32_t mask = __get_PRIMASK(); __disable_irq();
  GimbalImuYaw sample = g_imu_yaw; bool available = g_has_imu_yaw;
  BoardLinkHeartbeat peer = g_heartbeat;
  bool peer_available = g_has_heartbeat;
  sample.sender_boot_id = peer_available ? peer.sender_boot_id : 0U;
  __set_PRIMASK(mask);
  uint32_t age = (uint32_t)(HAL_GetTick() - sample.stamp.timestamp_ms);
  if (!available || !sample.valid || !peer_available || !peer.stamp.valid ||
      (uint32_t)(HAL_GetTick() - peer.stamp.timestamp_ms) > kGimbalLinkHeartbeatTimeoutMs ||
      sample.receiver_boot_id != g_local_boot_id || sample.source_age_ms > 50U ||
      age > 50U - sample.source_age_ms) {
    *yaw = (GimbalImuYaw){0}; return false;
  }
  sample.source_age_ms += age;
  *yaw = sample; return true;
}

bool GimbalLinkGetBigYawRequest(BoardLinkBigYawRequest *request) {
  if (request == NULL) {
    return false;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  BoardLinkBigYawRequest sample = g_big_yaw_request;
  bool available = g_has_big_yaw_request;
  __set_PRIMASK(primask);
  if (!available || !sample.stamp.valid ||
      sample.receiver_boot_id != g_local_boot_id ||
      (uint32_t)(HAL_GetTick() - sample.stamp.timestamp_ms) >
          kGimbalLinkBigYawTimeoutMs ||
      sample.source_age_ms > kGimbalLinkBigYawTimeoutMs ||
      sample.command_age_ms > kGimbalLinkBigYawTimeoutMs ||
      (sample.permission_valid &&
       sample.permission_age_ms > kGimbalLinkPermissionTimeoutMs)) {
    *request = (BoardLinkBigYawRequest){0};
    return false;
  }
  *request = sample;
  return true;
}

bool GimbalLinkPeerOnline(void) {
  BoardLinkHeartbeat heartbeat;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  heartbeat = g_heartbeat;
  bool available = g_has_heartbeat;
  __set_PRIMASK(primask);
  return available && heartbeat.stamp.valid &&
         (uint32_t)(HAL_GetTick() - heartbeat.stamp.timestamp_ms) <=
             kGimbalLinkHeartbeatTimeoutMs;
}

uint64_t GimbalLinkLocalBootId(void) {
  return g_local_boot_id;
}

static bool GimbalLinkSendFrame(const uint8_t *frame, uint16_t length) {
  if (frame == NULL || length == 0U) {
    return false;
  }
  return GimbalLinkTransportSend(frame, length);
}

bool GimbalLinkSendHeartbeat(uint8_t safety_state, uint32_t active_reasons,
                             bool ready, bool sync_requested) {
  BoardLinkHeartbeat heartbeat = {
      .role = kBoardLinkRoleChassisController,
      .safety_state = safety_state,
      .active_reasons = active_reasons,
      .sender_uptime_ms = HAL_GetTick(),
      .sender_boot_id = g_local_boot_id,
      .ready = ready,
      .sync_requested = sync_requested,
  };
  uint8_t frame[kBoardLinkMaxFrameSize];
  uint16_t length = 0U;
  if (!BoardLinkEncodeHeartbeat(&heartbeat, g_tx_frame_sequence++, frame,
                                sizeof(frame), &length)) {
    return false;
  }
  return GimbalLinkSendFrame(frame, length);
}

bool GimbalLinkSendChassisFeedback(uint8_t execution_state,
                                    uint8_t safety_state, bool ready,
                                    bool armed, uint32_t active_reasons,
                                    uint32_t last_command_sequence,
                                    uint32_t valid_fields, float vx_m_s,
                                    float vy_m_s, float wz_rad_s,
                                    float power_w) {
  BoardLinkChassisFeedback feedback = {
      .execution_state = execution_state,
      .safety_state = safety_state,
      .ready = ready,
      .armed = armed,
      .active_reasons = active_reasons,
      .last_command_sequence = last_command_sequence,
      .valid_fields = valid_fields,
      .vx_m_s = vx_m_s,
      .vy_m_s = vy_m_s,
      .wz_rad_s = wz_rad_s,
      .power_w = power_w,
  };
  uint8_t frame[kBoardLinkMaxFrameSize];
  uint16_t length = 0U;
  if (!BoardLinkEncodeChassisFeedback(&feedback, g_tx_frame_sequence++, frame,
                                      sizeof(frame), &length)) {
    return false;
  }
  return GimbalLinkSendFrame(frame, length);
}

bool GimbalLinkSendBigYawFeedback(const BoardLinkBigYawFeedback *feedback) {
  if (feedback == NULL) {
    return false;
  }
  uint8_t frame[kBoardLinkMaxFrameSize];
  uint16_t length = 0U;
  if (!BoardLinkEncodeBigYawFeedback(feedback, g_tx_frame_sequence++, frame,
                                     sizeof(frame), &length)) {
    return false;
  }
  return GimbalLinkSendFrame(frame, length);
}

bool GimbalLinkGetLatest(GimbalYawFeedback *feedback) {
  if (feedback == NULL) return false;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  GimbalYawFeedback sample = g_legacy_feedback;
  bool has_feedback = g_has_legacy_feedback;
  __set_PRIMASK(primask);
  *feedback = has_feedback ? sample : (GimbalYawFeedback){0};
  return has_feedback;
}

bool GimbalLinkGetYaw(GimbalYawFeedback *feedback) {
  if (feedback == NULL) return false;
  if (!GimbalLinkGetLatest(feedback) || !feedback->valid ||
      (uint32_t)(HAL_GetTick() - feedback->last_update_ms) >
          kGimbalLinkTimeoutMs) {
    *feedback = (GimbalYawFeedback){0};
    return false;
  }
  return true;
}
