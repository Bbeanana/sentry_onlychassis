#include "alg_board_link.h"

#include <math.h>
#include <string.h>

static uint16_t LoadLe16(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t LoadLe32(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint64_t LoadLe64(const uint8_t *data) {
  uint64_t value = 0U;
  for (uint8_t i = 0U; i < 8U; ++i) {
    value |= (uint64_t)data[i] << (8U * i);
  }
  return value;
}

static void StoreLe16(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static void StoreLe32(uint8_t *data, uint32_t value) {
  for (uint8_t i = 0U; i < 4U; ++i) {
    data[i] = (uint8_t)(value >> (8U * i));
  }
}

static void StoreLe64(uint8_t *data, uint64_t value) {
  for (uint8_t i = 0U; i < 8U; ++i) {
    data[i] = (uint8_t)(value >> (8U * i));
  }
}

static float LoadFloatLe(const uint8_t *data) {
  uint32_t raw = LoadLe32(data);
  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static void StoreFloatLe(uint8_t *data, float value) {
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  StoreLe32(data, raw);
}

uint16_t BoardLinkCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFU;
  if (data == NULL) {
    return crc;
  }
  for (size_t i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((uint32_t)crc << 1) ^ 0x1021U
                : (uint16_t)((uint32_t)crc << 1);
    }
  }
  return crc;
}

static bool BoardLinkRoleIsValid(BoardLinkRole role) {
  return role == kBoardLinkRoleGimbalController ||
         role == kBoardLinkRoleChassisController;
}

static bool BoardLinkMessageIsKnown(uint16_t id) {
  switch (id) {
    case kBoardLinkHeartbeat:
    case kBoardLinkChassisControl:
    case kBoardLinkChassisConstraint:
    case kBoardLinkChassisFeedback:
    case kBoardLinkOperatorControl:
    case kBoardLinkBigYawRequest:
    case kBoardLinkBigYawFeedback:
    case kBoardLinkGimbalImuYaw:
      return true;
    default:
      return false;
  }
}

static bool BoardLinkMessageRoleIsValid(uint16_t id, BoardLinkRole role) {
  if (id == kBoardLinkHeartbeat) {
    return BoardLinkRoleIsValid(role);
  }
  if (id == kBoardLinkChassisControl || id == kBoardLinkChassisConstraint ||
      id == kBoardLinkOperatorControl || id == kBoardLinkBigYawRequest ||
      id == kBoardLinkGimbalImuYaw) {
    return role == kBoardLinkRoleGimbalController;
  }
  if (id == kBoardLinkChassisFeedback || id == kBoardLinkBigYawFeedback) {
    return role == kBoardLinkRoleChassisController;
  }
  return false;
}

bool BoardLinkDecodeFrame(const uint8_t *frame, size_t length,
                          BoardLinkFrame *decoded) {
  if (frame == NULL || decoded == NULL || length < kBoardLinkFrameOverhead ||
      length > kBoardLinkMaxFrameSize || frame[0] != 0xA5U ||
      frame[1] != 0x5AU || frame[2] != kBoardLinkProtocolVersion ||
      !BoardLinkRoleIsValid((BoardLinkRole)frame[3])) {
    return false;
  }
  uint16_t payload_length = LoadLe16(&frame[6]);
  size_t expected_length = (size_t)payload_length + kBoardLinkFrameOverhead;
  if (payload_length > kBoardLinkMaxPayload || expected_length != length ||
      !BoardLinkMessageIsKnown(LoadLe16(&frame[4])) ||
      !BoardLinkMessageRoleIsValid(LoadLe16(&frame[4]),
                                   (BoardLinkRole)frame[3])) {
    return false;
  }
  uint16_t expected_crc = LoadLe16(&frame[length - 2U]);
  if (BoardLinkCrc16(frame, length - 2U) != expected_crc) {
    return false;
  }
  memset(decoded, 0, sizeof(*decoded));
  decoded->sender_role = (BoardLinkRole)frame[3];
  decoded->message_id = LoadLe16(&frame[4]);
  decoded->payload_length = payload_length;
  decoded->frame_sequence = LoadLe32(&frame[8]);
  memcpy(decoded->payload, &frame[12], payload_length);
  return true;
}

static bool BoardLinkEncodePayload(BoardLinkRole role, uint16_t message_id,
                                   uint32_t frame_sequence,
                                   const uint8_t *payload,
                                   uint16_t payload_length, uint8_t *frame,
                                   size_t capacity, uint16_t *length) {
  if (payload == NULL || frame == NULL || length == NULL ||
      !BoardLinkMessageRoleIsValid(message_id, role) ||
      payload_length > kBoardLinkMaxPayload ||
      capacity < (size_t)payload_length + kBoardLinkFrameOverhead) {
    return false;
  }
  size_t total = (size_t)payload_length + kBoardLinkFrameOverhead;
  memset(frame, 0, total);
  frame[0] = 0xA5U;
  frame[1] = 0x5AU;
  frame[2] = kBoardLinkProtocolVersion;
  frame[3] = (uint8_t)role;
  StoreLe16(&frame[4], message_id);
  StoreLe16(&frame[6], payload_length);
  StoreLe32(&frame[8], frame_sequence);
  memcpy(&frame[12], payload, payload_length);
  StoreLe16(&frame[12U + payload_length],
            BoardLinkCrc16(frame, 12U + payload_length));
  *length = (uint16_t)total;
  return true;
}

static bool BoardLinkFinite4(float a, float b, float c, float d) {
  return isfinite(a) && isfinite(b) && isfinite(c) && isfinite(d);
}

bool BoardLinkEncodeHeartbeat(const BoardLinkHeartbeat *message,
                              uint32_t frame_sequence, uint8_t *frame,
                              size_t capacity, uint16_t *length) {
  if (message == NULL || !BoardLinkRoleIsValid(message->role) ||
      message->safety_state > kBoardLinkSafetyConfigBlocked ||
      message->sender_boot_id == 0U) {
    return false;
  }
  uint8_t payload[20] = {0};
  payload[0] = (uint8_t)message->role;
  payload[1] = message->safety_state;
  payload[2] = message->ready ? 1U : 0U;
  payload[3] = message->sync_requested ? 1U : 0U;
  StoreLe32(&payload[4], message->active_reasons);
  StoreLe32(&payload[8], message->sender_uptime_ms);
  StoreLe64(&payload[12], message->sender_boot_id);
  return BoardLinkEncodePayload(message->role, kBoardLinkHeartbeat,
                                frame_sequence, payload, sizeof(payload),
                                frame, capacity, length);
}

bool BoardLinkEncodeChassisControl(const BoardLinkChassisControl *message,
                                   uint32_t frame_sequence, uint8_t *frame,
                                   size_t capacity, uint16_t *length) {
  if (message == NULL || message->mode > kBoardLinkModeSpin ||
      message->source > 4U || message->global_action > 2U ||
      message->receiver_boot_id == 0U ||
      !BoardLinkFinite4(message->vx_m_s, message->vy_m_s,
                        message->wz_rad_s, 0.0f)) {
    return false;
  }
  uint8_t payload[32] = {0};
  payload[0] = message->mode;
  payload[1] = message->source;
  payload[2] = message->global_action;
  StoreFloatLe(&payload[4], message->vx_m_s);
  StoreFloatLe(&payload[8], message->vy_m_s);
  StoreFloatLe(&payload[12], message->wz_rad_s);
  StoreLe32(&payload[16], message->active_reasons);
  StoreLe32(&payload[20], message->command_sequence);
  StoreLe64(&payload[24], message->receiver_boot_id);
  return BoardLinkEncodePayload(kBoardLinkRoleGimbalController,
                                kBoardLinkChassisControl, frame_sequence,
                                payload, sizeof(payload), frame, capacity,
                                length);
}

bool BoardLinkEncodeChassisConstraint(const BoardLinkChassisConstraint *message,
                                      uint32_t frame_sequence, uint8_t *frame,
                                      size_t capacity, uint16_t *length) {
  if (message == NULL || message->power_limit_w < 0.0f ||
      message->buffer_energy_j < 0.0f ||
      !isfinite(message->power_limit_w) ||
      !isfinite(message->buffer_energy_j)) {
    return false;
  }
  uint8_t payload[20] = {0};
  payload[0] = message->output_valid ? 1U : 0U;
  payload[1] = message->output_enabled ? 1U : 0U;
  payload[2] = message->power_valid ? 1U : 0U;
  StoreFloatLe(&payload[4], message->power_limit_w);
  StoreFloatLe(&payload[8], message->buffer_energy_j);
  StoreLe32(&payload[12], message->output_age_ms);
  StoreLe32(&payload[16], message->power_age_ms);
  return BoardLinkEncodePayload(kBoardLinkRoleGimbalController,
                                kBoardLinkChassisConstraint, frame_sequence,
                                payload, sizeof(payload), frame, capacity,
                                length);
}

bool BoardLinkEncodeChassisFeedback(const BoardLinkChassisFeedback *message,
                                    uint32_t frame_sequence, uint8_t *frame,
                                    size_t capacity, uint16_t *length) {
  if (message == NULL || message->execution_state > kBoardLinkExecutionDisabled ||
      message->safety_state > kBoardLinkSafetyConfigBlocked ||
      (message->valid_fields & ~3U) != 0U || message->power_w < 0.0f ||
      !BoardLinkFinite4(message->vx_m_s, message->vy_m_s,
                        message->wz_rad_s, message->power_w)) {
    return false;
  }
  uint8_t payload[32] = {0};
  payload[0] = message->execution_state;
  payload[1] = message->safety_state;
  payload[2] = message->ready ? 1U : 0U;
  payload[3] = message->armed ? 1U : 0U;
  StoreLe32(&payload[4], message->active_reasons);
  StoreLe32(&payload[8], message->last_command_sequence);
  StoreLe32(&payload[12], message->valid_fields);
  StoreFloatLe(&payload[16], message->vx_m_s);
  StoreFloatLe(&payload[20], message->vy_m_s);
  StoreFloatLe(&payload[24], message->wz_rad_s);
  StoreFloatLe(&payload[28], message->power_w);
  return BoardLinkEncodePayload(kBoardLinkRoleChassisController,
                                kBoardLinkChassisFeedback, frame_sequence,
                                payload, sizeof(payload), frame, capacity,
                                length);
}

bool BoardLinkEncodeOperatorControl(const BoardLinkOperatorControl *message,
                                    uint32_t frame_sequence, uint8_t *frame,
                                    size_t capacity, uint16_t *length) {
  if (message == NULL || message->sender_boot_id == 0U ||
      message->receiver_boot_id == 0U ||
      (message->run_allowed && message->emergency_stop) ||
      (message->clear_event_id != 0U &&
       (message->run_allowed || message->emergency_stop)) ||
      (message->clear_event_id == 0U && message->clear_event_age_ms != UINT32_MAX)) {
    return false;
  }
  uint8_t payload[36] = {0};
  payload[0] = message->run_allowed ? 1U : 0U;
  payload[1] = message->emergency_stop ? 1U : 0U;
  StoreLe32(&payload[4], message->source_sequence);
  StoreLe32(&payload[8], message->source_age_ms);
  StoreLe32(&payload[12], message->clear_event_id);
  StoreLe32(&payload[16], message->clear_event_age_ms);
  StoreLe64(&payload[20], message->sender_boot_id);
  StoreLe64(&payload[28], message->receiver_boot_id);
  return BoardLinkEncodePayload(kBoardLinkRoleGimbalController,
                                kBoardLinkOperatorControl, frame_sequence,
                                payload, sizeof(payload), frame, capacity,
                                length);
}

bool BoardLinkEncodeBigYawRequest(const BoardLinkBigYawRequest *message,
                                  uint32_t frame_sequence, uint8_t *frame,
                                  size_t capacity, uint16_t *length) {
  if (message == NULL || message->mode > kBoardLinkBigYawFollowCenter ||
      message->receiver_boot_id == 0U ||
      !isfinite(message->target_rate_rad_s)) {
    return false;
  }
  uint8_t payload[36] = {0};
  payload[0] = message->mode;
  payload[1] = message->permission_valid ? 1U : 0U;
  payload[2] = message->permission_enabled ? 1U : 0U;
  StoreFloatLe(&payload[4], message->target_rate_rad_s);
  StoreLe32(&payload[8], message->source_sequence);
  StoreLe32(&payload[12], message->source_age_ms);
  StoreLe32(&payload[16], message->command_age_ms);
  StoreLe64(&payload[20], message->receiver_boot_id);
  StoreLe32(&payload[28], message->permission_age_ms);
  StoreLe32(&payload[32], message->stamp.sequence);
  return BoardLinkEncodePayload(kBoardLinkRoleGimbalController,
                                kBoardLinkBigYawRequest, frame_sequence,
                                payload, sizeof(payload), frame, capacity,
                                length);
}

bool BoardLinkEncodeBigYawFeedback(const BoardLinkBigYawFeedback *message,
                                   uint32_t frame_sequence, uint8_t *frame,
                                   size_t capacity, uint16_t *length) {
  if (message == NULL || message->execution_state > kBoardLinkExecutionDisabled ||
      !isfinite(message->actual_rate_rad_s)) {
    return false;
  }
  uint8_t payload[24] = {0};
  payload[0] = message->execution_state;
  payload[1] = message->ready ? 1U : 0U;
  payload[2] = message->armed ? 1U : 0U;
  payload[3] = message->valid ? 1U : 0U;
  StoreFloatLe(&payload[4], message->actual_rate_rad_s);
  StoreLe32(&payload[8], message->active_reasons);
  StoreLe32(&payload[12], message->last_command_sequence);
  StoreLe32(&payload[16], message->source_sequence);
  StoreLe32(&payload[20], message->production_age_ms);
  return BoardLinkEncodePayload(kBoardLinkRoleChassisController,
                                kBoardLinkBigYawFeedback, frame_sequence,
                                payload, sizeof(payload), frame, capacity,
                                length);
}

static bool BoardLinkFrameMatches(const BoardLinkFrame *frame, uint16_t id,
                                  BoardLinkRole role, uint16_t payload_length) {
  return frame != NULL && frame->message_id == id &&
         frame->sender_role == role && frame->payload_length == payload_length;
}

static BoardLinkStamp BoardLinkReceiveStamp(const BoardLinkFrame *frame,
                                            uint32_t sequence) {
  BoardLinkStamp stamp = {0};
  stamp.timestamp_ms = frame->received_ms;
  stamp.sequence = sequence;
  stamp.valid = true;
  return stamp;
}

bool BoardLinkDecodeHeartbeat(const BoardLinkFrame *frame,
                              BoardLinkHeartbeat *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkHeartbeat, frame ? frame->sender_role : 0,
                             20U) || !BoardLinkRoleIsValid(frame->sender_role)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  if (p[0] != (uint8_t)frame->sender_role || p[1] > kBoardLinkSafetyConfigBlocked ||
      p[2] > 1U || p[3] > 1U || LoadLe64(&p[12]) == 0U) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->role = frame->sender_role;
  message->safety_state = p[1];
  message->ready = p[2] != 0U;
  message->sync_requested = p[3] != 0U;
  message->active_reasons = LoadLe32(&p[4]);
  message->sender_uptime_ms = LoadLe32(&p[8]);
  message->sender_boot_id = LoadLe64(&p[12]);
  message->stamp = BoardLinkReceiveStamp(frame, frame->frame_sequence);
  return true;
}

bool BoardLinkDecodeChassisControl(const BoardLinkFrame *frame,
                                   BoardLinkChassisControl *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkChassisControl,
                             kBoardLinkRoleGimbalController, 32U)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  float vx = LoadFloatLe(&p[4]);
  float vy = LoadFloatLe(&p[8]);
  float wz = LoadFloatLe(&p[12]);
  if (p[0] > kBoardLinkModeSpin || p[1] > 4U || p[2] > 2U || p[3] != 0U ||
      LoadLe64(&p[24]) == 0U || !BoardLinkFinite4(vx, vy, wz, 0.0f)) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->mode = p[0];
  message->source = p[1];
  message->global_action = p[2];
  message->vx_m_s = vx;
  message->vy_m_s = vy;
  message->wz_rad_s = wz;
  message->active_reasons = LoadLe32(&p[16]);
  message->command_sequence = LoadLe32(&p[20]);
  message->receiver_boot_id = LoadLe64(&p[24]);
  message->stamp = BoardLinkReceiveStamp(frame, message->command_sequence);
  return true;
}

bool BoardLinkDecodeChassisConstraint(const BoardLinkFrame *frame,
                                      BoardLinkChassisConstraint *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkChassisConstraint,
                             kBoardLinkRoleGimbalController, 20U)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  float power_limit = LoadFloatLe(&p[4]);
  float buffer_energy = LoadFloatLe(&p[8]);
  if (p[0] > 1U || p[1] > 1U || p[2] > 1U || p[3] != 0U ||
      power_limit < 0.0f || buffer_energy < 0.0f ||
      !isfinite(power_limit) || !isfinite(buffer_energy)) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->output_valid = p[0] != 0U;
  message->output_enabled = p[1] != 0U;
  message->power_valid = p[2] != 0U;
  message->power_limit_w = power_limit;
  message->buffer_energy_j = buffer_energy;
  message->output_age_ms = LoadLe32(&p[12]);
  message->power_age_ms = LoadLe32(&p[16]);
  message->stamp = BoardLinkReceiveStamp(frame, frame->frame_sequence);
  return true;
}

bool BoardLinkDecodeChassisFeedback(const BoardLinkFrame *frame,
                                    BoardLinkChassisFeedback *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkChassisFeedback,
                             kBoardLinkRoleChassisController, 32U)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  float vx = LoadFloatLe(&p[16]);
  float vy = LoadFloatLe(&p[20]);
  float wz = LoadFloatLe(&p[24]);
  float power = LoadFloatLe(&p[28]);
  if (p[0] > kBoardLinkExecutionDisabled || p[1] > kBoardLinkSafetyConfigBlocked ||
      p[2] > 1U || p[3] > 1U || (LoadLe32(&p[12]) & ~3U) != 0U ||
      power < 0.0f || !BoardLinkFinite4(vx, vy, wz, power)) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->execution_state = p[0];
  message->safety_state = p[1];
  message->ready = p[2] != 0U;
  message->armed = p[3] != 0U;
  message->active_reasons = LoadLe32(&p[4]);
  message->last_command_sequence = LoadLe32(&p[8]);
  message->valid_fields = LoadLe32(&p[12]);
  message->vx_m_s = vx;
  message->vy_m_s = vy;
  message->wz_rad_s = wz;
  message->power_w = power;
  message->stamp = BoardLinkReceiveStamp(frame, frame->frame_sequence);
  return true;
}

bool BoardLinkDecodeOperatorControl(const BoardLinkFrame *frame,
                                    BoardLinkOperatorControl *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkOperatorControl,
                             kBoardLinkRoleGimbalController, 36U)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  uint32_t clear_event_id = LoadLe32(&p[12]);
  uint32_t clear_event_age = LoadLe32(&p[16]);
  if (p[0] > 1U || p[1] > 1U || p[2] != 0U || p[3] != 0U ||
      LoadLe64(&p[20]) == 0U || LoadLe64(&p[28]) == 0U ||
      (p[0] != 0U && p[1] != 0U) ||
      (clear_event_id != 0U && (p[0] != 0U || p[1] != 0U)) ||
      (clear_event_id == 0U && clear_event_age != UINT32_MAX)) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->run_allowed = p[0] != 0U;
  message->emergency_stop = p[1] != 0U;
  message->source_sequence = LoadLe32(&p[4]);
  message->source_age_ms = LoadLe32(&p[8]);
  message->clear_event_id = clear_event_id;
  message->clear_event_age_ms = clear_event_age;
  message->sender_boot_id = LoadLe64(&p[20]);
  message->receiver_boot_id = LoadLe64(&p[28]);
  message->stamp = BoardLinkReceiveStamp(frame, frame->frame_sequence);
  return true;
}

bool BoardLinkDecodeBigYawRequest(const BoardLinkFrame *frame,
                                  BoardLinkBigYawRequest *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkBigYawRequest,
                             kBoardLinkRoleGimbalController, 36U)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  float target_rate = LoadFloatLe(&p[4]);
  if (p[0] > kBoardLinkBigYawFollowCenter || p[1] > 1U || p[2] > 1U ||
      p[3] != 0U || LoadLe64(&p[20]) == 0U || !isfinite(target_rate)) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->mode = p[0];
  message->permission_valid = p[1] != 0U;
  message->permission_enabled = p[2] != 0U;
  message->target_rate_rad_s = target_rate;
  message->source_sequence = LoadLe32(&p[8]);
  message->source_age_ms = LoadLe32(&p[12]);
  message->command_age_ms = LoadLe32(&p[16]);
  message->receiver_boot_id = LoadLe64(&p[20]);
  message->permission_age_ms = LoadLe32(&p[28]);
  message->stamp = BoardLinkReceiveStamp(frame, LoadLe32(&p[32]));
  return true;
}

bool BoardLinkDecodeBigYawFeedback(const BoardLinkFrame *frame,
                                   BoardLinkBigYawFeedback *message) {
  if (!BoardLinkFrameMatches(frame, kBoardLinkBigYawFeedback,
                             kBoardLinkRoleChassisController, 24U)) {
    return false;
  }
  const uint8_t *p = frame->payload;
  float actual_rate = LoadFloatLe(&p[4]);
  if (p[0] > kBoardLinkExecutionDisabled || p[1] > 1U || p[2] > 1U ||
      p[3] > 1U || !isfinite(actual_rate)) {
    return false;
  }
  memset(message, 0, sizeof(*message));
  message->execution_state = p[0];
  message->ready = p[1] != 0U;
  message->armed = p[2] != 0U;
  message->valid = p[3] != 0U;
  message->actual_rate_rad_s = actual_rate;
  message->active_reasons = LoadLe32(&p[8]);
  message->last_command_sequence = LoadLe32(&p[12]);
  message->source_sequence = LoadLe32(&p[16]);
  message->production_age_ms = LoadLe32(&p[20]);
  message->stamp = BoardLinkReceiveStamp(frame, frame->frame_sequence);
  return true;
}

void BoardLinkParserInit(BoardLinkParser *parser) {
  if (parser != NULL) {
    memset(parser, 0, sizeof(*parser));
  }
}

static void BoardLinkParserDiscard(BoardLinkParser *parser, uint16_t count) {
  if (parser == NULL || count == 0U) {
    return;
  }
  if (count >= parser->length) {
    parser->length = 0U;
    return;
  }
  memmove(parser->bytes, &parser->bytes[count], parser->length - count);
  parser->length = (uint16_t)(parser->length - count);
}

static void BoardLinkParserScan(BoardLinkParser *parser, uint64_t now_ms) {
  while (parser != NULL && parser->length != 0U) {
    if (parser->length >= 2U &&
        (parser->bytes[0] != 0xA5U || parser->bytes[1] != 0x5AU)) {
      BoardLinkParserDiscard(parser, 1U);
      continue;
    }
    if (parser->length < 2U) {
      break;
    }
    if (parser->length < 12U) {
      break;
    }
    uint8_t version = parser->bytes[2];
    uint8_t role = parser->bytes[3];
    uint16_t message_id = LoadLe16(&parser->bytes[4]);
    uint16_t payload_length = LoadLe16(&parser->bytes[6]);
    if (version != kBoardLinkProtocolVersion || !BoardLinkRoleIsValid((BoardLinkRole)role)) {
      parser->version_errors++;
      BoardLinkParserDiscard(parser, 1U);
      continue;
    }
    if (payload_length > kBoardLinkMaxPayload) {
      parser->length_errors++;
      BoardLinkParserDiscard(parser, 1U);
      continue;
    }
    size_t total = (size_t)payload_length + kBoardLinkFrameOverhead;
    if (parser->length < total) {
      break;
    }
    if (BoardLinkCrc16(parser->bytes, total - 2U) !=
        LoadLe16(&parser->bytes[total - 2U])) {
      parser->crc_errors++;
      BoardLinkParserDiscard(parser, 1U);
      continue;
    }
    if (!BoardLinkMessageIsKnown(message_id) ||
        !BoardLinkMessageRoleIsValid(message_id, (BoardLinkRole)role)) {
      parser->unknown_messages++;
      BoardLinkParserDiscard(parser, (uint16_t)total);
      continue;
    }
    if (parser->queue_count >= kBoardLinkParserQueueSize) {
      parser->queue_head = (uint8_t)((parser->queue_head + 1U) %
                                     kBoardLinkParserQueueSize);
      parser->queue_count--;
      parser->queue_overflows++;
    }
    BoardLinkFrame *queued =
        &parser->queue[(parser->queue_head + parser->queue_count) %
                       kBoardLinkParserQueueSize];
    memset(queued, 0, sizeof(*queued));
    queued->sender_role = (BoardLinkRole)role;
    queued->message_id = message_id;
    queued->payload_length = payload_length;
    queued->frame_sequence = LoadLe32(&parser->bytes[8]);
    queued->received_ms = now_ms;
    memcpy(queued->payload, &parser->bytes[12], payload_length);
    parser->queue_count++;
    BoardLinkParserDiscard(parser, (uint16_t)total);
  }
}

void BoardLinkParserFeed(BoardLinkParser *parser, const uint8_t *data,
                         uint16_t length, uint64_t now_ms) {
  if (parser == NULL || data == NULL) {
    return;
  }
  if (parser->length != 0U && now_ms >= parser->byte_start_ms &&
      now_ms - parser->byte_start_ms > 100U) {
    parser->assembly_timeouts++;
    BoardLinkParserDiscard(parser, 1U);
    BoardLinkParserScan(parser, now_ms);
  }
  for (uint16_t i = 0U; i < length; ++i) {
    if (parser->length == 0U) {
      parser->byte_start_ms = now_ms;
    }
    if (parser->length >= kBoardLinkParserBufferSize) {
      parser->length = 0U;
      parser->length_errors++;
      parser->byte_start_ms = now_ms;
    }
    parser->bytes[parser->length++] = data[i];
    BoardLinkParserScan(parser, now_ms);
  }
}

bool BoardLinkParserPop(BoardLinkParser *parser, BoardLinkFrame *frame) {
  if (parser == NULL || frame == NULL || parser->queue_count == 0U) {
    return false;
  }
  *frame = parser->queue[parser->queue_head];
  parser->queue_head = (uint8_t)((parser->queue_head + 1U) %
                                 kBoardLinkParserQueueSize);
  parser->queue_count--;
  return true;
}

bool BoardLinkSequenceIsNewer(uint32_t sequence, uint32_t previous) {
  uint32_t delta = sequence - previous;
  return delta != 0U && delta < 0x80000000U;
}

/* Legacy V1 codec retained for old source users while v4 is migrated. */
static uint16_t LegacyBoardLinkCrc(const uint8_t *data) {
  return BoardLinkCrc16(data, 8U);
}

void BoardLinkEncode(const BoardLinkMessage *message,
                     uint8_t frame[kBoardLinkFrameSize]) {
  if (message == NULL || frame == NULL) return;
  uint16_t value = (uint16_t)message->value;
  frame[0] = 0xA5U; frame[1] = 0x5AU; frame[2] = 1U;
  frame[3] = message->type; frame[4] = message->sequence;
  frame[5] = message->flags; frame[6] = (uint8_t)value;
  frame[7] = (uint8_t)(value >> 8);
  uint16_t crc = LegacyBoardLinkCrc(frame);
  frame[8] = (uint8_t)crc; frame[9] = (uint8_t)(crc >> 8);
}

bool BoardLinkDecode(const uint8_t frame[kBoardLinkFrameSize],
                     BoardLinkMessage *message) {
  if (frame == NULL || message == NULL || frame[0] != 0xA5U ||
      frame[1] != 0x5AU || frame[2] != 1U ||
      (frame[3] != kBoardLinkYaw && frame[3] != kBoardLinkStatus)) return false;
  uint16_t crc = (uint16_t)frame[8] | (uint16_t)frame[9] << 8;
  if (LegacyBoardLinkCrc(frame) != crc) return false;
  uint16_t raw = (uint16_t)frame[6] | (uint16_t)frame[7] << 8;
  message->type = frame[3]; message->sequence = frame[4];
  message->flags = frame[5];
  message->value = (int16_t)(raw >= 0x8000U ? (int32_t)raw - 65536 : raw);
  return true;
}

bool BoardLinkEncodeYaw(float relative_yaw_rad, bool valid, uint8_t sequence,
                        uint8_t frame[kBoardLinkFrameSize]) {
  if (frame == NULL || (valid && !isfinite(relative_yaw_rad))) return false;
  BoardLinkMessage message = {.type = kBoardLinkYaw, .sequence = sequence,
                              .flags = valid ? kBoardLinkYawValid : 0U};
  if (valid) {
    float angle = remainderf(relative_yaw_rad, 6.283185307179586f);
    message.value = (int16_t)lroundf(angle * 1000.0f);
  }
  BoardLinkEncode(&message, frame);
  return true;
}
