#include "alg_board_link.h"

#include <math.h>
#include <stddef.h>

static uint16_t BoardLinkCrc(const uint8_t *data) {
  uint16_t crc = 0xFFFFU;
  for (unsigned i = 0; i < 8; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (unsigned bit = 0; bit < 8; ++bit) {
      uint32_t shifted = (uint32_t)crc << 1;
      crc = (uint16_t)((crc & 0x8000U) ? shifted ^ 0x1021U : shifted);
    }
  }
  return crc;
}

void BoardLinkEncode(const BoardLinkMessage *message,
                     uint8_t frame[kBoardLinkFrameSize]) {
  if (message == NULL || frame == NULL) return;
  uint16_t value = (uint16_t)message->value;
  frame[0] = 0xA5U; frame[1] = 0x5AU; frame[2] = 1U;
  frame[3] = message->type;
  frame[4] = message->sequence;
  frame[5] = message->flags;
  frame[6] = (uint8_t)value; frame[7] = (uint8_t)(value >> 8);
  uint16_t crc = BoardLinkCrc(frame);
  frame[8] = (uint8_t)crc; frame[9] = (uint8_t)(crc >> 8);
}

bool BoardLinkDecode(const uint8_t frame[kBoardLinkFrameSize],
                     BoardLinkMessage *message) {
  if (frame == NULL || message == NULL || frame[0] != 0xA5U ||
      frame[1] != 0x5AU || frame[2] != 1U ||
      (frame[3] != kBoardLinkYaw && frame[3] != kBoardLinkStatus)) return false;
  uint16_t crc = (uint16_t)frame[8] | (uint16_t)frame[9] << 8;
  if (BoardLinkCrc(frame) != crc) return false;
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
