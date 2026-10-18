#include "app_gimbal_link.h"

#include <stddef.h>
#include <string.h>
#include "stm32h7xx_hal.h"

static uint8_t g_rx[kBoardLinkFrameSize];
static uint8_t g_rx_length;
static volatile GimbalYawFeedback g_feedback;
static volatile bool g_has_feedback;

void GimbalLinkInit(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_rx_length = 0U;
  g_feedback = (GimbalYawFeedback){0};
  g_has_feedback = false;
  __set_PRIMASK(primask);
}

void GimbalLinkRxCallback(uint8_t *data, uint16_t length) {
  if (data == NULL) return;
  for (uint16_t i = 0; i < length; ++i) {
    g_rx[g_rx_length++] = data[i];
    while (g_rx_length != 0U) {
      if (g_rx[0] == 0xA5U && (g_rx_length < 2U || g_rx[1] == 0x5AU)) {
        if (g_rx_length < kBoardLinkFrameSize) break;
        BoardLinkMessage message;
        if (BoardLinkDecode(g_rx, &message)) {
          if (message.type == kBoardLinkYaw && (message.flags & ~1U) == 0U &&
              message.value >= -3142 && message.value <= 3142) {
            bool valid = (message.flags & kBoardLinkYawValid) != 0U;
            // Retransmitting the same sample must not keep a stale angle alive.
            // Invalid feedback always revokes control, including a duplicate.
            if (!valid || !g_has_feedback || message.sequence != g_feedback.sequence) {
              g_feedback.relative_yaw_rad = (float)message.value * 0.001f;
              g_feedback.last_update_ms = HAL_GetTick();
              g_feedback.sequence = message.sequence;
              g_feedback.valid = valid;
              ++g_feedback.frame_count;
              g_has_feedback = true;
            }
          }
          g_rx_length = 0U;
          break;
        }
      }
      --g_rx_length;
      memmove(g_rx, &g_rx[1], g_rx_length);
    }
  }
}

bool GimbalLinkGetLatest(GimbalYawFeedback *feedback) {
  if (feedback == NULL) return false;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  GimbalYawFeedback sample = g_feedback;
  bool has_feedback = g_has_feedback;
  __set_PRIMASK(primask);
  *feedback = has_feedback ? sample : (GimbalYawFeedback){0};
  return has_feedback;
}

bool GimbalLinkGetYaw(GimbalYawFeedback *feedback) {
  if (feedback == NULL) return false;
  if (!GimbalLinkGetLatest(feedback) || !feedback->valid ||
      (uint32_t)(HAL_GetTick() - feedback->last_update_ms) > kGimbalLinkTimeoutMs) {
    *feedback = (GimbalYawFeedback){0};
    return false;
  }
  return true;
}
