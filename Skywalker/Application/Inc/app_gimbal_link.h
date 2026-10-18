#ifndef APP_GIMBAL_LINK_H
#define APP_GIMBAL_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "alg_board_link.h"

enum { kGimbalLinkTimeoutMs = 50U };
typedef struct {
  float relative_yaw_rad; /* Gimbal forward relative to chassis forward, CCW+. */
  uint32_t last_update_ms;
  uint32_t frame_count;
  uint8_t sequence;
  bool valid;
} GimbalYawFeedback;

void GimbalLinkInit(void);
void GimbalLinkRxCallback(uint8_t *data, uint16_t length);
/* Coherent diagnostic sample, even if invalid or stale. */
bool GimbalLinkGetLatest(GimbalYawFeedback *feedback);
/* Requires the remote validity bit and a fresh local receive timestamp. */
bool GimbalLinkGetYaw(GimbalYawFeedback *feedback);

#endif
