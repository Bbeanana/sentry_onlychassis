#ifndef APP_GIMBAL_LINK_H
#define APP_GIMBAL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "alg_board_link.h"

enum {
  kGimbalLinkTimeoutMs = 50U,             /* Retired V1 yaw compatibility. */
  kGimbalLinkControlTimeoutMs = 100U,
  kGimbalLinkHeartbeatTimeoutMs = 100U,
  kGimbalLinkPermissionTimeoutMs = 300U,
  /* BigYawRequest is produced at ~100 ms.  A 100 ms freshness window makes
   * jitter look like request loss and toggles enable/disable; keep a wider
   * window for this actuator only. */
  kGimbalLinkBigYawTimeoutMs = 250U,
};

typedef struct {
  float relative_yaw_rad;
  uint32_t last_update_ms;
  uint32_t frame_count;
  uint8_t sequence;
  bool valid;
} GimbalYawFeedback;

void GimbalLinkInit(void);
void GimbalLinkRxCallback(uint8_t *data, uint16_t length);

/* Board-specific UART/CAN owner supplies the actual batch transmission. */
bool GimbalLinkTransportSend(const uint8_t *frame, uint16_t length);

/* v4 snapshots consumed by the chassis application. */
bool GimbalLinkGetChassisControl(BoardLinkChassisControl *control);
bool GimbalLinkGetBigYawRequest(BoardLinkBigYawRequest *request);
bool GimbalLinkPeerOnline(void);
uint64_t GimbalLinkLocalBootId(void);

/* v4 status messages sent by the chassis application. */
bool GimbalLinkSendHeartbeat(uint8_t safety_state, uint32_t active_reasons,
                             bool ready, bool sync_requested);
bool GimbalLinkSendChassisFeedback(uint8_t execution_state,
                                    uint8_t safety_state, bool ready,
                                    bool armed, uint32_t active_reasons,
                                    uint32_t last_command_sequence,
                                    uint32_t valid_fields, float vx_m_s,
                                    float vy_m_s, float wz_rad_s,
                                    float power_w);
bool GimbalLinkSendBigYawFeedback(const BoardLinkBigYawFeedback *feedback);

/* Retired V1 angle snapshot retained for local diagnostics and migration tests. */
bool GimbalLinkGetLatest(GimbalYawFeedback *feedback);
bool GimbalLinkGetYaw(GimbalYawFeedback *feedback);

#endif
