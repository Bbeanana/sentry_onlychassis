#ifndef ALG_BOARD_LINK_H
#define ALG_BOARD_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* Portable codec shared by the gimbal and chassis boards; no HAL dependency.
 * A5 5A 01 type sequence flags value_lo value_hi crc_lo crc_hi
 * CRC-16/CCITT-FALSE over bytes 0..7: poly 0x1021, init 0xFFFF.
 */
enum {
  kBoardLinkFrameSize = 10,
  kBoardLinkYaw = 1,
  kBoardLinkStatus = 2,
  kBoardLinkYawValid = 1,
  kBoardLinkStatusImuReady = 1,
  kBoardLinkStatusYawReady = 2,
  kBoardLinkStatusHostControl = 4,
  kBoardLinkStatusDriveReady = 8,
};

typedef struct {
  uint8_t type, sequence, flags;
  int16_t value; /* Yaw: mrad; chassis status: measured yaw rate, mrad/s. */
} BoardLinkMessage;

void BoardLinkEncode(const BoardLinkMessage *message,
                     uint8_t frame[kBoardLinkFrameSize]);
bool BoardLinkDecode(const uint8_t frame[kBoardLinkFrameSize],
                     BoardLinkMessage *message);
/* Relative yaw: zero at chassis forward; counterclockwise positive, radians.
 * Accepts continuous angles and wraps them to [-pi, pi]. Invalid feedback
 * emits a validity=0 frame. Returns false for nonfinite valid angles.
 */
bool BoardLinkEncodeYaw(float relative_yaw_rad, bool valid, uint8_t sequence,
                        uint8_t frame[kBoardLinkFrameSize]);

#endif
