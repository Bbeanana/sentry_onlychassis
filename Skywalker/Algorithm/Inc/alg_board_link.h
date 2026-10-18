#ifndef ALG_BOARD_LINK_H
#define ALG_BOARD_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* SkyWalker inter-board v4 wire protocol. */
enum {
  kBoardLinkProtocolVersion = 4U,
  kBoardLinkMaxPayload = 128U,
  kBoardLinkFrameOverhead = 14U,
  kBoardLinkMaxFrameSize = kBoardLinkMaxPayload + kBoardLinkFrameOverhead,
  kBoardLinkParserBufferSize = 512U,
  kBoardLinkParserQueueSize = 4U,

  /* Kept only for source compatibility with the retired local V1 tests. */
  kBoardLinkFrameSize = 10U,
};

typedef enum {
  kBoardLinkRoleUnknown = 0,
  kBoardLinkRoleGimbalController = 1,
  kBoardLinkRoleChassisController = 2,
} BoardLinkRole;

typedef enum {
  kBoardLinkHeartbeat = 0x0001,
  kBoardLinkChassisControl = 0x0101,
  kBoardLinkChassisConstraint = 0x0102,
  kBoardLinkChassisFeedback = 0x0103,
  kBoardLinkOperatorControl = 0x0301,
  kBoardLinkBigYawRequest = 0x0402,
  kBoardLinkBigYawFeedback = 0x0403,
} BoardLinkMessageId;

typedef enum {
  kBoardLinkModeDisabled = 0,
  kBoardLinkModeBodyVelocity = 1,
  kBoardLinkModeSpin = 2,
} BoardLinkChassisMode;

typedef enum {
  kBoardLinkBigYawDisabled = 0,
  kBoardLinkBigYawFollowCenter = 1,
} BoardLinkBigYawMode;

typedef enum {
  kBoardLinkSafetyBoot = 0,
  kBoardLinkSafetyWaiting = 1,
  kBoardLinkSafetyReady = 2,
  kBoardLinkSafetyActive = 3,
  kBoardLinkSafetyDegraded = 4,
  kBoardLinkSafetyEmergencyStop = 5,
  kBoardLinkSafetyConfigBlocked = 6,
} BoardLinkSafetyState;

typedef enum {
  kBoardLinkExecutionWaiting = 0,
  kBoardLinkExecutionStarting = 1,
  kBoardLinkExecutionRunning = 2,
  kBoardLinkExecutionStopping = 3,
  kBoardLinkExecutionFault = 4,
  kBoardLinkExecutionDisabled = 5,
} BoardLinkExecutionState;

typedef struct {
  uint64_t timestamp_ms;
  uint32_t sequence;
  bool valid;
} BoardLinkStamp;

typedef struct {
  BoardLinkRole sender_role;
  uint16_t message_id;
  uint16_t payload_length;
  uint32_t frame_sequence;
  uint64_t received_ms;
  uint8_t payload[kBoardLinkMaxPayload];
} BoardLinkFrame;

typedef struct {
  uint8_t bytes[kBoardLinkParserBufferSize];
  uint64_t byte_start_ms;
  uint16_t length;
  BoardLinkFrame queue[kBoardLinkParserQueueSize];
  uint8_t queue_head;
  uint8_t queue_count;
  uint32_t crc_errors;
  uint32_t length_errors;
  uint32_t version_errors;
  uint32_t unknown_messages;
  uint32_t queue_overflows;
  uint32_t assembly_timeouts;
} BoardLinkParser;

typedef struct {
  BoardLinkRole role;
  uint8_t safety_state;
  uint32_t active_reasons;
  uint32_t sender_uptime_ms;
  uint64_t sender_boot_id;
  bool ready;
  bool sync_requested;
  BoardLinkStamp stamp;
} BoardLinkHeartbeat;

typedef struct {
  uint8_t mode;
  uint8_t source;
  uint8_t global_action;
  float vx_m_s;
  float vy_m_s;
  float wz_rad_s;
  uint32_t active_reasons;
  uint32_t command_sequence;
  uint64_t receiver_boot_id;
  BoardLinkStamp stamp;
} BoardLinkChassisControl;

typedef struct {
  bool output_valid;
  bool output_enabled;
  bool power_valid;
  float power_limit_w;
  float buffer_energy_j;
  uint32_t output_age_ms;
  uint32_t power_age_ms;
  BoardLinkStamp stamp;
} BoardLinkChassisConstraint;

typedef struct {
  uint8_t execution_state;
  uint8_t safety_state;
  bool ready;
  bool armed;
  uint32_t active_reasons;
  uint32_t last_command_sequence;
  uint32_t valid_fields;
  float vx_m_s;
  float vy_m_s;
  float wz_rad_s;
  float power_w;
  BoardLinkStamp stamp;
} BoardLinkChassisFeedback;

typedef struct {
  bool run_allowed;
  bool emergency_stop;
  uint32_t source_sequence;
  uint32_t source_age_ms;
  uint32_t clear_event_id;
  uint32_t clear_event_age_ms;
  uint64_t sender_boot_id;
  uint64_t receiver_boot_id;
  BoardLinkStamp stamp;
} BoardLinkOperatorControl;

typedef struct {
  uint8_t mode;
  bool permission_valid;
  bool permission_enabled;
  float target_rate_rad_s;
  uint32_t source_sequence;
  uint32_t source_age_ms;
  uint32_t command_age_ms;
  uint64_t receiver_boot_id;
  uint32_t permission_age_ms;
  BoardLinkStamp stamp;
} BoardLinkBigYawRequest;

typedef struct {
  uint8_t execution_state;
  bool ready;
  bool armed;
  bool valid;
  float actual_rate_rad_s;
  uint32_t active_reasons;
  uint32_t last_command_sequence;
  uint32_t source_sequence;
  uint32_t production_age_ms;
  BoardLinkStamp stamp;
} BoardLinkBigYawFeedback;

uint16_t BoardLinkCrc16(const uint8_t *data, size_t length);

void BoardLinkParserInit(BoardLinkParser *parser);
void BoardLinkParserFeed(BoardLinkParser *parser, const uint8_t *data,
                         uint16_t length, uint64_t now_ms);
bool BoardLinkParserPop(BoardLinkParser *parser, BoardLinkFrame *frame);
bool BoardLinkSequenceIsNewer(uint32_t sequence, uint32_t previous);

bool BoardLinkDecodeFrame(const uint8_t *frame, size_t length,
                          BoardLinkFrame *decoded);
bool BoardLinkEncodeHeartbeat(const BoardLinkHeartbeat *message,
                              uint32_t frame_sequence, uint8_t *frame,
                              size_t capacity, uint16_t *length);
bool BoardLinkEncodeChassisControl(const BoardLinkChassisControl *message,
                                   uint32_t frame_sequence, uint8_t *frame,
                                   size_t capacity, uint16_t *length);
bool BoardLinkEncodeChassisConstraint(const BoardLinkChassisConstraint *message,
                                      uint32_t frame_sequence, uint8_t *frame,
                                      size_t capacity, uint16_t *length);
bool BoardLinkEncodeChassisFeedback(const BoardLinkChassisFeedback *message,
                                    uint32_t frame_sequence, uint8_t *frame,
                                    size_t capacity, uint16_t *length);
bool BoardLinkEncodeOperatorControl(const BoardLinkOperatorControl *message,
                                    uint32_t frame_sequence, uint8_t *frame,
                                    size_t capacity, uint16_t *length);
bool BoardLinkEncodeBigYawRequest(const BoardLinkBigYawRequest *message,
                                  uint32_t frame_sequence, uint8_t *frame,
                                  size_t capacity, uint16_t *length);
bool BoardLinkEncodeBigYawFeedback(const BoardLinkBigYawFeedback *message,
                                   uint32_t frame_sequence, uint8_t *frame,
                                   size_t capacity, uint16_t *length);

bool BoardLinkDecodeHeartbeat(const BoardLinkFrame *frame,
                              BoardLinkHeartbeat *message);
bool BoardLinkDecodeChassisControl(const BoardLinkFrame *frame,
                                   BoardLinkChassisControl *message);
bool BoardLinkDecodeChassisConstraint(const BoardLinkFrame *frame,
                                      BoardLinkChassisConstraint *message);
bool BoardLinkDecodeChassisFeedback(const BoardLinkFrame *frame,
                                    BoardLinkChassisFeedback *message);
bool BoardLinkDecodeOperatorControl(const BoardLinkFrame *frame,
                                    BoardLinkOperatorControl *message);
bool BoardLinkDecodeBigYawRequest(const BoardLinkFrame *frame,
                                  BoardLinkBigYawRequest *message);
bool BoardLinkDecodeBigYawFeedback(const BoardLinkFrame *frame,
                                   BoardLinkBigYawFeedback *message);

/* Retired V1 codec retained as a compatibility boundary for old callers. */
enum {
  kBoardLinkYaw = 1,
  kBoardLinkStatus = 2,
  kBoardLinkYawValid = 1,
  kBoardLinkStatusImuReady = 1,
  kBoardLinkStatusYawReady = 2,
  kBoardLinkStatusHostControl = 4,
  kBoardLinkStatusDriveReady = 8,
};

typedef struct {
  uint8_t type;
  uint8_t sequence;
  uint8_t flags;
  int16_t value;
} BoardLinkMessage;

void BoardLinkEncode(const BoardLinkMessage *message,
                     uint8_t frame[kBoardLinkFrameSize]);
bool BoardLinkDecode(const uint8_t frame[kBoardLinkFrameSize],
                     BoardLinkMessage *message);
bool BoardLinkEncodeYaw(float relative_yaw_rad, bool valid, uint8_t sequence,
                        uint8_t frame[kBoardLinkFrameSize]);

#endif
