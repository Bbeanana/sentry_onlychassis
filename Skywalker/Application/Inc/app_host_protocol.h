#ifndef APP_HOST_PROTOCOL_H
#define APP_HOST_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

// communication.md: AA 06 vx_lo vx_hi vy_lo vy_hi wz_lo wz_hi xor.
enum { kHostVelocityFrameLength = 9U };

typedef struct {
  float vx; // m/s, positive toward current vehicle front, including rotation.
  float vy; // m/s, positive toward current vehicle left, including rotation.
  float wz; // rad/s, positive counterclockwise.
  uint32_t last_update_ms;
  uint32_t frame_count;
  uint8_t raw_frame[kHostVelocityFrameLength]; // Same validated frame as vx/vy/wz.
} HostVelocityCommand;

// Call before enabling UART reception.
void HostVelocityProtocolInit(void);
void HostVelocityProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length);

// Diagnostic snapshot of the latest valid frame, including when it is stale.
// Returns false and zeroes the output if no valid frame has arrived.
bool HostVelocityProtocolGetLatestCommand(HostVelocityCommand *command);

// Copy one coherent sample. Returns false and zeroes the output when no valid
// frame has arrived or its age exceeds timeout_ms. Time base: HAL_GetTick().
bool HostVelocityProtocolGetCommand(HostVelocityCommand *command,
                                    uint32_t timeout_ms);

#endif // APP_HOST_PROTOCOL_H
