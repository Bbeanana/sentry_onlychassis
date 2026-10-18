#include "app_host_protocol.h"

#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"

static uint8_t g_host_rx_frame[kHostVelocityFrameLength];
static uint8_t g_host_rx_length;
static volatile HostVelocityCommand g_host_velocity_command;
static volatile bool g_host_has_command;

static float HostVelocityDecode(const uint8_t *data) {
  uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  // Explicit sign extension avoids implementation-defined unsigned->signed casts.
  int32_t value = raw >= 0x8000U ? (int32_t)raw - 65536 : (int32_t)raw;
  return (float)value * 0.001f;
}

void HostVelocityProtocolInit(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_host_rx_length = 0U;
  g_host_velocity_command = (HostVelocityCommand){0};
  g_host_has_command = false;
  __set_PRIMASK(primask);
}

void HostVelocityProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length) {
  if (rx_buffer == NULL) {
    return;
  }
  for (uint16_t i = 0U; i < rx_length; ++i) {
    g_host_rx_frame[g_host_rx_length++] = rx_buffer[i];
    // Keep incomplete frames across DMA callbacks. On any framing/checksum
    // error, slide one byte so an overlapping next header is not lost.
    while (g_host_rx_length != 0U) {
      if (g_host_rx_frame[0] == 0xAAU &&
          (g_host_rx_length < 2U || g_host_rx_frame[1] == 0x06U)) {
        if (g_host_rx_length < kHostVelocityFrameLength) {
          break;
        }
        uint8_t checksum = 0U;
        for (uint8_t j = 0U; j < kHostVelocityFrameLength - 1U; ++j) {
          checksum ^= g_host_rx_frame[j];
        }
        if (checksum == g_host_rx_frame[kHostVelocityFrameLength - 1U]) {
          // Wire order is vx (forward), vy (left), wz (counterclockwise).
          // All three are signed little-endian int16 values scaled by 0.001.
          g_host_velocity_command.vx = HostVelocityDecode(&g_host_rx_frame[2]);
          g_host_velocity_command.vy = HostVelocityDecode(&g_host_rx_frame[4]);
          g_host_velocity_command.wz = HostVelocityDecode(&g_host_rx_frame[6]);
          for (uint8_t j = 0U; j < kHostVelocityFrameLength; ++j) {
            g_host_velocity_command.raw_frame[j] = g_host_rx_frame[j];
          }
          g_host_velocity_command.last_update_ms = HAL_GetTick();
          g_host_velocity_command.frame_count++;
          g_host_has_command = true;
          g_host_rx_length = 0U;
          break;
        }
      }
      --g_host_rx_length;
      memmove(g_host_rx_frame, &g_host_rx_frame[1], g_host_rx_length);
    }
  }
}

bool HostVelocityProtocolGetLatestCommand(HostVelocityCommand *command) {
  if (command == NULL) {
    return false;
  }
  // UART callbacks run in interrupt context; prevent a mixed vx/vy/wz sample.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  HostVelocityCommand sample = g_host_velocity_command;
  bool has_command = g_host_has_command;
  __set_PRIMASK(primask);
  if (!has_command) {
    *command = (HostVelocityCommand){0};
    return false;
  }
  *command = sample;
  return true;
}

bool HostVelocityProtocolGetCommand(HostVelocityCommand *command,
                                    uint32_t timeout_ms) {
  if (command == NULL) {
    return false;
  }
  if (!HostVelocityProtocolGetLatestCommand(command) ||
      (uint32_t)(HAL_GetTick() - command->last_update_ms) > timeout_ms) {
    *command = (HostVelocityCommand){0};
    return false;
  }
  return true;
}
