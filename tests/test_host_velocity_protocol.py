"""Run the actual UART7 velocity parser with a host HAL/interrupt stub."""

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]

HAL_STUB = r"""
#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H
#include <stdint.h>
extern uint32_t test_tick;
extern uint32_t test_primask;
static inline uint32_t HAL_GetTick(void) { return test_tick; }
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __set_PRIMASK(uint32_t mask) { test_primask = mask; }
#endif
"""

HARNESS = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_host_protocol.h"

uint32_t test_tick;
uint32_t test_primask;

static void check(float actual, float expected) {
  assert(fabsf(actual - expected) < 0.00001f);
}

static void frame(uint8_t *dst, int16_t vx, int16_t vy, int16_t wz) {
  int16_t values[3] = {vx, vy, wz};
  dst[0] = 0xAA;
  dst[1] = 0x06;
  for (unsigned i = 0; i < 3; ++i) {
    dst[2 + i * 2] = (uint8_t)(uint16_t)values[i];
    dst[3 + i * 2] = (uint8_t)((uint16_t)values[i] >> 8);
  }
  dst[8] = 0;
  for (unsigned i = 0; i < 8; ++i) dst[8] ^= dst[i];
}

int main(void) {
  HostVelocityCommand command = {.vx = 1.0f};
  HostVelocityProtocolInit();
  assert(!HostVelocityProtocolGetCommand(&command, 100));
  check(command.vx, 0.0f);
  assert(!HostVelocityProtocolGetLatestCommand(&command));
  assert(!HostVelocityProtocolGetLatestCommand(NULL));
  assert(!HostVelocityProtocolGetCommand(NULL, 100));
  HostVelocityProtocolRxCallback(NULL, 10);
  uint8_t sample[9] = {0xAA, 0x06, 0xE8, 0x03, 0, 0, 0, 0, 0x47};
  HostVelocityProtocolRxCallback(sample, 0);
  assert(!HostVelocityProtocolGetCommand(&command, 100));

  // Every split boundary, including one byte per callback.
  for (unsigned split = 1; split < 9; ++split) {
    HostVelocityProtocolInit();
    test_tick = 42;
    HostVelocityProtocolRxCallback(sample, split);
    assert(!HostVelocityProtocolGetCommand(&command, 100));
    HostVelocityProtocolRxCallback(sample + split, 9 - split);
    assert(HostVelocityProtocolGetCommand(&command, 100));
    check(command.vx, 1.0f);
    check(command.vy, 0.0f);
    check(command.wz, 0.0f);
    assert(command.frame_count == 1 && command.last_update_ms == 42);
    assert(memcmp(command.raw_frame, sample, 9) == 0);
  }
  HostVelocityProtocolInit();
  for (unsigned i = 0; i < 9; ++i)
    HostVelocityProtocolRxCallback(sample + i, 1);
  assert(HostVelocityProtocolGetCommand(&command, 100));
  assert(command.frame_count == 1);

  // Concatenated frames, signed limits, and a header byte inside the payload.
  uint8_t batch[18];
  frame(batch, -1500, 250, -2000);
  frame(batch + 9, -32768, 32767, 170);
  HostVelocityProtocolRxCallback(batch, sizeof(batch));
  assert(HostVelocityProtocolGetCommand(&command, 100));
  check(command.vx, -32.768f);
  check(command.vy, 32.767f);
  check(command.wz, 0.170f);
  assert(command.frame_count == 3);
  assert(memcmp(command.raw_frame, batch + 9, 9) == 0);

  // Bad checksum and wrong length must not update data or refresh the timestamp.
  test_tick = 100;
  uint8_t bad[9];
  memcpy(bad, sample, 9);
  bad[8] ^= 1;
  HostVelocityProtocolRxCallback(bad, 9);
  memcpy(bad, sample, 9);
  bad[1] = 7;
  HostVelocityProtocolRxCallback(bad, 9);
  assert(HostVelocityProtocolGetCommand(&command, 100));
  assert(command.frame_count == 3 && command.last_update_ms == 42);
  assert(memcmp(command.raw_frame, batch + 9, 9) == 0);

  // False header overlaps a valid frame: resynchronize by sliding one byte.
  HostVelocityProtocolInit();
  uint8_t overlap[12] = {0xAA, 0x06, 0x55};
  memcpy(overlap + 3, sample, 9);
  HostVelocityProtocolRxCallback(overlap, sizeof(overlap));
  assert(HostVelocityProtocolGetCommand(&command, 100));
  check(command.vx, 1.0f);
  assert(command.frame_count == 1);

  // A large DMA chunk must process all frames, without truncating the stream.
  HostVelocityProtocolInit();
  uint8_t large[512];
  memset(large, 0x55, sizeof(large));
  for (unsigned i = 0; i < 56; ++i) memcpy(large + i * 9, sample, 9);
  HostVelocityProtocolRxCallback(large, sizeof(large));
  assert(HostVelocityProtocolGetCommand(&command, 100));
  assert(command.frame_count == 56);

  test_tick = 200;
  assert(HostVelocityProtocolGetCommand(&command, 100));
  test_tick = 201;
  assert(!HostVelocityProtocolGetCommand(&command, 100));
  check(command.vx, 0.0f);
  check(command.vy, 0.0f);
  check(command.wz, 0.0f);
  assert(HostVelocityProtocolGetLatestCommand(&command));
  check(command.vx, 1.0f);
  assert(command.frame_count == 56 && command.last_update_ms == 100);
  assert(memcmp(command.raw_frame, sample, 9) == 0);
  // Timeout arithmetic remains correct across the uint32 millisecond wrap.
  test_tick = UINT32_MAX - 10;
  HostVelocityProtocolRxCallback(sample, 9);
  test_tick = 9;
  assert(HostVelocityProtocolGetCommand(&command, 20));
  test_tick = 10;
  assert(!HostVelocityProtocolGetCommand(&command, 20));

  test_primask = 1;
  HostVelocityProtocolInit();
  assert(test_primask == 1);
  assert(!HostVelocityProtocolGetCommand(&command, 100));
  assert(test_primask == 1);
  test_primask = 0;
  HostVelocityProtocolGetCommand(&command, 100);
  assert(test_primask == 0);
  puts("PASS: UART7 velocity decoding, framing, resync, signed limits and timeout");
  return 0;
}
"""

with tempfile.TemporaryDirectory(prefix="host_velocity_protocol_") as directory:
    work = Path(directory)
    (work / "stm32h7xx_hal.h").write_text(HAL_STUB, encoding="utf-8")
    (work / "test.c").write_text(HARNESS, encoding="utf-8")
    executable = work / "test.exe"
    subprocess.run(
        [
            "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(work),
            "-I", str(ROOT / "Skywalker/Application/Inc"),
            str(work / "test.c"),
            str(ROOT / "Skywalker/Application/Src/app_host_protocol.c"),
            "-lm", "-o", str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)
