"""Integration test for v4 frames entering the application snapshot."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#include <stdint.h>
extern uint32_t tick, primask;
static inline uint32_t HAL_GetTick(void) { return tick; }
static inline uint32_t __get_PRIMASK(void) { return primask; }
static inline void __disable_irq(void) { primask = 1; }
static inline void __set_PRIMASK(uint32_t value) { primask = value; }
'''
HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "app_gimbal_link.h"
uint32_t tick = 100, primask = 0;
int main(void) {
  uint8_t frame[kBoardLinkMaxFrameSize];
  uint16_t length = 0;
  BoardLinkChassisControl source = {
      .mode = kBoardLinkModeBodyVelocity,
      .source = 1,
      .global_action = 2,
      .vx_m_s = 1.0f,
      .vy_m_s = -0.25f,
      .wz_rad_s = 2.0f,
      .command_sequence = 9,
      .receiver_boot_id = 123,
  };
  assert(BoardLinkEncodeChassisControl(&source, 4, frame, sizeof(frame), &length));
  GimbalLinkInit();
  GimbalLinkRxCallback(frame, 7);
  GimbalLinkRxCallback(frame + 7, (uint16_t)(length - 7));
  BoardLinkChassisControl received;
  assert(GimbalLinkGetChassisControl(&received));
  assert(fabsf(received.vx_m_s - 1.0f) < 0.0001f);
  assert(fabsf(received.vy_m_s + 0.25f) < 0.0001f);
  assert(received.command_sequence == 9);
  tick = 201;
  assert(!GimbalLinkGetChassisControl(&received));
  puts("PASS: application v4 control snapshot and freshness");
}
'''

with tempfile.TemporaryDirectory(prefix="app_gimbal_link_v4_") as directory:
    work = Path(directory)
    (work / "stm32h7xx_hal.h").write_text(STUB)
    (work / "test.c").write_text(HARNESS)
    exe = work / "test"
    subprocess.run([
        "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-I", str(work), "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
        "-I", str(ROOT / "Skywalker/Application/Inc"),
        str(work / "test.c"),
        str(ROOT / "Skywalker/Algorithm/Src/alg_board_link.c"),
        str(ROOT / "Skywalker/Application/Src/app_gimbal_link.c"),
        "-lm", "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
