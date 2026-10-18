"""Compile real board codec/receiver; exercise framing, validity and freshness."""
from pathlib import Path
import binascii
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
stub = """
#include <stdint.h>
extern uint32_t tick, primask;
static inline uint32_t HAL_GetTick(void) { return tick; }
static inline uint32_t __get_PRIMASK(void) { return primask; }
static inline void __disable_irq(void) { primask = 1; }
static inline void __set_PRIMASK(uint32_t value) { primask = value; }
"""
payload = bytes([0xA5, 0x5A, 1, 1, 7, 1]) + (1571).to_bytes(2, "little")
golden = payload + binascii.crc_hqx(payload, 0xFFFF).to_bytes(2, "little")
harness = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_gimbal_link.h"
uint32_t tick, primask;
static void near(float a, float b) { assert(fabsf(a-b) < 0.0011f); }
static void send(float angle, bool valid, uint8_t sequence) {
  uint8_t frame[kBoardLinkFrameSize];
  assert(BoardLinkEncodeYaw(angle, valid, sequence, frame));
  GimbalLinkRxCallback(frame, sizeof(frame));
}
int main(void) {
  GimbalYawFeedback feedback;
  uint8_t frame[kBoardLinkFrameSize];
  uint8_t golden[kBoardLinkFrameSize] = {GOLDEN};
  assert(BoardLinkEncodeYaw(1.571f, true, 7, frame));
  assert(memcmp(frame, golden, sizeof(frame)) == 0);
  for (unsigned split = 0; split <= sizeof(frame); ++split) {
    GimbalLinkInit(); tick = 100;
    assert(!GimbalLinkGetYaw(&feedback));
    GimbalLinkRxCallback(frame, split);
    GimbalLinkRxCallback(frame + split, sizeof(frame) - split);
    assert(GimbalLinkGetYaw(&feedback)); near(feedback.relative_yaw_rad, 1.571f);
    assert(feedback.sequence == 7 && feedback.frame_count == 1);
  }
  // A corrupted bit anywhere in a frame cannot replace a valid sample.
  for (unsigned byte = 0; byte < sizeof(frame); ++byte) {
    for (unsigned bit = 0; bit < 8; ++bit) {
      GimbalLinkInit(); send(0, true, 1); tick = 101;
      memcpy(frame, golden, sizeof(frame)); frame[byte] ^= 1U << bit;
      GimbalLinkRxCallback(frame, sizeof(frame));
      assert(GimbalLinkGetYaw(&feedback)); near(feedback.relative_yaw_rad, 0);
      assert(feedback.sequence == 1);
      send(-1.57f, true, 2); // resync after corrupted data, even header overlap
      assert(GimbalLinkGetYaw(&feedback)); near(feedback.relative_yaw_rad, -1.57f);
    }
  }
  GimbalLinkInit(); tick = 100;
  send(-1.57f, true, 255); send(0.5f, true, 0); // sequence wraps normally
  tick = 150; assert(GimbalLinkGetYaw(&feedback));
  send(0.5f, true, 0); // duplicate must not refresh receive time
  tick = 151; assert(!GimbalLinkGetYaw(&feedback));
  send(0.5f, true, 0); assert(!GimbalLinkGetYaw(&feedback));
  send(0.5f, true, 1); assert(GimbalLinkGetYaw(&feedback));
  send(0.5f, false, 1); assert(!GimbalLinkGetYaw(&feedback));
  assert(GimbalLinkGetLatest(&feedback) && !feedback.valid);
  send(0.5f, true, 2); assert(GimbalLinkGetYaw(&feedback));
  // Tick rollover and no-refresh on out-of-range or reserved-bit messages.
  tick = UINT32_MAX - 20; send(0, true, 3);
  tick = 29; assert(GimbalLinkGetYaw(&feedback));
  tick = 30; assert(!GimbalLinkGetYaw(&feedback));
  BoardLinkMessage bad = {.type=kBoardLinkYaw, .sequence=4, .flags=1, .value=3143};
  BoardLinkEncode(&bad, frame); GimbalLinkRxCallback(frame, sizeof(frame));
  assert(!GimbalLinkGetYaw(&feedback));
  bad.value=0; bad.flags=3;
  BoardLinkEncode(&bad, frame); GimbalLinkRxCallback(frame, sizeof(frame));
  assert(!GimbalLinkGetYaw(&feedback));
  bad.type=kBoardLinkStatus; bad.flags=0;
  BoardLinkEncode(&bad, frame); GimbalLinkRxCallback(frame, sizeof(frame));
  assert(!GimbalLinkGetYaw(&feedback));
  // Multiple frames and arbitrary noise in one DMA callback.
  uint8_t stream[23] = {0x00, 0xA5, 0x00};
  assert(BoardLinkEncodeYaw(0.7f, true, 5, stream+3));
  assert(BoardLinkEncodeYaw(-0.8f, true, 6, stream+13));
  GimbalLinkRxCallback(stream,sizeof(stream));
  assert(GimbalLinkGetYaw(&feedback)); near(feedback.relative_yaw_rad,-0.8f);
  primask=1; assert(GimbalLinkGetYaw(&feedback)); assert(primask==1);
  assert(!BoardLinkEncodeYaw(NAN,true,7,frame));
  assert(!BoardLinkEncodeYaw(INFINITY,true,7,frame));
  send(7.0f,true,7); assert(GimbalLinkGetYaw(&feedback));
  near(feedback.relative_yaw_rad, 7.0f-6.283185307f);
  send(NAN,false,8); assert(!GimbalLinkGetYaw(&feedback));
  puts("PASS: gimbal link CRC, split frames, resync, signed yaw, validity and timeout");
}
""".replace("GOLDEN", ",".join(str(v) for v in golden))

if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="gimbal_link_") as directory:
        work = Path(directory)
        (work / "stm32h7xx_hal.h").write_text(stub)
        (work / "test.c").write_text(harness)
        exe = work / "test"
        subprocess.run([
            "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
            "-I", str(work), "-I", str(ROOT / "Skywalker/Algorithm/Inc"),
            "-I", str(ROOT / "Skywalker/Application/Inc"), str(work / "test.c"),
            str(ROOT / "Skywalker/Algorithm/Src/alg_board_link.c"),
            str(ROOT / "Skywalker/Application/Src/app_gimbal_link.c"),
            "-lm", "-o", str(exe),
        ], check=True)
        subprocess.run([str(exe)], check=True)
