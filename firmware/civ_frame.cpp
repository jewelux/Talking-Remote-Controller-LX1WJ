#include "civ_frame.h"

#include <algorithm>

bool CivFrame::dropPayloadPrefix(size_t n) {
  if (n > payloadLen) return false;
  std::copy(payload.begin() + n, payload.begin() + payloadLen, payload.begin());
  payloadLen -= n;
  return true;
}

std::optional<CivFrame> civDecodeFrame(const uint8_t* buf, size_t n) {
  if (!buf || n < kCivFrameOverhead) return std::nullopt;
  if (buf[0] != 0xFE || buf[1] != 0xFE || buf[n - 1] != 0xFD) return std::nullopt;
  const size_t payloadLen = n - kCivFrameOverhead;
  if (payloadLen > kCivMaxPayload) return std::nullopt;
  CivFrame f;
  f.to = buf[2];
  f.from = buf[3];
  f.cmd = buf[4];
  std::copy_n(buf + 5, payloadLen, f.payload.begin());
  f.payloadLen = payloadLen;
  return f;
}
