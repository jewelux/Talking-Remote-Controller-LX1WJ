#pragma once

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <optional>

// Longest CI-V frame we read: FE FE to from cmd <payload> FD.
static constexpr size_t kCivMaxFrame = 96;
static constexpr size_t kCivFrameOverhead = 6;
static constexpr size_t kCivMaxPayload = kCivMaxFrame - kCivFrameOverhead;

// A decoded CI-V frame. It owns a copy of its payload, so it stays valid after
// the buffer it was read into is reused or goes out of scope.
struct CivFrame {
  uint8_t to = 0;
  uint8_t from = 0;
  uint8_t cmd = 0;
  std::array<uint8_t, kCivMaxPayload> payload{};
  size_t payloadLen = 0;

  // Drops the first n payload bytes, such as an echoed sub-command. Returns
  // false and leaves the frame as it was when the payload is shorter than n.
  bool dropPayloadPrefix(size_t n);
};

// Decodes the n bytes of one frame. Empty when they are not a whole frame or
// the payload does not fit in CivFrame.
std::optional<CivFrame> civDecodeFrame(const uint8_t* buf, size_t n);
