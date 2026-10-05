#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "driver/uart.h"
#include <Keypad.h>

#include "civ_frame.h"
#include "config_pins.h"
#include "radio_profile_types.h"
#include "voice_data.h"

struct VoiceClip {
  const char* name;
  const uint8_t* data;
  size_t len;
};

static constexpr uint8_t CIV_MY_ADDR = 0xE0;
static constexpr uint8_t CIV_CTRL_ADDR = CIV_MY_ADDR;
// Command byte of the radio's "NG" (command rejected) reply.
static constexpr uint8_t CIV_REPLY_NG = 0xFA;
static constexpr uint32_t CIV_PUMP_BUDGET_MS = 3;

static const bool FREQ_SPEAK_START_IMMEDIATELY = false;
static const uint32_t FREQ_SPEAK_IDLE_MS = 1500;
static const bool FREQ_POLL_ENABLE = true;
static const uint32_t FREQ_POLL_MS = 400;
static const uint32_t FREQ_POLL_TIMEOUT_MS = 80;
// FT-8x7 answers slowly (5-byte frames at 4800 baud 8N2, busy CPU while tuning),
// so it gets a longer timeout and a slower cadence to keep the loop responsive.
static const uint32_t FREQ_POLL_MS_FT8X7 = 700;
static const uint32_t FREQ_POLL_TIMEOUT_MS_FT8X7 = 300;
// After this many consecutive failed polls (radio off/disconnected), poll
// rarely so the blocking timeout does not starve keypad scanning.
static const uint8_t FREQ_POLL_BACKOFF_AFTER_FAILURES = 3;
static const uint32_t FREQ_POLL_BACKOFF_MS = 3000;
// Minimum distance from the last announced frequency before tuning is announced again.
static const uint32_t FREQ_SPEAK_MIN_STEP_HZ = 100;
// Minimum silence after a tuning announcement finishes before the next one starts.
static const uint32_t FREQ_SPEAK_MIN_GAP_MS = 500;

static const bool SMETER_POLL_ENABLE = false;
static const uint32_t SMETER_POLL_MS = 350;
static const uint32_t SMETER_POLL_TIMEOUT_MS = 80;
static const bool SMETER_SPEAK_ENABLE = false;
static const uint8_t SMETER_SPEAK_MIN_DELTA_S = 1;
static const uint32_t SMETER_SPEAK_MIN_INTERVAL_MS = 2500;
static const int32_t SMETER_RAW_AT_S0 = 21;
static const int32_t SMETER_RAW_AT_S9 = 297;

// Line settings the protocol needs.
enum class SerialFraming : uint8_t {
  Standard,  // 8N1
  Ft8x7Cat,  // 8N2, with TX driven idle before the UART opens (FT-8x7 and FT-847)
};

// The active link to the radio: the profile's port, and the baud and CI-V
// address in use, which the user may have changed from the profile's defaults.
struct ConnectionProfile {
  RadioPort port;
  uint32_t baud;
  uint8_t civAddr;  // CI-V radios only
  SerialFraming framing;
};

enum NotchWidth : uint8_t {
  NOTCH_WIDTH_NAR = 0,
  NOTCH_WIDTH_MID = 1,
  NOTCH_WIDTH_WIDE = 2,
  NOTCH_WIDTH_UNKNOWN = 0xFF
};

struct BandStackEntry {
  uint8_t bandCode = 0;
  uint8_t registerCode = 0;
  uint64_t freqHz = 0;
  uint8_t mode = 0xFF;
  uint8_t filter = 0xFF;
};

// S-meter reading normalised across protocols: S0..S9, then dB over S9.
struct SMeterReading {
  uint8_t sUnits = 0;    // 0..9
  uint8_t dbOverS9 = 0;  // 0, 10, 20 ... (only when sUnits == 9)
  // Linear scale in S-unit-sized steps (S9+10 = 10) for change detection.
  uint8_t steps() const { return sUnits + dbOverS9 / 10; }
  // "S5", "S9+20dB"
  String toString() const {
    String s = "S" + String(sUnits);
    if (dbOverS9) s += "+" + String(dbOverS9) + "dB";
    return s;
  }
};

struct LiveState {
  bool freqValid = false;
  uint64_t freqHz = 0;
  uint32_t lastFreqMs = 0;
  uint32_t lastFreqPollMs = 0;
  uint8_t freqPollFailures = 0;
  bool freqPollCandidateValid = false;
  uint64_t freqPollCandidateHz = 0;
  bool tuning = false;
  uint64_t pendingHz = 0;
  uint64_t tuningStartSpokenHz = 0;
  uint32_t lastChangeMs = 0;
  uint64_t lastSpokenHz = 0;
  bool modeValid = false;
  uint8_t mode = 0xFF;
  uint32_t lastModeMs = 0;
  bool smValid = false;
  int32_t smRaw = 0;
  SMeterReading sm;
  uint32_t lastSmMs = 0;
  uint8_t lastSpokenSmSteps = 0xFF;
  uint32_t lastSmPollMs = 0;
  uint32_t lastSmSpokenMs = 0;
  bool powerValid = false;
  int32_t powerRaw = 0;
  uint32_t lastPowerMs = 0;
  bool swrValid = false;
  int32_t swrRaw = 0;
  uint32_t lastSwrMs = 0;
  bool nrValid = false;
  bool nrOn = false;
  uint32_t lastNrMs = 0;
  bool nbValid = false;
  bool nbOn = false;
  uint32_t lastNbMs = 0;
  bool notchValid = false;
  bool notchOn = false;
  bool notchWidthValid = false;
  NotchWidth notchWidth = NOTCH_WIDTH_UNKNOWN;
  uint32_t lastNotchMs = 0;
  bool ctcssValid = false;
  uint16_t ctcssTenths = 0;
  bool dcsValid = false;
  uint16_t dcsCode = 0;
  bool activeVfoKnown = false;
  bool activeVfoA = true;
  bool lockKnown = false;
  bool lockOn = false;
};
