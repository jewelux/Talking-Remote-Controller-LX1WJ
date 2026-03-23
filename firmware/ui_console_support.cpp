#include "ui_console_support.h"

#include "protocol_ascii.h"
#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_profile.h"
#include "radio_utils.h"
#include "ui_keypad.h"

void printModeList() {
  const StoredProfile& sp = currentStoredProfile();
  struct ModeDigitMap {
    char digit;
    uint8_t mode;
  };
  static const ModeDigitMap kModeDigits[] = {
    {'1', 0x00},
    {'2', 0x01},
    {'3', 0x03},
    {'4', 0x02},
    {'5', 0x05},
    {'6', 0x11},
    {'7', 0x04},
    {'8', 0x07},
    {'9', 0x08},
  };
  Serial.println("MODE LIST:");
  for (const auto& it : kModeDigits) {
    String code;
    if (!profileModeCodeForInternal(sp, it.mode, code)) continue;
    Serial.print("  ");
    Serial.print(it.digit);
    Serial.print(" = ");
    Serial.println(modeToString(it.mode));
  }
}

void printStatusSummary() {
  Serial.println("[STATUS]");
  Serial.print("  bank: ");
  Serial.println((int)uiGetBank());
  Serial.print("  profile slot: ");
  Serial.println((int)g_profileId);
  Serial.print("  profile name: ");
  Serial.println(currentProfile().name ? currentProfile().name : "(null)");
  Serial.print("  protocol: ");
  Serial.println(protocolTypeToString(currentProtocolType()));
  Serial.print("  speech: ");
  Serial.println(g_speechEnabled ? "ON" : "OFF");
  Serial.print("  quiet: ");
  Serial.println(g_quiet ? "ON" : "OFF");
  Serial.print("  tuning speech: ");
  Serial.println(g_tuningSpeakEnabled ? "ON" : "OFF");
  Serial.print("  volume: ");
  Serial.println((int)g_volumeLevel);
  if (live.freqValid) {
    Serial.print("  last freq: ");
    Serial.print(hzToMHzString3(live.freqHz));
    Serial.println(" MHz");
  } else {
    Serial.println("  last freq: (unknown)");
  }
  if (live.modeValid) {
    Serial.print("  last mode: ");
    Serial.println(modeToString(live.mode));
  } else {
    Serial.println("  last mode: (unknown)");
  }
  if (live.smValid) {
    Serial.print("  last s-meter raw: ");
    Serial.println(live.smRaw);
  } else {
    Serial.println("  last s-meter raw: (unknown)");
  }
  if (live.powerValid) {
    Serial.print("  last power raw: ");
    Serial.println(live.powerRaw);
  } else {
    Serial.println("  last power raw: (unknown)");
  }
  if (live.swrValid) {
    Serial.print("  last swr raw: ");
    Serial.println(live.swrRaw);
  } else {
    Serial.println("  last swr raw: (unknown)");
  }
}

uint8_t findAdjacentValidProfile(int8_t direction) {
  if (direction == 0) return g_profileId;
  for (uint8_t step = 0; step < MAX_PROFILE_SLOTS; ++step) {
    int next = (int)g_profileId + direction * (step + 1);
    while (next < 1) next += MAX_PROFILE_SLOTS;
    while (next > MAX_PROFILE_SLOTS) next -= MAX_PROFILE_SLOTS;
    if (storedProfileForId((uint8_t)next)) return (uint8_t)next;
  }
  return g_profileId;
}
