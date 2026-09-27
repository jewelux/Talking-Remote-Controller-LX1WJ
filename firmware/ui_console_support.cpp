#include "ui_console_support.h"

#include "radio_catalog.h"
#include "radio_mode.h"
#include "radio_profile.h"
#include "radio_protocol.h"
#include "radio_utils.h"
#include "ui_keypad.h"

// Lists what mode select accepts: the mode digits the current profile can set.
void printModeList() {
  Serial.println("MODE LIST:");
  for (char digit = '1'; digit <= '9'; ++digit) {
    uint8_t mode = 0;
    if (!modeFromDigit(digit, mode) || !canSetMode(mode)) continue;
    Serial.print("  ");
    Serial.print(digit);
    Serial.print(" = ");
    Serial.println(modeToString(mode));
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
