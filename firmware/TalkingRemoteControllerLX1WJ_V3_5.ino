/*
  Talking Remote Controller LX1WJ - ESP32-S3 + Keypad + Speech + Multi-Radio Profiles
  File: TalkingRemoteControllerLX1WJ_V3_5.ino
  Version: V3.5
*/

#include "radio_globals.h"
#include "profile_loader.h"
#include "radio_catalog.h"
#include "radio_monitor.h"
#include "radio_prefs.h"
#include "radio_profile.h"
#include "radio_runtime.h"
#include "sd_slots.h"
#include "engine_civ.h"
#include "engine_kenwood.h"
#include "ui_console.h"
#include "ui_keypad.h"
#include "ui_speech.h"
#include "debug_log.h"

static bool g_bootSpeakPending = false;
static uint32_t g_bootSpeakAtMs = 0;

void setup() {
  Serial.begin(PC_BAUD);
  delay(200);

  seedBuiltInSlots();
  (void)loadProfilesFromSd();
  printProfileSlots();

  initSpeech();

  g_profileId = loadProfileFromNvs(g_profileId);
  g_tuningSpeakEnabled = loadTuningSpeakFromNvs(true);
  applyProfile(g_profileId);

  DBG_PRINT("Talking Remote Controller LX1WJ (V3.5) - Selected radio: ");
  DBG_PRINT(currentProfile().name);
  DBG_PRINT("  CI-V addr=0x");
  if (debugLogEnabled()) Serial.println(g_civRadioAddr, HEX);

  g_bootSpeakPending = true;
  g_bootSpeakAtMs = millis() + 900;

  initKeypadUi();
  printHelp();

  (void)refreshLiveFrequency();
  (void)refreshLiveMode();
}

void loop() {
  pollKeypadUi();

  if (g_bootSpeakPending && (int32_t)(millis() - g_bootSpeakAtMs) >= 0) {
    g_bootSpeakPending = false;
    speakBootProfile();
  }

  pumpIncoming(CIV_PUMP_BUDGET_MS);
  pollFrequencyIfDue();
  speakPendingFreqIfIdle();
  pollSMeterIfDue();
  pollKeypadUi();

  String line = readLine();
  if (line.length()) processCommand(line);
}
