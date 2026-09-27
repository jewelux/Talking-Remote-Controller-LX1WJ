// Bank 9 keypad actions: profiles, tuning speech and volume.
#include "keypad_actions.h"
#include "debug_log.h"
#include "engine_civ.h"
#include "packet_ascii.h"
#include "protocol_ascii.h"
#include "protocol_ops_yaesu.h"
#include "radio_catalog.h"
#include "radio_frequency.h"
#include "radio_mode.h"
#include "radio_monitor.h"
#include "radio_profile.h"
#include "radio_prefs.h"
#include "radio_protocol.h"
#include "radio_runtime.h"
#include "radio_state.h"
#include "radio_utils.h"
#include "sd_slots.h"
#include "ui_console_support.h"
#include "ui_keypad.h"
#include "ui_keypad_common.h"
#include "ui_speech.h"

static void speakChoosePlease() {
  if (!g_speechEnabled) return;
  speakToken("choose");
  playSilenceMs(60);
  speakToken("please");
}

void speakTuningSpeechState() {
  if (!g_speechEnabled) return;
  speakToken("tune");
  playSilenceMs(60);
  speakFrequencyWord();
  playSilenceMs(60);
  speakToken(g_tuningSpeakEnabled ? "on" : "off");
}

void setTuningSpeechEnabled(bool enabled) {
  g_tuningSpeakEnabled = enabled;
  saveTuningSpeakToNvs(g_tuningSpeakEnabled);
  if (!g_tuningSpeakEnabled) cancelPendingFreqAnnouncement();
}

void beginBank9ProfileSelect() {
  keypadInput().beginProfileSelect();
  printKeypadCommand("BANK9 A LONG -> PROFILE SELECT");
  printKeypadStatus("CHOOSE PLEASE");
  speakChoosePlease();
}

void selectNextProfile() {
  printKeypadCommand("BANK9 B SHORT -> PROFILE NEXT");
  uint8_t next = findAdjacentValidProfile(1);
  applyProfile(next);
  printKeypadStatus(String("PROFILE ") + String((int)next));
  speakCurrentProfile();
}

void selectPrevProfile() {
  printKeypadCommand("BANK9 C SHORT -> PROFILE PREV");
  uint8_t prev = findAdjacentValidProfile(-1);
  applyProfile(prev);
  printKeypadStatus(String("PROFILE ") + String((int)prev));
  speakCurrentProfile();
}

void queryBank9TuningSpeech() {
  printKeypadCommand("BANK9 4 SHORT -> TUNINGSPEECH?");
  printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
  speakTuningSpeechState();
}

void toggleBank9TuningSpeech() {
  printKeypadCommand("BANK9 4 LONG -> TUNINGSPEECH");
  setTuningSpeechEnabled(!g_tuningSpeakEnabled);
  printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
  speakTuningSpeechState();
}

void adjustBank9Volume(int delta) {
  switch (delta) {
    case -1: printKeypadCommand("BANK9 7 SHORT -> VOLUME DOWN"); break;
    case 1: printKeypadCommand("BANK9 8 SHORT -> VOLUME UP"); break;
    case -2: printKeypadCommand("BANK9 7 LONG -> VOLUME DOWN FAST"); break;
    case 2: printKeypadCommand("BANK9 8 LONG -> VOLUME UP FAST"); break;
    default: break;
  }
  int next = (int)g_volumeLevel + delta;
  if (next < 1) next = 1;
  if (next > 9) next = 9;
  applyVolumeLevel((uint8_t)next);
  saveVolumeToNvs((uint8_t)next);
  printKeypadStatus(String("VOLUME ") + String(next));
  if (g_speechEnabled) {
    speakVolumeLevel((uint8_t)next);
    playSilenceMs(60);
    speakToken("ok");
  }
}

void queryBank9Volume() {
  printKeypadCommand("BANK9 9 SHORT -> VOLUME?");
  printKeypadStatus(String("VOLUME ") + String((int)g_volumeLevel));
  if (g_speechEnabled) speakVolumeLevel(g_volumeLevel);
}

void queryBank9Profile() {
  printKeypadCommand("BANK9 A SHORT -> PROFILE?");
  printKeypadStatus("PROFILE CURRENT");
  speakCurrentProfile();
}

bool selectBank9DirectProfile(char key) {
  if (!lightIcomFallbackActive()) return false;
  if (key < '1' || key > '9') return false;
  const uint8_t slot = (uint8_t)(key - '0');
  if (!storedProfileForId(slot)) {
    printKeypadCommand(String("BANK9 ") + key + " SHORT -> PROFILE");
    printKeypadStatus(String("PROFILE ") + String((int)slot) + " EMPTY");
    if (g_speechEnabled) speakNotAvailable();
    return true;
  }
  printKeypadCommand(String("BANK9 ") + key + " SHORT -> PROFILE " + String((int)slot));
  applyProfile(slot);
  printKeypadStatus(String("PROFILE ") + String((int)slot));
  speakCurrentProfile();
  return true;
}
