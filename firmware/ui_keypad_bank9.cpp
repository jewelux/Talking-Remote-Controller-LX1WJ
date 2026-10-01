// Bank 9 keypad actions: profiles, tuning speech and volume.
#include "ui_keypad_bank.h"
#include "radio_monitor.h"
#include "radio_prefs.h"
#include "radio_profile.h"
#include "ui_console_support.h"
#include "ui_keypad.h"

void speakTuningSpeechState() {
  if (!g_speechEnabled) return;
  speakTokenState("tune frequency", g_tuningSpeakEnabled);
}

void setTuningSpeechEnabled(bool enabled) {
  g_tuningSpeakEnabled = enabled;
  saveTuningSpeakToNvs(g_tuningSpeakEnabled);
  if (!g_tuningSpeakEnabled) cancelPendingFreqAnnouncement();
}

// Always with its name: alone, "off" would not say what verbose off is.
void speakVerboseState() {
  if (!g_speechEnabled) return;
  speakToken("verbose");
  playSilenceMs(60);
  speakToken(g_verboseSpeech ? "on" : "off");
}

void setVerboseSpeech(bool verbose) {
  g_verboseSpeech = verbose;
  saveVerboseToNvs(g_verboseSpeech);
}

void beginBank9ProfileSelect() {
  keypadBeginProfileSelect();
  printKeypadAction("PROFILE SELECT");
  printKeypadStatus("PROFILE PLEASE");
  speakPrompt("profile");
}

void selectNextProfile() {
  printKeypadAction("PROFILE NEXT");
  uint8_t next = findAdjacentValidProfile(1);
  applyProfile(next);
  printKeypadStatus("PROFILE {}", next);
  speakCurrentProfile();
}

void selectPrevProfile() {
  printKeypadAction("PROFILE PREV");
  uint8_t prev = findAdjacentValidProfile(-1);
  applyProfile(prev);
  printKeypadStatus("PROFILE {}", prev);
  speakCurrentProfile();
}

void queryBank9TuningSpeech() {
  printKeypadAction("TUNINGSPEECH?");
  printKeypadStatus("TUNINGSPEECH {}", g_tuningSpeakEnabled ? "ON" : "OFF");
  speakTuningSpeechState();
}

void toggleBank9TuningSpeech() {
  printKeypadAction("TUNINGSPEECH");
  setTuningSpeechEnabled(!g_tuningSpeakEnabled);
  printKeypadStatus("TUNINGSPEECH {}", g_tuningSpeakEnabled ? "ON" : "OFF");
  speakTuningSpeechState();
}

void queryBank9Verbose() {
  printKeypadAction("VERBOSE?");
  printKeypadStatus("VERBOSE {}", g_verboseSpeech ? "ON" : "OFF");
  speakVerboseState();
}

void toggleBank9Verbose() {
  printKeypadAction("VERBOSE");
  setVerboseSpeech(!g_verboseSpeech);
  printKeypadStatus("VERBOSE {}", g_verboseSpeech ? "ON" : "OFF");
  speakVerboseState();
}

// "volume" and the level.
static void speakVolumeWithLevel(uint8_t lvl) {
  speakLabel("volume");
  speakVolumeLevel(lvl);
}

void adjustBank9Volume(int delta) {
  printKeypadAction("VOLUME {}{}", delta < 0 ? "DOWN" : "UP", delta < -1 || delta > 1 ? " FAST" : "");
  int next = (int)g_volumeLevel + delta;
  if (next < 1) next = 1;
  if (next > 9) next = 9;
  applyVolumeLevel((uint8_t)next);
  saveVolumeToNvs((uint8_t)next);
  printKeypadStatus("VOLUME {}", next);
  if (g_speechEnabled) {
    speakVolumeWithLevel((uint8_t)next);
    speakValueOk();
  }
}

void queryBank9Volume() {
  printKeypadAction("VOLUME?");
  printKeypadStatus("VOLUME {}", g_volumeLevel);
  if (g_speechEnabled) speakVolumeWithLevel(g_volumeLevel);
}

void queryBank9Profile() {
  printKeypadAction("PROFILE?");
  printKeypadStatus("PROFILE CURRENT");
  if (!g_speechEnabled) return;
  speakLabel("profile");
  speakProfileIdentityFromSlot(g_profileId, false);
}

void selectBank9DirectProfile(char key) {
  if (key < '1' || key > '9') return;
  const uint8_t slot = (uint8_t)(key - '0');
  if (!storedProfileForId(slot)) {
    printKeypadAction("PROFILE");
    printKeypadStatus("PROFILE {} EMPTY", slot);
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  printKeypadAction("PROFILE {}", slot);
  applyProfile(slot);
  printKeypadStatus("PROFILE {}", slot);
  speakCurrentProfile();
}
