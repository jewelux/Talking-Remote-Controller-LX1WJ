// Bank 9 keypad actions: profiles, tuning speech and volume.
#include "ui_keypad_bank.h"
#include "radio_monitor.h"
#include "radio_prefs.h"
#include "radio_profile.h"
#include "ui_console_support.h"
#include "ui_keypad.h"

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
  keypadBeginProfileSelect();
  printKeypadAction("PROFILE SELECT");
  printKeypadStatus("PROFILE PLEASE");
  speakPrompt("profile");
}

void selectNextProfile() {
  printKeypadAction("PROFILE NEXT");
  uint8_t next = findAdjacentValidProfile(1);
  applyProfile(next);
  printKeypadStatus(String("PROFILE ") + String((int)next));
  speakCurrentProfile();
}

void selectPrevProfile() {
  printKeypadAction("PROFILE PREV");
  uint8_t prev = findAdjacentValidProfile(-1);
  applyProfile(prev);
  printKeypadStatus(String("PROFILE ") + String((int)prev));
  speakCurrentProfile();
}

void queryBank9TuningSpeech() {
  printKeypadAction("TUNINGSPEECH?");
  printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
  speakTuningSpeechState();
}

void toggleBank9TuningSpeech() {
  printKeypadAction("TUNINGSPEECH");
  setTuningSpeechEnabled(!g_tuningSpeakEnabled);
  printKeypadStatus(String("TUNINGSPEECH ") + (g_tuningSpeakEnabled ? "ON" : "OFF"));
  speakTuningSpeechState();
}

// "volume" and the level.
static void speakVolumeWithLevel(uint8_t lvl) {
  speakToken("volume");
  playSilenceMs(60);
  speakVolumeLevel(lvl);
}

void adjustBank9Volume(int delta) {
  printKeypadAction(String(delta < 0 ? "VOLUME DOWN" : "VOLUME UP") + (delta < -1 || delta > 1 ? " FAST" : ""));
  int next = (int)g_volumeLevel + delta;
  if (next < 1) next = 1;
  if (next > 9) next = 9;
  applyVolumeLevel((uint8_t)next);
  saveVolumeToNvs((uint8_t)next);
  printKeypadStatus(String("VOLUME ") + String(next));
  if (g_speechEnabled) {
    speakVolumeWithLevel((uint8_t)next);
    playSilenceMs(60);
    speakToken("ok");
  }
}

void queryBank9Volume() {
  printKeypadAction("VOLUME?");
  printKeypadStatus(String("VOLUME ") + String((int)g_volumeLevel));
  if (g_speechEnabled) speakVolumeWithLevel(g_volumeLevel);
}

void queryBank9Profile() {
  printKeypadAction("PROFILE?");
  printKeypadStatus("PROFILE CURRENT");
  if (!g_speechEnabled) return;
  speakToken("profile");
  playSilenceMs(60);
  speakProfileIdentityFromSlot(g_profileId, false);
}

void selectBank9DirectProfile(char key) {
  if (key < '1' || key > '9') return;
  const uint8_t slot = (uint8_t)(key - '0');
  if (!storedProfileForId(slot)) {
    printKeypadAction("PROFILE");
    printKeypadStatus(String("PROFILE ") + String((int)slot) + " EMPTY");
    if (g_speechEnabled) speakNotAvailable();
    return;
  }
  printKeypadAction(String("PROFILE ") + String((int)slot));
  applyProfile(slot);
  printKeypadStatus(String("PROFILE ") + String((int)slot));
  speakCurrentProfile();
}
