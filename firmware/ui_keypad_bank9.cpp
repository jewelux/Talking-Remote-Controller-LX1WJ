// Bank 9 keypad actions: profiles, tuning speech, volume and speech speed.
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
  uint8_t next = adjacentProfileSlot(g_profileId, 1);
  applyProfile(next);
  printKeypadStatus("PROFILE {}", next);
  speakCurrentProfile();
}

void selectPrevProfile() {
  printKeypadAction("PROFILE PREV");
  uint8_t prev = adjacentProfileSlot(g_profileId, -1);
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

void queryBank9Speed() {
  printKeypadAction("SPEED?");
  printKeypadStatus("SPEED {}", speechSpeedName(g_speechSpeed));
  speakSpeechSpeed();
}

// Slow, normal, fast, slow again; said at the new speed.
void stepBank9Speed() {
  printKeypadAction("SPEED NEXT");
  const SpeechSpeed next = (SpeechSpeed)(((uint8_t)g_speechSpeed + 1) % SPEECH_SPEED_COUNT);
  applySpeechSpeed(next);
  saveSpeechSpeedToNvs((uint8_t)next);
  printKeypadStatus("SPEED {}", speechSpeedName(next));
  speakSpeechSpeed();
  speakValueOk();
}

void resetBank9Profile() {
  printKeypadAction("PROFILE RESET");
  resetCurrentConnection();
  const ConnectionProfile& link = currentConnectionProfile();
  if (currentProtocolType() == PROTO_CIV) {
    char hex[3] = "";
    formatHexByte(link.civAddr, hex, sizeof(hex));
    printKeypadStatus("BAUD {} CI {}", link.baud, hex);
  } else {
    printKeypadStatus("BAUD {}", link.baud);
  }
  speakProfileReset();
}

void queryBank9Profile() {
  printKeypadAction("PROFILE?");
  printKeypadStatus("PROFILE CURRENT");
  if (!g_speechEnabled) return;
  speakLabel("profile");
  speakProfileIdentityFromSlot(g_profileId, false);
}

