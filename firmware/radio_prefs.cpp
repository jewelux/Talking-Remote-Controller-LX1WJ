#include "radio_prefs.h"

#include "radio_profile_table.h"

uint8_t loadProfileFromNvs(uint8_t fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  uint8_t v = prefs.getUChar("profile", fallback);
  prefs.end();
  return profileForSlot(v) ? v : fallback;
}

void saveProfileToNvs(uint8_t id) {
  if (!profileForSlot(id)) return;
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putUChar("profile", id);
  prefs.end();
}

bool loadTuningSpeakFromNvs(bool fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  bool v = prefs.getBool("tuningspk", fallback);
  prefs.end();
  return v;
}

void saveTuningSpeakToNvs(bool v) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putBool("tuningspk", v);
  prefs.end();
}

bool loadVerboseFromNvs(bool fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  bool v = prefs.getBool("verbose", fallback);
  prefs.end();
  return v;
}

void saveVerboseToNvs(bool v) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putBool("verbose", v);
  prefs.end();
}

uint8_t loadVolumeFromNvs(uint8_t fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  uint8_t v = prefs.getUChar("volume", fallback);
  prefs.end();
  return (v >= 1 && v <= 9) ? v : fallback;
}

void saveVolumeToNvs(uint8_t level) {
  if (level < 1) level = 1;
  if (level > 9) level = 9;
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putUChar("volume", level);
  prefs.end();
}

uint8_t loadSpeechSpeedFromNvs(uint8_t fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  uint8_t v = prefs.getUChar("speed", fallback);
  prefs.end();
  return v;
}

void saveSpeechSpeedToNvs(uint8_t speed) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putUChar("speed", speed);
  prefs.end();
}

ToneDir loadToneDirFromNvs(ToneDir fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  uint8_t v = prefs.getUChar("tonedir", (uint8_t)fallback);
  prefs.end();
  return v <= (uint8_t)ToneDir::RxOnly ? (ToneDir)v : fallback;
}

void saveToneDirToNvs(ToneDir dir) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putUChar("tonedir", (uint8_t)dir);
  prefs.end();
}

static String connectionKey(const char* prefix, uint8_t id) {
  return String(prefix) + String((int)id);
}

bool loadConnectionOverrideFromNvs(uint8_t id, uint8_t& civAddr, uint32_t& baud) {
  if (!profileForSlot(id)) return false;
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return false;
  String civKey = connectionKey("civ", id);
  String baudKey = connectionKey("baud", id);
  bool hasCiv = prefs.isKey(civKey.c_str());
  bool hasBaud = prefs.isKey(baudKey.c_str());
  if (hasCiv) civAddr = prefs.getUChar(civKey.c_str(), civAddr);
  if (hasBaud) baud = prefs.getULong(baudKey.c_str(), baud);
  prefs.end();
  return hasCiv || hasBaud;
}

void saveConnectionOverrideToNvs(uint8_t id, uint8_t civAddr, uint32_t baud) {
  if (!profileForSlot(id)) return;
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  String civKey = connectionKey("civ", id);
  String baudKey = connectionKey("baud", id);
  prefs.putUChar(civKey.c_str(), civAddr);
  prefs.putULong(baudKey.c_str(), baud);
  prefs.end();
}

void clearConnectionOverrideInNvs(uint8_t id) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  const String civKey = connectionKey("civ", id);
  const String baudKey = connectionKey("baud", id);
  if (prefs.isKey(civKey.c_str())) prefs.remove(civKey.c_str());
  if (prefs.isKey(baudKey.c_str())) prefs.remove(baudKey.c_str());
  prefs.end();
}

bool loadNanoVnaSetupFromNvs(uint32_t& startHz, uint32_t& stopHz, int8_t& calSlot) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return false;
  const bool saved = prefs.isKey("vnastart") && prefs.isKey("vnastop") && prefs.isKey("vnacal");
  if (saved) {
    startHz = prefs.getULong("vnastart", startHz);
    stopHz = prefs.getULong("vnastop", stopHz);
    calSlot = prefs.getChar("vnacal", calSlot);
  }
  prefs.end();
  return saved;
}

void saveNanoVnaSetupToNvs(uint32_t startHz, uint32_t stopHz, int8_t calSlot) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  prefs.putULong("vnastart", startHz);
  prefs.putULong("vnastop", stopHz);
  prefs.putChar("vnacal", calSlot);
  prefs.end();
}
