#include "radio_prefs.h"

#include "radio_catalog.h"

uint8_t loadProfileFromNvs(uint8_t fallback) {
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return fallback;
  uint8_t v = prefs.getUChar("profile", fallback);
  prefs.end();
  return isValidProfileId(v) ? v : fallback;
}

void saveProfileToNvs(uint8_t id) {
  if (!isValidProfileId(id)) return;
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

static String connectionKey(const char* prefix, uint8_t id) {
  return String(prefix) + String((int)id);
}

bool loadConnectionOverrideFromNvs(uint8_t id, uint8_t& civAddr, uint32_t& baud) {
  if (!isValidProfileId(id)) return false;
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
  if (!isValidProfileId(id)) return;
  Preferences prefs;
  if (!prefs.begin("talkingrc", false)) return;
  String civKey = connectionKey("civ", id);
  String baudKey = connectionKey("baud", id);
  prefs.putUChar(civKey.c_str(), civAddr);
  prefs.putULong(baudKey.c_str(), baud);
  prefs.end();
}

void applyConnectionOverridesFromNvs() {
  for (uint8_t id = 1; id <= MAX_PROFILE_SLOTS; ++id) {
    StoredProfile& sp = g_slotProfiles[id - 1];
    if (!sp.valid || sp.protocolType != PROTO_CIV) continue;
    uint8_t civAddr = sp.civ.civAddr;
    uint32_t baud = sp.civ.baud;
    if (!loadConnectionOverrideFromNvs(id, civAddr, baud)) continue;
    sp.civ.civAddr = civAddr;
    sp.civ.baud = baud;
  }
}
