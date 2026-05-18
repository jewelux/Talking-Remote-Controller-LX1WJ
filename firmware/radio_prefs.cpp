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
