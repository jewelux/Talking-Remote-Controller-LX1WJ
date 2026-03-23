#include "sd_slots.h"
#include "sd_profile_parser.h"

static const char* kSdCardDir = "/SDCard";
static const char* kSlotsFilePath = "/SDCard/slots.ini";

static String buildSdProfilePath(const String& file) {
  if (!file.length()) return String();
  if (file.startsWith("/")) return file;
  return String(kSdCardDir) + "/" + file;
}

bool loadProfilesFromSd() {
  if (!initSdProfiles()) {
    Serial.println("[SD] init failed");
    return false;
  }

  File f = SD.open(kSlotsFilePath);
  if (!f) {
    Serial.print("[SD] missing ");
    Serial.println(kSlotsFilePath);
    return false;
  }

  Serial.print("[SD] loaded ");
  Serial.println(kSlotsFilePath);
  bool any = false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("#") || line.startsWith(";") || line.startsWith("[")) continue;

    int eq = line.indexOf('=');
    if (eq < 0) continue;

    int slot = line.substring(0, eq).toInt();
    String file = line.substring(eq + 1);
    file.trim();
    if (slot < 1 || slot > MAX_PROFILE_SLOTS || !file.length()) continue;

    String path = buildSdProfilePath(file);
    StoredProfile sp;
    if (loadSingleProfileIni(path, sp)) {
      g_slotProfiles[slot - 1] = sp;
      g_slotProfiles[slot - 1].civ.name = g_slotProfiles[slot - 1].name;
      Serial.print("[SD] slot ");
      Serial.print(slot);
      Serial.print(" <- ");
      Serial.println(file);
      any = true;
    }
  }

  f.close();
  return any;
}

void printProfileSlots() {
  Serial.println("[SLOTS]");
  for (uint8_t i = 0; i < MAX_PROFILE_SLOTS; ++i) {
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(" -> ");
    if (g_slotProfiles[i].valid) Serial.println(g_slotProfiles[i].civ.name);
    else Serial.println("(empty)");
  }
}
