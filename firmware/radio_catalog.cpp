#include "radio_catalog.h"

#include "radio_globals.h"

bool isValidProfileId(uint8_t id) {
  return id >= 1 && id <= MAX_PROFILE_SLOTS;
}

const StoredProfile* storedProfileForId(uint8_t id) {
  if (!isValidProfileId(id)) return nullptr;
  const StoredProfile& sp = g_slotProfiles[id - 1];
  return sp.valid ? &sp : nullptr;
}

const StoredProfile& currentStoredProfile() {
  const StoredProfile* sp = storedProfileForId(g_profileId);
  return sp ? *sp : g_slotProfiles[0];
}

const CivProfile& currentProfile() {
  return currentStoredProfile().civ;
}

ProtocolType currentProtocolType() {
  return currentStoredProfile().protocolType;
}
