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

static StoredProfile s_experimentalProfile;
static const StoredProfile* s_experimentalSource = nullptr;

void invalidateExperimentalProfile() {
  s_experimentalSource = nullptr;
}

static const StoredProfile& withAllCaps(const StoredProfile& base) {
  if (s_experimentalSource != &base) {
    s_experimentalProfile = base;
    bool* flags = reinterpret_cast<bool*>(&s_experimentalProfile.caps);
    for (size_t i = 0; i < sizeof(RadioCapabilities) / sizeof(bool); ++i) flags[i] = true;
    s_experimentalSource = &base;
  }
  return s_experimentalProfile;
}

const StoredProfile& currentStoredProfile() {
  const StoredProfile* sp = storedProfileForId(g_profileId);
  const StoredProfile& base = sp ? *sp : g_slotProfiles[0];
  return g_experimentalCaps ? withAllCaps(base) : base;
}

const ConnectionProfile& currentConnectionProfile() {
  return currentStoredProfile().connection;
}

ProtocolType currentProtocolType() {
  return currentStoredProfile().protocolType;
}

const char* currentProfileVariant() {
  return currentStoredProfile().variant;
}

bool currentProfileVariantIs(const char* variant) {
  if (!variant) return false;
  return strcmp(currentStoredProfile().variant, variant) == 0;
}
