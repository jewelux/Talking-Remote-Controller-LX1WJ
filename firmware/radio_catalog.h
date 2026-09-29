#pragma once

#include "radio_globals.h"

bool isValidProfileId(uint8_t id);
const StoredProfile* storedProfileForId(uint8_t id);
// With g_experimentalCaps, a copy of the active profile with every capability on.
const StoredProfile& currentStoredProfile();
// Call when the active profile or g_experimentalCaps changes.
void invalidateExperimentalProfile();
const ConnectionProfile& currentConnectionProfile();
ProtocolType currentProtocolType();
const char* currentProfileVariant();
bool currentProfileVariantIs(const char* variant);
