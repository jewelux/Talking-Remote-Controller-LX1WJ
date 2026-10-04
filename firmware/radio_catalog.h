#pragma once

#include "ft8x7_model.h"
#include "radio_globals.h"

bool isValidProfileId(uint8_t id);
const RadioProfile* storedProfileForId(uint8_t id);
// With g_experimentalCaps, a copy of the active profile with every capability on.
const RadioProfile& currentProfile();
// Call when the active profile or g_experimentalCaps changes.
void invalidateExperimentalProfile();
const ConnectionProfile& currentConnectionProfile();
ProtocolType currentProtocolType();
const char* currentProfileVariant();
// The FT-8x7 model of the active profile; None for a profile of another protocol.
Ft8x7Model currentFt8x7Model();
// FT-817 or FT-818.
bool currentIsFt817Family();
// FT-857 or FT-897.
bool currentIsFt857Family();
