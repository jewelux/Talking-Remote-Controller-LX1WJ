#pragma once

#include "radio_globals.h"

bool isValidProfileId(uint8_t id);
const StoredProfile* storedProfileForId(uint8_t id);
const StoredProfile& currentStoredProfile();
const ConnectionProfile& currentConnectionProfile();
ProtocolType currentProtocolType();
const char* currentProfileVariant();
bool currentProfileVariantIs(const char* variant);
