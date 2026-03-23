#pragma once

#include "radio_globals.h"

bool isValidProfileId(uint8_t id);
const StoredProfile* storedProfileForId(uint8_t id);
const StoredProfile& currentStoredProfile();
const CivProfile& currentProfile();
ProtocolType currentProtocolType();
