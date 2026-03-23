#pragma once

#include "radio_globals.h"

bool initSdProfiles();
bool loadSingleProfileIni(const String& path, StoredProfile& out);
