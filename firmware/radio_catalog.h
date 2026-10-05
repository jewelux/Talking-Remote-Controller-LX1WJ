#pragma once

// The active radio: a copy of its built-in profile, and the link in use.

#include "ft8x7_model.h"
#include "radio_globals.h"

// Makes profile the active radio, linked at baud and, on CI-V, civAddr.
void selectActiveProfile(const RadioProfile& profile, uint32_t baud, uint8_t civAddr);
// The active profile. With g_experimentalCaps, every capability is on.
const RadioProfile& currentProfile();
// Sets g_experimentalCaps and turns the active profile's capabilities to match.
void setExperimentalCaps(bool on);
const ConnectionProfile& currentConnectionProfile();
ProtocolType currentProtocolType();
RadioModel currentRadioModel();
// The FT-8x7 model of the active profile; None for a profile of another protocol.
Ft8x7Model currentFt8x7Model();
// FT-817 or FT-818.
bool currentIsFt817Family();
// FT-857 or FT-897.
bool currentIsFt857Family();
