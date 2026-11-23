#pragma once

// Console helpers shared by the ui_console*.cpp files (defined in ui_console.cpp); not for other
// callers.

#include "radio_features.h"
#include "radio_globals.h"

// A shared feature operation (radio_features.h) that did not succeed: prints why, with the speech
// the other console commands give. Ok: false.
bool reportFeatureFailure(FeatureStatus status, const char* label);
// Prints message and says "not available".
void reportNotAvailable(const char* message);
// "<prefix><n>" with n an optional sign and digits, e.g. "RIT STEP -100".
bool parseStepArg(const String& line, const String& upper, const char* prefix, int32_t& stepOut);
// "<prefix><n>" with n a whole number lo..hi, e.g. "NBLEVEL 50".
bool parseRangedIntArg(const String& upper, const char* prefix, long lo, long hi, long& out);
