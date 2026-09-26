#pragma once

#include "radio_globals.h"

// Helpers shared by the keypad UI files.

void printKeypadStatus(const String& line);
// Serial "CMD <line>" trace of a keypad action.
void printKeypadCommand(const String& line);

// Keypad feedback. Each prints a status line and gives the matching audio cue.
// Radio gave no answer: say "timeout" and return true. Otherwise return false, so
// the caller keeps its own handling of the failure (unsupported, rejected).
bool keypadReportIfTimedOut(const char* label);
// The key has no action here: short beep.
void keypadReportUnassigned(const String& label);
// When supported is false the key's feature is missing on this profile: beep like
// an unassigned key and return true.
bool keypadReportIfUnsupported(bool supported, const char* label);
// The key is hidden on the FTDX10 layout: say "not available".
void reportFtdx10HiddenKey(const char* label);

bool isFtdx10KeypadProfile();
bool isFt8x7Keypad();
bool isFt8x7Ft817Keypad();
bool isFt8x7Ft857FamilyKeypad();

// The FT-817 and FT-857/897 keypad workflows track VFO A/B locally. Until a
// toggle or query tells otherwise they assume VFO A.
void ensureFt8x7VfoTrackingInitialized();
// Tracked VFO letter ('A' or 'B') on FT-817 and FT-857/897, '?' otherwise.
char ft8x7CurrentVfoLabel();
char ft8x7OtherVfoLabel();

void formatHexByte(uint8_t value, char* out, size_t outSize);
void speakHexNibble(char c);
void speakCivAddressValue(uint8_t addr, bool ok);

void formatCtcssTenthsLabel(uint16_t toneTenths, char* out, size_t outSize);
// BCD encoding for the FT-8x7 CAT tone and DCS commands.
bool encodeCtcssTenths(uint16_t toneTenths, uint8_t& b0, uint8_t& b1);
bool encodeDcsCode(uint16_t dcsCode, uint8_t& b0, uint8_t& b1);

void speakFrequencyWord();
void speakBinaryFeatureState(const uint8_t* featureData, size_t featureLen, bool on);
void speakNotchCycleState(bool on, NotchWidth width);
