#pragma once

#include "keypad_actions.h"
#include "keypad_input.h"
#include "radio_globals.h"

// Helpers shared by the keypad UI files.

// The keypad state machine (ui_keypad.cpp). Actions call it to start an entry,
// mode select or profile select.
KeypadInput& keypadInput();

// Runs a console command for a key, with the keypad's polling and speech holds.
void keypadSendNow(const String& cmd);
// Keeps cmd for the next Enter.
void keypadStageCommand(const String& cmd);
void speakKeypadCommandWord(const String& cmd);

// Entries and selections (ui_keypad_entry.cpp), called by the state machine.
void keypadEntryDigit(InputMode mode, char key, const char* digits);
void keypadEntryCommit(InputMode mode, const char* digits, uint8_t targetVfo);
bool keypadModeDigit(char key, uint8_t& mode);
void keypadModeCommit(uint8_t mode, uint8_t targetVfo);
// Shared frequency writer for entry commit and the round-to-500 Hz action.
// targetVfo is one of the KEYPAD_VFO_* values.
bool keypadApplyFrequencyHz(uint64_t hz, uint8_t targetVfo);

// No SD card profiles: Bank 9 digits pick the built-in light-Icom profiles.
bool lightIcomFallbackActive();
// Holds polling and tuning speech while a key's answer is prepared.
void prepareKeypadSpeechResponse();
// FT-8x7 with the dial lock on: say "lock on" and return false.
bool guardFt8x7VfoToggleLock();
void speakSimpleBinaryState(bool on);
void speakQueriedFrequencyHz(uint64_t hz);
void speakFeatureValue(const uint8_t* featureData, size_t featureLen, uint8_t value);
uint8_t levelRawToPercent(uint16_t raw);
uint16_t levelPercentToRaw(int percent);

void printKeypadStatus(const String& line);
// Serial "CMD <line>" trace.
void printKeypadCommand(const String& line);
// Serial trace of a bank key action: "CMD <key> -> <what>", the key being
// keymapActiveKey() (e.g. "BANK3 2 LONG"). Just "CMD <what>" outside the keymap.
void printKeypadAction(const String& what);

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
void reportFtdx10HiddenKey();

// The radio checks behind KeypadTraits::layout. Bank actions do not use them:
// the keymap already picked the action for the radio.
bool isFtdx10KeypadProfile();
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

void speakFrequencyWord();
void speakBinaryFeatureState(const uint8_t* featureData, size_t featureLen, bool on);
void speakNotchCycleState(bool on, NotchWidth width);
