#pragma once

#include "keypad_actions.h"
#include "keypad_input.h"
#include "radio_features.h"
#include "radio_globals.h"

// Helpers shared by the keypad UI files.

// What an action may ask of the keypad state machine (ui_keypad.cpp): the keys
// after it go to an entry, mode select or profile select.
void keypadBeginEntry(InputMode mode, TargetVfo targetVfo = TargetVfo::Current);
void keypadBeginModeSelect(TargetVfo targetVfo);
void keypadBeginProfileSelect();

// Runs a console command for a key, with the keypad's polling and speech holds.
void keypadSendNow(const String& cmd);
// Keeps cmd for the next Enter.
void keypadStageCommand(const String& cmd);
void speakKeypadCommandWord(const String& cmd);

// Entries and selections (ui_keypad_entry.cpp), called by the state machine.
void keypadEntryDigit(const EntrySpec& entry, char key, const char* digits);
void keypadEntryCommit(InputMode mode, const char* digits, TargetVfo targetVfo);
bool keypadModeDigit(char key, uint8_t& mode);
void keypadModeCommit(uint8_t mode, TargetVfo targetVfo);
// Shared frequency writer for entry commit and the round-to-500 Hz action.
bool keypadApplyFrequencyHz(uint64_t hz, TargetVfo targetVfo);

// No SD card profiles: Bank 9 digits pick the built-in light-Icom profiles.
bool lightIcomFallbackActive();
// Holds background polling briefly, so it does not talk over a key's exchange.
void holdKeypadPolling();
// Holds polling and tuning speech while a key's answer is prepared.
void prepareKeypadSpeechResponse();
// Holds polling a little longer while a key writes to the radio and reads it back.
void prepareKeypadRadioWrite();
// FT-8x7 with the dial lock on: say "lock on" and return false.
bool guardFt8x7VfoToggleLock();
void speakSimpleBinaryState(bool on);
// "transceiver rx" or "transceiver tx".
void speakRxTxState(bool tx);
void speakQueriedFrequencyHz(uint64_t hz);
void speakFeatureValue(const uint8_t* featureData, size_t featureLen, uint8_t value);
uint8_t levelRawToPercent(uint16_t raw);
uint16_t levelPercentToRaw(int percent);

void printKeypadStatus(const String& line);
// Serial "CMD <line>" trace.
void printKeypadCommand(const String& line);
// Serial trace of a bank key action: "CMD <key> -> <what>", the key being
// keypadActiveKey() (e.g. "BANK3 2 LONG"). Just "CMD <what>" outside a key action.
void printKeypadAction(const String& what);

// Keypad feedback. Each prints a status line and gives the matching audio cue.
// Radio gave no answer: say "timeout" and return true. Otherwise return false, so
// the caller keeps its own handling of the failure (unsupported, rejected).
bool keypadReportIfTimedOut(const char* label);
// A shared feature operation did not succeed: beep when unsupported, say
// "timeout" or give the error sound otherwise, and return true. Ok: false.
bool keypadReportFeatureFailure(FeatureStatus status, const char* label);
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

// The FT-817 and FT-857/897 keypad workflows track VFO A/B. The FT-857/897 reads the
// active VFO from the radio; the FT-817 assumes VFO A until a toggle or sync says otherwise.
void refreshFt8x7ActiveVfo();
// Tracked VFO letter ('A' or 'B') on FT-817 and FT-857/897, '?' otherwise.
char ft8x7CurrentVfoLabel();
char ft8x7OtherVfoLabel();

void formatHexByte(uint8_t value, char* out, size_t outSize);
void speakHexNibble(char c);
void speakCivAddressValue(uint8_t addr, bool ok);

void formatCtcssTenthsLabel(uint16_t toneTenths, char* out, size_t outSize);

void speakFrequencyWord();
// A short gap, then "please". Ends a prompt such as "mode please".
void speakPlease();
// "<token> please".
void speakPrompt(const char* token);
// "vfo a" or "vfo b" (just "vfo" for any other letter).
void speakVfoLabel(char which);
// "vfo a frequency".
void speakVfoFrequencyLabel(char which);
// "vfo a frequency" and the frequency in MHz.
void speakVfoFrequency(char which, uint64_t hz);
