#pragma once

#include "radio_globals.h"

void keypadClearAll();
void keypadStageCommand(const String& cmd);
void keypadSendNow(const String& cmd);
void keypadEnter();
bool keypadApplyFrequencyHz(uint64_t hz, uint8_t targetVfo);
void keypadHandleReleased(char k);
void keypadBank2QueryNr();
void keypadBank2QueryNb();
void keypadBank2QueryNotch();

// Keypad feedback shared by ui_keypad.cpp and ui_keypad_actions.cpp. Each prints a
// status line and gives the matching audio cue.
// Radio gave no answer: say "timeout" and return true. Otherwise return false, so
// the caller keeps its own handling of the failure (unsupported, rejected).
bool keypadReportIfTimedOut(const char* label);
// The key has no action here: short beep.
void keypadReportUnassigned(const String& label);
// When supported is false the key's feature is missing on this profile: beep like
// an unassigned key and return true.
bool keypadReportIfUnsupported(bool supported, const char* label);
