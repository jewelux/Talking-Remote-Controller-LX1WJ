#pragma once

// The spoken name of a keypad trace label: "NRLEVEL?" is "noisereduction
// level". A key's answer says it first (KeypadAnswer), and the keypad settings
// (keypad_settings.h) are named by it. Pure C++ with no Arduino.h, so the host
// tests check that every setting and command has a name.

#include <stddef.h>

// The clip tokens for label, with or without its trailing '?', or nullptr when
// the table has no row for it.
const char* spokenLabelWords(const char* label);

// The label of a console command a key runs: no '?', and no last word the reply
// says itself (ON, OFF, TOGGLE or a number). "LOCK TOGGLE" is "LOCK", "BSTACK? 2"
// "BSTACK", "PS ON" "PS", "VFOA MODE?" "VFOA MODE". Cut to fit out.
void keypadCommandLabel(const char* command, char* out, size_t outSize);
