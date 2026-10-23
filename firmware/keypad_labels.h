#pragma once

// The spoken name of a keypad trace label: "NRLEVEL?" is "noisereduction
// level". Verbose speech says it before "not available", "timeout" or "error",
// and the keypad settings (keypad_settings.h) are named by it. Pure C++ with no
// Arduino.h, so the host tests check that every setting has a name.

// The clip tokens for label, with or without its trailing '?', or nullptr when
// the table has no row for it.
const char* spokenLabelWords(const char* label);
