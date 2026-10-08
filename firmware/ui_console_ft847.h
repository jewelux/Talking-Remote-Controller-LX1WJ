#pragma once

#include <Arduino.h>

// FT-847 console commands (only while an FT-847 profile is active). Mostly for bringing up
// and testing the radio: raw frames, CAT ON/OFF, byte gap, decoded status bytes.
bool handleConsoleFt847Commands(const String& upper);
void printFt847ConsoleHelp();
