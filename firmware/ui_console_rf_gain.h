#pragma once

#include <Arduino.h>

// The RF gain console commands (radio_features.h rfGain*): RFGAIN? | RFGAIN <0..100> |
// RFGAIN STEP <+-n>, in percent of the radio's range. "unsupported" where the radio has none.
bool handleConsoleRfGain(const String& line, const String& upper);
// The help line for what the radio has; none where it has no RF gain.
void printRfGainHelp();
