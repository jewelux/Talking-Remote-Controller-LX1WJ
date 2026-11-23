#pragma once

#include <Arduino.h>

// The receive filter console commands (radio_features.h rxFilterLevel*, rxFilterSwitch*): IFSHIFT,
// WIDTH, NAR, NOTCHFREQ, CONTOUR, CONTOURFREQ, APF, APFFREQ, HPF, LPF, HPFSLOPE and LPFSLOPE. Each
// reads, sets, or steps the setting, and says "unsupported" where the radio has none.
bool handleConsoleRxFilter(const String& line, const String& upper);
// The help lines for what the radio has.
void printRxFilterHelp();
