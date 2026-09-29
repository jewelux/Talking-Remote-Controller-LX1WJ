#pragma once

#include <Arduino.h>

const char* modeToString(uint8_t mode);
// Says "mode", then the mode name.
void speakMode(uint8_t mode);
// Says only the mode name, for replies where the context is already clear.
void speakModeName(uint8_t mode);
