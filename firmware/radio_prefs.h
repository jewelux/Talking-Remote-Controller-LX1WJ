#pragma once

#include "radio_globals.h"

uint8_t loadProfileFromNvs(uint8_t fallback);
void saveProfileToNvs(uint8_t id);
bool loadTuningSpeakFromNvs(bool fallback);
void saveTuningSpeakToNvs(bool v);
