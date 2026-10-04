#pragma once

#include "radio_globals.h"

uint8_t loadProfileFromNvs(uint8_t fallback);
void saveProfileToNvs(uint8_t id);
bool loadTuningSpeakFromNvs(bool fallback);
void saveTuningSpeakToNvs(bool v);
bool loadVerboseFromNvs(bool fallback);
void saveVerboseToNvs(bool v);
uint8_t loadVolumeFromNvs(uint8_t fallback);
void saveVolumeToNvs(uint8_t level);
// The CI-V address and baud the user picked for the profile in slot id. Leaves
// civAddr and baud as they are, and returns false, when none were saved.
bool loadConnectionOverrideFromNvs(uint8_t id, uint8_t& civAddr, uint32_t& baud);
void saveConnectionOverrideToNvs(uint8_t id, uint8_t civAddr, uint32_t baud);
