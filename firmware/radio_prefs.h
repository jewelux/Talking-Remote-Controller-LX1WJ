#pragma once

#include "radio_globals.h"

uint8_t loadProfileFromNvs(uint8_t fallback);
void saveProfileToNvs(uint8_t id);
bool loadTuningSpeakFromNvs(bool fallback);
void saveTuningSpeakToNvs(bool v);
uint8_t loadVolumeFromNvs(uint8_t fallback);
void saveVolumeToNvs(uint8_t level);
bool loadConnectionOverrideFromNvs(uint8_t id, uint8_t& civAddr, uint32_t& baud);
void saveConnectionOverrideToNvs(uint8_t id, uint8_t civAddr, uint32_t baud);
void applyConnectionOverridesFromNvs();
