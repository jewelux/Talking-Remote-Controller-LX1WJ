#pragma once

// The radios HamTRC knows, one built-in profile per slot. To add a radio, add
// an entry to kProfiles in radio_profile_table.cpp. No Arduino.h.

#include "radio_profile_types.h"

// The profile used when none has been picked, or the saved one is gone.
static constexpr uint8_t kDefaultProfileSlot = 1;

// The profile in slot, or nullptr for a free slot.
const RadioProfile* profileForSlot(uint8_t slot);
// The profiles in slot order: profileAt(0) .. profileAt(profileCount() - 1).
size_t profileCount();
const RadioProfile& profileAt(size_t index);
// The next (direction > 0) or previous (direction < 0) used slot after slot,
// wrapping around. slot itself when direction is 0.
uint8_t adjacentProfileSlot(uint8_t slot, int direction);

// "FT-817", "generic", ...: for PROFILE? and the FT-8x7 status commands.
const char* radioModelName(RadioModel model);
