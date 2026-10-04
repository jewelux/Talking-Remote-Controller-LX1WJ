#pragma once

// The FT-8x7 radio a profile is for, from its RadioModel. The FT-8x7 code
// (EEPROM map, VFO handling, keypad layout) asks this, not the RadioModel.

#include <stdint.h>

#include "radio_profile_types.h"

enum class Ft8x7Model : uint8_t {
  None,  // not an FT-8x7 profile
  Ft817,
  Ft818,
  Ft857,  // FT-857 and FT-897, which share a CAT and an EEPROM map
};

inline Ft8x7Model ft8x7ModelFor(RadioModel model) {
  switch (model) {
    case RadioModel::Ft817: return Ft8x7Model::Ft817;
    case RadioModel::Ft818: return Ft8x7Model::Ft818;
    case RadioModel::Ft857: return Ft8x7Model::Ft857;
    default: return Ft8x7Model::None;
  }
}

// The FT-818 is an FT-817 with more power; everything else is the same.
inline bool ft8x7IsFt817Family(Ft8x7Model model) {
  return model == Ft8x7Model::Ft817 || model == Ft8x7Model::Ft818;
}
