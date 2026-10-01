#pragma once

// The FT-8x7 radio a profile is for, from its ini "variant". Every model check goes through
// this, so a misspelt variant cannot pass for a model in one place and not in another.

#include <stdint.h>
#include <string.h>

enum class Ft8x7Model : uint8_t {
  None,  // not an FT-8x7 profile, or a variant HamTRC does not know
  Ft817,
  Ft818,
  Ft857,  // FT-857 and FT-897, which share a CAT and an EEPROM map
};

inline Ft8x7Model ft8x7ModelForVariant(const char* variant) {
  if (!variant) return Ft8x7Model::None;
  if (!strcmp(variant, "ft817")) return Ft8x7Model::Ft817;
  if (!strcmp(variant, "ft818")) return Ft8x7Model::Ft818;
  if (!strcmp(variant, "ft857_897")) return Ft8x7Model::Ft857;
  return Ft8x7Model::None;
}

// The FT-818 is an FT-817 with more power; everything else is the same.
inline bool ft8x7IsFt817Family(Ft8x7Model model) {
  return model == Ft8x7Model::Ft817 || model == Ft8x7Model::Ft818;
}
