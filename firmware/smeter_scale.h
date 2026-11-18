#pragma once

// S-meter scale as a table: raw meter answer against dB relative to S9, interpolated between
// points, 6 dB per S unit below S9. A radio profile carries its table (RadioProfile::smeterScale);
// the tables themselves sit with the profiles in radio_profile_table.cpp. Without Arduino so
// the host tests can check it (tests/smeter).

#include <stddef.h>
#include <stdint.h>

// -54 dB relative to S9 is S0.
inline constexpr int16_t kSMeterDbRelS9AtS0 = -54;

struct SMeterCalPoint {
  int32_t raw;
  int16_t dbRelS9;  // -54 = S0, 0 = S9, 60 = S9+60 dB
};

// A radio's calibration points, raw values ascending. Empty on radios whose protocol decodes
// the S-meter itself.
struct SMeterScale {
  const SMeterCalPoint* points = nullptr;
  uint8_t count = 0;

  constexpr SMeterScale() = default;
  template <size_t N>
  constexpr SMeterScale(const SMeterCalPoint (&list)[N]) : points(list), count((uint8_t)N) {
    static_assert(N >= 2 && N <= 255, "an S-meter scale needs 2..255 points");
  }

  constexpr bool empty() const { return count == 0; }

  // Raw values strictly ascending and dB never falling, so interpolation is well defined.
  constexpr bool ascends() const {
    for (uint8_t i = 1; i < count; ++i) {
      if (points[i].raw <= points[i - 1].raw) return false;
      if (points[i].dbRelS9 < points[i - 1].dbRelS9) return false;
    }
    return true;
  }
};

// dB relative to S9 for a raw value, interpolated on the scale and clamped to its ends.
int16_t smeterScaleDbRelS9(const SMeterScale& scale, int32_t raw);

// S units 0..9 and, at S9, the dB over S9 rounded to 10 dB (0, 10, ... 60), like the Icom
// decoder (civ_smeter.h).
void smeterScaleDecode(const SMeterScale& scale, int32_t raw, uint8_t& sUnitsOut,
                       uint8_t& dbOverS9Out);
