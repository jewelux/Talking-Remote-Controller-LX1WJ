#pragma once

// Meter scales as tables: a raw meter answer against the value it stands for, interpolated
// between points and clamped to the ends. A radio profile carries one per meter
// (RadioProfile::smeterScale, ::swrScale); the tables themselves sit with the profiles in
// radio_profile_table.cpp. Without Arduino so the host tests can check it (tests/meters).

#include <stddef.h>
#include <stdint.h>

// -54 dB relative to S9 is S0.
inline constexpr int16_t kSMeterDbRelS9AtS0 = -54;

// value is in the unit of the scale it belongs to: dB relative to S9 on an S-meter scale
// (-54 = S0, 0 = S9, 60 = S9+60 dB), SWR x 100 on an SWR scale (150 = 1.5).
struct MeterCalPoint {
  int32_t raw;
  int16_t value;
};

// A radio's calibration points, raw values ascending. Empty when the radio has no such meter
// or its protocol decodes it itself.
struct MeterScale {
  const MeterCalPoint* points = nullptr;
  uint8_t count = 0;

  constexpr MeterScale() = default;
  template <size_t N>
  constexpr MeterScale(const MeterCalPoint (&list)[N]) : points(list), count((uint8_t)N) {
    static_assert(N >= 2 && N <= 255, "a meter scale needs 2..255 points");
  }

  constexpr bool empty() const { return count == 0; }

  // Raw values strictly ascending and values never falling, so interpolation is well defined.
  constexpr bool ascends() const {
    for (uint8_t i = 1; i < count; ++i) {
      if (points[i].raw <= points[i - 1].raw) return false;
      if (points[i].value < points[i - 1].value) return false;
    }
    return true;
  }
};

// ---- S-meter (value = dB relative to S9) ----

// dB relative to S9 for a raw value, interpolated on the scale and clamped to its ends.
int16_t smeterScaleDbRelS9(const MeterScale& scale, int32_t raw);

// S units 0..9 and, at S9, the dB over S9 rounded to 10 dB (0, 10, ... 60), like the Icom
// decoder (civ_smeter.h).
void smeterScaleDecode(const MeterScale& scale, int32_t raw, uint8_t& sUnitsOut,
                       uint8_t& dbOverS9Out);

// ---- SWR (value = SWR x 100) ----

// SWR in tenths (15 = 1.5), rounded once from the interpolated value. 10 (1.0) on an empty
// scale.
uint16_t swrTenthsFromRaw(const MeterScale& scale, int32_t raw);
