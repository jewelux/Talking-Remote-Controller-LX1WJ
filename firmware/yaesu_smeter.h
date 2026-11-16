#pragma once

// Yaesu FTDX family S-meter (CAT "SM0;", answer SM0 000..255), without Arduino so the host
// tests can check it (tests/yaesumeter).
//
// The CAT manuals give only the raw range 000..255. The scales below are Hamlib's str_cal
// tables (newcat.c yaesu_default_str_cal, ftdx101.h, ft891.h): raw value against dB relative
// to S9, 6 dB per S unit below S9. Between two points the value is interpolated.
//   FTDX10:      S9 = 130, S9+60 dB = 255 (Hamlib's generic newcat table; the FTDX10 has none
//                of its own)
//   FTDX101D/MP: S9 = 160, S9+60 dB = 255
//   FT-891:      S9 = 130, S9+60 dB = 255
// Not yet checked against a radio's own meter.

#include <stdint.h>

struct SMeterCalPoint {
  int16_t raw;
  int16_t dbRelS9;  // -54 = S0, 0 = S9, 60 = S9+60 dB
};

struct SMeterScale {
  const SMeterCalPoint* points = nullptr;
  uint8_t count = 0;

  constexpr SMeterScale() = default;
  template <unsigned N>
  constexpr SMeterScale(const SMeterCalPoint (&list)[N]) : points(list), count((uint8_t)N) {}
};

extern const SMeterScale kYaesuSMeterScaleFtdx10;
extern const SMeterScale kYaesuSMeterScaleFtdx101;
extern const SMeterScale kYaesuSMeterScaleFt891;

// dB relative to S9 for a raw value, interpolated on the scale and clamped to its ends.
int16_t smeterScaleDbRelS9(const SMeterScale& scale, int32_t raw);

// S units 0..9 and, at S9, the dB over S9 rounded to 10 dB (0, 10, ... 60), like the Icom
// decoder (civ_smeter.h).
void smeterScaleDecode(const SMeterScale& scale, int32_t raw, uint8_t& sUnitsOut,
                       uint8_t& dbOverS9Out);
