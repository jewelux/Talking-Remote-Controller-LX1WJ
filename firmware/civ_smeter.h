#pragma once

// Icom CI-V S-meter (command 0x15 0x02), without Arduino so the host tests can check it
// (tests/civmeter).
//
// The radio answers 0..255 (BCD 0000..0255). Icom's scale, as in the IC-7300, IC-705 and
// IC-7760 CI-V references and Hamlib's str_cal for these radios:
//   0 = S0, 120 = S9, 241 = S9+60 dB.
// Below S9 the scale is linear in S units (120 / 9 per unit); above S9 it is linear in dB
// (121 / 60 per dB). The IC-706 is assumed to use the same scale; not yet checked on a radio.

#include <stdint.h>

static constexpr int32_t CIV_SMETER_RAW_S9 = 120;
static constexpr int32_t CIV_SMETER_RAW_S9_PLUS_60 = 241;

// S units 0..9 and, at S9, the dB over S9 rounded to 10 dB (0, 10, ... 60).
void civDecodeSMeter(int32_t raw, uint8_t& sUnitsOut, uint8_t& dbOverS9Out);
