#pragma once

// Radio settings as HamTRC names them, whatever the protocol. No Arduino.h, so the host tests
// can build what uses them.

#include <stdint.h>

// AGC speeds. Not every radio has each: the Icoms have FAST, MID and SLOW, the FT-8x7 OFF, FAST,
// SLOW and AUTO.
enum class AgcMode : uint8_t { Off, Fast, Mid, Slow, Auto };

// CW break-in. On is break-in where CAT has no semi or full (Yaesu BI, the FT-8x7 menu).
enum class BreakInMode : uint8_t { Off, On, Semi, Full };

// The CW levels HamTRC reads and sets: the keyer speed in WPM, the pitch in Hz, the break-in delay
// in ms (0 is full break-in where the radio has it).
enum class CwLevel : uint8_t { Speed, Pitch, Delay };

// The receive DSP levels: noise reduction and noise blanker.
enum class RxLevel : uint8_t { Nr, Nb };

// The receive filter values (rx_filter_codec.h, radio_features.h rxFilterLevel*): the IF shift,
// the width, the manual notch, contour and APF frequencies and the low and high cut in Hz, the
// cut slopes in dB/oct. A width of 0 is the radio's default (NORMAL), a cut of 0 is off.
enum class RxFilterLevel : uint8_t {
  IfShift, Width, NotchFreq, ContourFreq, ApfFreq, LowCut, HighCut, LowCutSlope, HighCutSlope,
};

// The receive filter switches: narrow, contour and APF (audio peak filter).
enum class RxFilterSwitch : uint8_t { Narrow, Contour, Apf };

// How a radio sends the receive filter settings its caps name (rx_filter_codec.h).
enum class RxFilterEncoding : uint8_t {
  None,
  Icom,          // 1A 03, the width as an index of the current filter (IC-7300, IC-705, IC-7760)
  YaesuFtdx10,   // SH00nn, IS00+nnnn, NA0, BP0, CO0, the EX cut menus of the FTDX10
  YaesuFtdx101,  // the same but the SSB width table and the EX menu numbers (main band)
  YaesuFt891,    // IS0 with an on digit, its 4-digit EX cut menus; no width (it depends on NA)
  KenwoodTs480,  // SL and SH, the slope tune cuts in SSB, FM and AM
};

// The repeater shift: simplex, or the TX frequency above or below.
enum class RptShift : uint8_t { Simplex, Plus, Minus };

// How an ASCII radio's GT sets AGC (cat_setting_codec.h). The Icoms all use 16 12.
enum class AgcEncoding : uint8_t {
  None,
  YaesuGt0,    // GT0n: 0 off, 1 fast, 2 mid, 3 slow, 4 auto; read 4..6 is auto (FTDX10/101, FT-891)
  KenwoodGt,   // GTnnn: 000 off, 001 fast, 002 slow (TS-480)
  ElecraftGt,  // GTnnn: 002 fast, 004 slow (KX2)
};

// How an ASCII radio's PA and RA read and set the preamp and the attenuator. The Icoms all use
// 16 02 and 11.
enum class FrontEndEncoding : uint8_t {
  None,
  Yaesu,    // PA0n, RA0n (FTDX10/101, FT-891)
  Kenwood,  // PAn, RAnn (TS-480, KX2); the TS-480 answers PAn0 and RAnn00
};

// How an ASCII radio's PC reads and sets the RF power. The Icoms all use 14 0A.
enum class RfPowerEncoding : uint8_t {
  None,
  PcWatts,  // PCnnn in watts (FTDX10, FTDX101D, FT-891, TS-480, KX2); the KX2 may answer PCnnnx
};

// How a radio reads and sets the RF gain (cat_setting_codec.h rfGainScaleFor). HamTRC shows it in
// percent of the radio's range on every radio.
enum class RfGainEncoding : uint8_t {
  None,
  Civ,           // 14 02, 0000..0255 in BCD (IC-7300, IC-705, IC-7760)
  YaesuRg0,      // RG0nnn, 000..255 (FTDX10; FTDX101 main band)
  YaesuRg0Ft891, // RG0nnn, 000..030 (FT-891)
  KenwoodRg,     // RGnnn, 000..100 (TS-480)
  ElecraftRg,    // RGnnn, 190..250 = -60..0 dB (KX2)
};
