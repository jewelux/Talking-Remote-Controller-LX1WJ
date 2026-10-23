#pragma once

// Radio settings as HamTRC names them, whatever the protocol. No Arduino.h, so the host tests
// can build what uses them.

#include <stdint.h>

// AGC speeds. Not every radio has each: the Icoms have FAST, MID and SLOW, the FT-8x7 OFF, FAST,
// SLOW and AUTO.
enum class AgcMode : uint8_t { Off, Fast, Mid, Slow, Auto };

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
